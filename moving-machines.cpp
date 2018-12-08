#include "Core.h"
#include "Console.h"
#include "Export.h"
#include "PluginManager.h"
#include "DataDefs.h"
#include "VTableInterpose.h"
#include "MiscUtils.h"

#include "modules/Buildings.h"
#include "modules/Screen.h"
#include "modules/Gui.h"
#include "modules/Burrows.h"

using namespace DFHack;

#include "df/world.h"
#include "df/map_block.h"
#include "df/tiletype.h"
#include "df/coord.h"
#include "df/building.h"
#include "df/building_actual.h"
#include "df/building_constructionst.h"
#include "df/building_workshopst.h"
#include "df/building_def.h"
#include "df/construction.h"
#include "df/machine.h"
#include "df/machine_info.h"
#include "df/power_info.h"
#include "df/unit.h"
#include "df/burrow.h"
#include "df/ui.h"
#include "df/ui_sidebar_mode.h"
#include "df/viewscreen_dwarfmodest.h"

using namespace df::enums;

#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <algorithm>

DFHACK_PLUGIN("moving-machines");
REQUIRE_GLOBAL(world);
REQUIRE_GLOBAL(pause_state);
REQUIRE_GLOBAL(cursor);
REQUIRE_GLOBAL(ui);
REQUIRE_GLOBAL(selection_rect);

static const uint32_t B_FLAG_PLATFORM = 0x100U;
static const uint32_t B_FLAG_CLEARED = 0x200U;
static const uint8_t C_FLAG_PLATFORM = 0x4U;
static const uint8_t C_FLAG_CLEARED = 0x8U;
typedef df::specific_ref_type spec_ref_enum_t;
static const spec_ref_enum_t MV_MACHINE_SPEC_REF = (spec_ref_enum_t)0x1FU;
static const spec_ref_enum_t INFO_SPEC_REF = (spec_ref_enum_t)0x20U;
typedef df::ui_sidebar_mode smode_enum_t;

// Auxiliary functions

int str_to_int(const std::string& s) {
    std::stringstream ss;
    ss.exceptions(std::ios_base::failbit);
    int ret;
    ss << s;
    ss >> ret;
    return ret;
}

std::string int_to_str(int n) {
    std::stringstream ss;
    ss.exceptions(std::ios_base::failbit);
    std::string ret;
    ss << n;
    ss >> ret;
    return ret;
}

df::map_block* get_map_block(df::coord pos) {
	return world->map.block_index[pos.x/16][pos.y/16][pos.z];
}

void destroy_tile(df::coord pos) {
	df::map_block* block = get_map_block(pos);
	block->tiletype[pos.x%16][pos.y%16] = df::enums::tiletype::OpenSpace;
}

void move_building(df::building* b, df::coord translation) {
	b->moveBuilding(translation.x, translation.y, translation.z);
}

void move_machine(df::machine* m, df::coord translation) {
    m->moveMachine(translation.x, translation.y, translation.z);
    for (df::machine::T_components* comp : m->components) {
        df::building* b = df::building::find(comp->building_id);
        move_building(b, translation);
    }
}

void move_unit(df::unit* u, df::coord  translation) {
    u->pos.x += translation.x;
    u->pos.y += translation.y;
    u->pos.z += translation.z;
}

df::machine* get_machine(df::building* b) {
    df::machine_info* info = b->getMachineInfo();
    if (info == nullptr) return nullptr;
    return df::machine::find(info->machine_id);
}

void* get_spec_ref(df::building* b, spec_ref_enum_t type) {
    void* ret = nullptr;
    for (df::specific_ref* r : b->specific_refs) {
        if (r->type == type) {
            ret = r->object;
        }
    }
    return ret;
}

// Core logic

struct MovingMachine {
	static const int32_t TILE_DIMEN = 100000;
    static const int32_t GRAVITY = 5000;
    static const std::set<df::tiletype> OPEN_TILES;

	int32_t offsetx;
	int32_t offsety;
	int32_t offsetz;
	int32_t momentumx;
    int32_t momentumy;
    int32_t momentumz;
    int32_t wanted_momentumx;
    int32_t wanted_momentumy;
    int32_t wanted_momentumz;
	df::coord ref;
    std::set<df::coord> rel_positions;
    int lowest_relz;
	std::set<df::construction*> cmembers;
	std::set<df::building*> bmembers;
    std::set<df::unit*> umembers;
    df::burrow* um_burrow;

	MovingMachine(df::construction* c)
	 : offsetx(50000),
	   offsety(50000),
	   offsetz(0),
	   momentumx(0),
	   momentumy(0),
	   momentumz(0),
	   wanted_momentumx(0),
	   wanted_momentumy(0),
	   wanted_momentumz(0),
	   ref(c->pos),
	   rel_positions({ df::coord { 0, 0, 0 } }),
	   cmembers({ c }),
	   bmembers(),
	   umembers(),
	   um_burrow(new df::burrow) {
        std::vector<df::burrow*>& burrow_vec = df::burrow::get_vector();
        um_burrow->id = burrow_vec.size();
        um_burrow->name = "";
        um_burrow->tile = 0;
        um_burrow->fg_color = 0;
        um_burrow->bg_color = 0;
        // um_burrow->limit_workshops = /* TBD */;
        insert_into_vector(burrow_vec, &df::burrow::id, um_burrow);
        Burrows::setAssignedBlockTile(
            um_burrow,
            get_map_block(c->pos),
            df::coord2d { c->pos.x % 16, c->pos.y % 16 },
            true
        );
    }
    
    bool unit_passable() {
        return momentumx == 0 && momentumy == 0 && momentumz == 0;
    }
    
    void register_collision(df::construction* c, df::coord new_loc) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        log << "CONSTRUCTION COLLISION: " << c->pos.x << " " << c->pos.y << " " << c->pos.z << std::endl;
        log << "COLLIDED TERRAIN LOC: " << new_loc.x << " " << new_loc.y << " " << new_loc.z << std::endl;
        if (new_loc.z != c->pos.z) momentumz = wanted_momentumz;
        else {
            if (new_loc.x != c->pos.x) momentumx = wanted_momentumx;
            if (new_loc.y != c->pos.y) momentumy = wanted_momentumy;
        }
    }
    
    void register_collision(df::building* b, df::coord new_loc, df::coord old_loc) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        log << "BUILDING COLLISION: " << b->id << std::endl;
        log << "COLLIDED TERRAIN LOC: " << new_loc.x << " " << new_loc.y << " " << new_loc.z << std::endl;
        if (new_loc.z != old_loc.z) momentumz = wanted_momentumz;
        else {
            if (new_loc.x != old_loc.x) momentumx = wanted_momentumx;
            if (new_loc.y != old_loc.y) momentumy = wanted_momentumy;
        }
    }
    
    int get_mass() {
        return cmembers.size();
    }
    
    bool passable(df::coord loc) {
        df::map_block* block = get_map_block(loc);
        return block->walkable[loc.x%16][loc.y%16]
          || OPEN_TILES.count(block->tiletype[loc.x%16][loc.y%16]) > 0;
    }
    
    df::coord get_rel_pos(df::coord pos) {
        return df::coord {
            pos.x - ref.x,
            pos.y - ref.y,
            pos.z - ref.z
        };
    }
    
    bool contains(df::coord pos) {
        return rel_positions.count(get_rel_pos(pos)) > 0;
    }
    
    void add_spec_ref(df::building* b) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        log << "adding df::specific_ref" << std::endl;
        df::specific_ref* r = new df::specific_ref;
        log << "CHECKPOINT 1" << std::endl;
        r->type = MV_MACHINE_SPEC_REF;
        log << "CHECKPOINT 2" << std::endl;
        r->object = this;
        log << "CHECKPOINT 3" << std::endl;
        r->arg2.wrestle = nullptr;
        log << "CHECKPOINT 4" << std::endl;
        b->specific_refs.push_back(r);
        log << "CHECKPOINT 5" << std::endl;
    }

	void update(unsigned int num_steps) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        log << "MovingMachine::update" << std::endl;
        log << num_steps << std::endl;
        log << momentumy << std::endl;
        log << num_steps * momentumy << std::endl;
        log << "begin registering new mmembers" << std::endl;
        std::set<df::machine*> mmembers;
        for (df::building* b : bmembers) {
            df::machine* m = get_machine(b);
            if (m != nullptr) {
                mmembers.insert(m);
            }
        }
        log << "done (registering new mmembers)" << std::endl;
        log << "begin registering MV_MACHINE spec refs" << std::endl;
        for (df::machine* m : mmembers) {
            for (df::machine::T_components* comp : m->components) {
                df::building* b = df::building::find(comp->building_id);
                MovingMachine* ref_object
                    = (MovingMachine*)get_spec_ref(b, MV_MACHINE_SPEC_REF);
                if (ref_object != this) {
                    add_spec_ref(b);
                }
            }
        }
        log << "done (registering MV_MACHINE spec refs)" << std::endl;
        if (momentumx == 0 && momentumy == 0 && momentumz == 0) {
            log << "stationary; disabling burrow" << std::endl;
            Burrows::clearUnits(um_burrow);
            return;
        }
        log << "begin removing obsolete units" << std::endl;
        for (std::set<df::unit*>::iterator i = umembers.begin();
               i != umembers.end(); ++i) {
            if (!contains((*i)->pos)) {
                Burrows::setAssignedUnit(um_burrow, *i, false);
                umembers.erase(i);
            }
        }
        log << "done (removing obsolete units)" << std::endl;
        log << "begin adding new units" << std::endl;
        for (df::unit* u : world->units.active) {
            log << "u->id: " << u->id << std::endl;
            if (!contains(u->pos)) continue;
            log << "not on moving machine" << std::endl;
            if (umembers.count(u) == 0) {
                log << "adding to burrow" << std::endl;
                Burrows::setAssignedUnit(um_burrow, u, true);
                umembers.insert(u);
            }
        }
        log << "done (adding new units)" << std::endl;
        int mass = get_mass();
        int32_t velx = momentumx / mass;
        int32_t vely = momentumy / mass;
        int32_t velz = momentumz / mass;
		offsetx += num_steps * velx;
		offsety += num_steps * vely;
		offsetz += num_steps * velz;
        log << offsetx;
        log << "; y=";
        log << offsety;
        log << "; z=";
        log << offsetz;
        log << std::endl;
		int translatex = offsetx / TILE_DIMEN;
		int translatey = offsety / TILE_DIMEN;
		int translatez = offsetz / TILE_DIMEN;
		offsetx %= TILE_DIMEN;
		offsety %= TILE_DIMEN;
		offsetz %= TILE_DIMEN;
        log << "translate: x=";
        log << translatex;
        log << "; y=";
        log << translatey;
        log << "; z=";
        log << translatez;
        log << std::endl;
        log << "offset: x=";
        log << offsetx;
        log << "; y=";
        log << offsety;
        log << "; z=";
        log << offsetz;
        log << std::endl;
        if (translatex == 0 && translatey == 0 && translatez == 0) {
            return;
        }
        std::map<df::construction*, df::coord> to_move;
        bool is_collision = false;
		for (df::construction* c : cmembers) {
            df::coord new_loc {
				c->pos.x + translatex,
                c->pos.y + translatey,
                c->pos.z + translatez
            };
            to_move.insert(std::make_pair(c, new_loc));
            if (!passable(new_loc) && !contains(new_loc)) {
                register_collision(c, new_loc);
                is_collision = true;
            }
        }
        for (df::machine* m : mmembers) {
            for (df::machine::T_components* comp : m->components) {
                df::building* b = df::building::find(comp->building_id);
                for (int16_t x = b->x1; x <= b->x2; ++x) {
                    for (int16_t y = b->y1; y <= b->y2; ++y) {
                        df::coord old_loc { x, y, b->z };
                        df::coord new_loc {
                            x + translatex,
                            y + translatey,
                            b->z + translatez
                        };
                        if (!passable(new_loc) && !contains(new_loc)) {
                            register_collision(b, new_loc, old_loc);
                            is_collision = true;
                        }
                    }
                }
            }
        }
        if (is_collision) return;
        for (df::construction* c : cmembers) {
            Burrows::setAssignedBlockTile(
                um_burrow,
                get_map_block(c->pos),
                df::coord2d { c->pos.x % 16, c->pos.y % 16 },
                false
            );
        }
        for (auto i = to_move.begin(); i != to_move.end(); ++i) {
            df::construction* c = i->first;
            df::coord new_pos = i->second;
            int16_t cx = c->pos.x;
            int16_t cy = c->pos.y;
            int16_t newx = new_pos.x;
            int16_t newy = new_pos.y;
            // block from which c is moving
            df::map_block* old_block = get_map_block(c->pos);
            // block into which c is moving
            df::map_block* new_block = get_map_block(new_pos);
            // tiletype underneath c
            df::tiletype orig_tile = c->original_tile;
            // tiletype that WILL be underneath c
            df::tiletype old_tile = new_block->tiletype[newx%16][newy%16];
            // tiletype of c
            df::tiletype new_tile = old_block->tiletype[cx%16][cy%16];
            // if c is moving into a construction inside this machine...
            if (contains(new_pos) > 0) {
                // update the original_tile of that construction to be the same
                // as c's tiletype (so that the tiletype isn't overwritten when
                // that construction moves)
                df::construction* next_c = df::construction::find(new_pos);
                log << next_c << std::endl;
                old_tile = next_c->original_tile;
                next_c->original_tile = new_tile;
            }
            // remember the tile type that will go underneath c
            c->original_tile = old_tile;
            // update the tiletype of new_pos
            new_block->tiletype[newx%16][newy%16] = new_tile;
            // revert the tiletype of c's position to the original
            old_block->tiletype[cx%16][cy%16] = orig_tile;
            // actually move c
            c->pos = new_pos;
            Burrows::setAssignedBlockTile(
                um_burrow,
                get_map_block(c->pos),
                df::coord2d { c->pos.x % 16, c->pos.y % 16 },
                true
            );
        }
        df::coord translate {
            translatex,
            translatey,
            translatez
        };
        for (df::unit* u : umembers) {
            move_unit(u, translate);
        }
        for (df::machine* m : mmembers) {
            move_machine(m, translate);
        }
        for (df::building* b : bmembers) {
            df::machine* m = get_machine(b);
            if (m == nullptr) {
                move_building(b, translate);
            }
        }
        ref.x += translatex;
        ref.y += translatey;
        ref.z += translatez;
        bool drop = true;
        for (df::construction* c : cmembers) {
            if (c->pos.z - ref.z > lowest_relz) continue;
            if (OPEN_TILES.count(c->original_tile) == 0) {
                drop = false;
                break;
            }
        }
        if (drop) {
            momentumz -= GRAVITY * get_mass();
        }
    }
};

const std::set<df::tiletype> MovingMachine::OPEN_TILES {
    df::tiletype::OpenSpace,
    df::tiletype::RampTop
};

// Event binding/handling

typedef std::map<df::coord, MovingMachine*> loc_map_t;

std::vector<MovingMachine*> machines;
loc_map_t oldlocs;
loc_map_t newlocs;

void gen_newlocs(df::construction* c, MovingMachine* m) {
    for (int16_t dx = -1; dx <= 1; ++dx) {
        for (int16_t dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            newlocs.insert(std::make_pair(df::coord {
                c->pos.x + dx,
                c->pos.y + dy,
                c->pos.z
            }, m));
        }
    }
}

void handle_new_platform(df::construction* c) {
    std::ofstream log("moving-machines-log.txt", std::ios_base::app);
    log << "handle_new_platform" << std::endl;
    if (c->flags.whole & C_FLAG_PLATFORM) return;
    loc_map_t::iterator i = newlocs.find(c->pos);
	MovingMachine* m;
	if (i == newlocs.end()) {
        log << "NEW machine" << std::endl;
		m = new MovingMachine(c);
		machines.push_back(m);
	} else {
        log << "OLD machine" << std::endl;
		m = i->second;
        df::coord rel_pos = m->get_rel_pos(c->pos);
        m->rel_positions.insert(rel_pos);
        if (rel_pos.z < m->lowest_relz) {
            m->lowest_relz = rel_pos.z;
        }
		m->cmembers.insert(c);
	}
	oldlocs.insert(std::make_pair(c->pos, m));
	gen_newlocs(c, m);
    c->flags.whole |= C_FLAG_PLATFORM;
}

void handle_new_building(df::building* b) {
    if (b->flags.whole & B_FLAG_PLATFORM) return;
    MovingMachine* m;
	loc_map_t::iterator i = oldlocs.find(df::coord { b->x1, b->y1, b->z });
	if (i != oldlocs.end()) {
        m = i->second;
        m->bmembers.insert(b);
        m->add_spec_ref(b);
        b->flags.whole |= B_FLAG_PLATFORM;
    }
}

void handle_del_construction(df::construction* c) {
	loc_map_t::iterator i = oldlocs.find(c->pos);
	if (i != oldlocs.end()) {
		std::set<df::construction*>& cmembers = i->second->cmembers;
		cmembers.erase(cmembers.find(c));
		oldlocs.erase(i);
	}
}

void handle_del_building(df::building* b) {
	loc_map_t::iterator i = oldlocs.find(df::coord { b->x1, b->y1, b->z });
	if (i != oldlocs.end()) {
		std::set<df::building*>& bmembers = i->second->bmembers;
		bmembers.erase(bmembers.find(b));
	}
}

struct wheel_info {
    bool active;
    int32_t consumed;
};

struct wheel_hook : df::building_workshopst {
    static const unsigned int MOMENTUM_POWER_RATIO = 400;
    typedef df::building_workshopst interpose_base;

    wheel_info* create_wheel_info() {
        df::specific_ref* r = new df::specific_ref;
        wheel_info* w_info = new wheel_info;
        w_info->active = true;
        w_info->consumed = 0;
        r->type = INFO_SPEC_REF;
        r->object = w_info;
        r->arg2.wrestle = nullptr;
        specific_refs.push_back(r);
        return w_info;
    }
    
    wheel_info* get_wheel_info() {
        wheel_info* ret = (wheel_info*)get_spec_ref(this, INFO_SPEC_REF);
        if (ret == nullptr) return create_wheel_info();
        return ret;
    }
    
    MovingMachine* get_mv_machine() {
        return (MovingMachine*)get_spec_ref(this, MV_MACHINE_SPEC_REF);
    }
    
    bool can_run() {
        df::map_block* block = get_map_block(df::coord { x1, y1, z });
        return block->tiletype[x1%16][y1%16] != df::tiletype::OpenSpace;
    }
    
    bool is_wheel() {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        log << "CHECKING IF *this IS WHEEL" << std::endl;
        df::building_def* def = df::building_def::find(custom_type);
        if (def == nullptr) return false;
        std::string code = def->code;
        return code.substr(0, 5) == "WHEEL";
    }
    
    DEFINE_VMETHOD_INTERPOSE(void, updateAction, ()) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        log << "wheel_hook::updateAction HOOK" << std::endl;
        INTERPOSE_NEXT(updateAction)();
        log << "INTERPOSE_NEXT(updateAction) completed" << std::endl;
        df::building_def* def = df::building_def::find(custom_type);
        log << "df::building_def::find completed: ";
        log << def << std::endl;
        if (def == nullptr) return;
        log << "POINT A" << std::endl;
        std::string code = def->code;
        log << "POINT B" << std::endl;
        if (!is_wheel()) return;
        log << "is wheel" << std::endl;
        CoreSuspendClaimer claimer;
        wheel_info* w_info = get_wheel_info();
        MovingMachine* mv_machine = get_mv_machine();
        log << "w_info: " << w_info << std::endl;
        log << "mv_machine: " << mv_machine << std::endl;
        if (mv_machine == nullptr) {
            log << "mv_machine NONEXISTENT" << std::endl;
            return;
        }
        if (!w_info->active) return;
        if (!can_run()) return;
        df::machine_info m_info = machine;
        df::machine* m = df::machine::find(m_info.machine_id);
        log << "m: " << m << std::endl;
        log << "m->cur_power: " << m->cur_power << std::endl;
        log << "m->min_power: " << m->min_power << std::endl;
        int dx;
        int dy;
        switch (code[5]) {
            case 'N':
                dx = 0;
                dy = -1;
                break;
            case 'E':
                dx = 1;
                dy = 0;
                break;
            case 'S':
                dx = 0;
                dy = 1;
                break;
            case 'W':
                dx = -1;
                dy = 0;
                break;
        }
        log << "dx: " << dx << std::endl;
        log << "dy: " << dy << std::endl;
        int dp = std::max(m->cur_power - m->min_power,
                          -w_info->consumed);
        log << "dp: " << dp << std::endl;
        int dv = MOMENTUM_POWER_RATIO * dp;
        log << "dv: " << dv << std::endl;
        mv_machine->momentumx += dx * dv;
        mv_machine->momentumy += dy * dv;
        mv_machine->wanted_momentumx += dx * dv;
        mv_machine->wanted_momentumy += dy * dv;
        log << "done accelerating mv_machine" << std::endl;
        w_info->consumed += dp;
        log << "done updating w_info->consumed" << std::endl;
        m->min_power += dp;
    }
    
    DEFINE_VMETHOD_INTERPOSE(df::machine_info*, getMachineInfo, ()) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        log << "wheel_hook::getMachineInfo HOOK" << std::endl;
        if (is_wheel()) {
            return &machine;
        } else {
            return INTERPOSE_NEXT(getMachineInfo)();
        }
    }
    
    DEFINE_VMETHOD_INTERPOSE(void, getPowerInfo, (df::power_info* p_info)) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        log << "wheel_hook::getMachineInfo HOOK" << std::endl;
        if (!is_wheel()) {
            INTERPOSE_NEXT(getPowerInfo)(p_info);
            return;
        }
        wheel_info* w_info = get_wheel_info();
        p_info->produced = 0;
        p_info->consumed = w_info->consumed;
    }
    
    DEFINE_VMETHOD_INTERPOSE(bool, isUnpowered, ()) {
        if (!is_wheel()) return INTERPOSE_NEXT(isUnpowered)();
        wheel_info* w_info = get_wheel_info();
        return w_info->consumed == 0;
    }
    
    DEFINE_VMETHOD_INTERPOSE(void, categorize, (bool free)) {
        CoreSuspendClaimer claimer;
        if (is_wheel()) {
            std::vector<df::building*>& vec = world->buildings.other[0x6];
            insert_into_vector(vec, &df::building::id, (df::building*)this);
        }
        INTERPOSE_NEXT(categorize)(free);
    }

    DEFINE_VMETHOD_INTERPOSE(void, uncategorize, ()) {
        CoreSuspendClaimer claimer;
        if (is_wheel()) {
            std::vector<df::building*>& vec = world->buildings.other[0x6];
            erase_from_vector(vec, &df::building::id, id);
        }
        INTERPOSE_NEXT(uncategorize)();
    }
};

IMPLEMENT_VMETHOD_INTERPOSE(wheel_hook, updateAction);
IMPLEMENT_VMETHOD_INTERPOSE(wheel_hook, getMachineInfo);
IMPLEMENT_VMETHOD_INTERPOSE(wheel_hook, getPowerInfo);
IMPLEMENT_VMETHOD_INTERPOSE(wheel_hook, isUnpowered);
IMPLEMENT_VMETHOD_INTERPOSE(wheel_hook, categorize);
IMPLEMENT_VMETHOD_INTERPOSE(wheel_hook, uncategorize);

void desig_platform(df::coord corner1, df::coord corner2) {
    int16_t min_x = std::min(corner1.x, corner2.x);
    int16_t max_x = std::max(corner1.x, corner2.x);
    int16_t min_y = std::min(corner1.y, corner2.y);
    int16_t max_y = std::max(corner1.y, corner2.y);
    int16_t min_z = std::min(corner1.z, corner2.z);
    int16_t max_z = std::max(corner1.z, corner2.z);
    for (int16_t x = min_x; x <= max_x; ++x) {
        for (int16_t y = min_y; y <= max_y; ++y) {
            for (int16_t z = min_z; z <= max_z; ++z) {
                df::coord pos { x, y, z };
                df::construction* c = df::construction::find(pos);
                if (c != nullptr) {
                    handle_new_platform(c);
                    df::building* b = Buildings::findAtTile(pos);
                    if (b != nullptr) {
                        b->flags.whole &= ~B_FLAG_CLEARED;
                    }
                }
            }
        }
    }
}

struct dwarfmode_hook : df::viewscreen_dwarfmodest {
    typedef df::viewscreen_dwarfmodest interpose_base;
    
    static bool in_plat_menu;
    static const std::set<smode_enum_t> DESIG_MODES;
    static const long MIN_DESIGNATE_KEY = 0x48F;
    static const long MAX_DESIGNATE_KEY = 0x4B4;
    
    bool in_desig_menu() {
        return DESIG_MODES.count(ui->main.mode) > 0;
    }
    
    DEFINE_VMETHOD_INTERPOSE(void, feed, (std::set<df::interface_key>* keys)) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        CoreSuspendClaimer claimer;
        if (!in_desig_menu()) {
            log << "!in_desig_menu()" << std::endl;
            INTERPOSE_NEXT(feed)(keys);
            return;
        }
        std::set<df::interface_key> propagate;
        for (df::interface_key k : *keys) {
            log << "k: " << k << std::endl;
            if (k == df::interface_key::CUSTOM_SHIFT_P) { // key for entering platform menu
                log << "ENTERING PLATFORM MENU" << std::endl;
                ui->main.mode = smode_enum_t::DesignateMine;
                log << "CHECKPOINT 1" << std::endl;
                in_plat_menu = true;
                log << "CHECKPOINT 2" << std::endl;
            }
            if (!in_plat_menu) {
                log << "!in_plat_menu" << std::endl;
                propagate.insert(k);
                continue;
            }
            if (k == df::interface_key::SELECT) {
                df::coord start { 
                    selection_rect->start_x,
                    selection_rect->start_y,
                    selection_rect->start_z
                };
                if (start.x < 0) {
                    propagate.insert(k);
                    continue;
                }
                df::coord end { 
                    cursor->x,
                    cursor->y,
                    cursor->z
                };
                desig_platform(start, end);
                selection_rect->start_x = -30000;
                selection_rect->start_y = -30000;
                selection_rect->start_z = -30000;
            } else {
                propagate.insert(k);
                if (MIN_DESIGNATE_KEY <= (long)k
                      && (long)k <= MAX_DESIGNATE_KEY) in_plat_menu = false;
            }
        }
        log << "OUT OF LOOP" << std::endl;
        INTERPOSE_NEXT(feed)(&propagate);
        log << "DONE PROPAGATING" << std::endl;
        return;
    }
    
    DEFINE_VMETHOD_INTERPOSE(void, render, ()) {
        std::ofstream log("moving-machines-log.txt", std::ios_base::app);
        CoreSuspendClaimer claimer;
        INTERPOSE_NEXT(render)();
        if (!in_desig_menu()) {
            in_plat_menu = false;
            return;
        }
        Gui::DwarfmodeDims dims = Gui::getDwarfmodeViewDims();
        Screen::Painter p = Screen::Painter(dims.menu());
        Screen::Pen green_pen(' ', 2, 0, true);
        Screen::Pen text_pen(' ', 7, 0, false);
        Screen::Pen bold_pen(' ', 7, 0, true);
        p.seek(2, 1);
        if (in_plat_menu) {
            p.pen(text_pen);
            p.string(": Mine");
        } else if (ui->main.mode == smode_enum_t::DesignateMine) {
            p.pen(bold_pen);
            p.string(": Mine");
        }
        p.seek(1, 17);
        p.pen(green_pen);
        p.tile('P');
        p.pen(in_plat_menu ? bold_pen : text_pen);
        p.string(": Create Platform");
    }
};

bool dwarfmode_hook::in_plat_menu = false;

const std::set<smode_enum_t> dwarfmode_hook::DESIG_MODES {
    smode_enum_t::DesignateMine,
    smode_enum_t::DesignateRemoveRamps,
    smode_enum_t::DesignateUpStair,
    smode_enum_t::DesignateDownStair,
    smode_enum_t::DesignateUpDownStair,
    smode_enum_t::DesignateUpRamp,
    smode_enum_t::DesignateChannel,
    smode_enum_t::DesignateGatherPlants,
    smode_enum_t::DesignateRemoveDesignation,
    smode_enum_t::DesignateSmooth,
    smode_enum_t::DesignateCarveTrack,
    smode_enum_t::DesignateEngrave,
    smode_enum_t::DesignateCarveFortification,
    smode_enum_t::DesignateItemsClaim,
    smode_enum_t::DesignateItemsForbid,
    smode_enum_t::DesignateItemsMelt,
    smode_enum_t::DesignateItemsUnmelt,
    smode_enum_t::DesignateItemsDump,
    smode_enum_t::DesignateItemsUndump,
    smode_enum_t::DesignateItemsHide, 
    smode_enum_t::DesignateItemsUnhide, 
    smode_enum_t::DesignateChopTrees, 
    smode_enum_t::DesignateToggleEngravings, 
    smode_enum_t::DesignateToggleMarker,
    smode_enum_t::DesignateTrafficHigh, 
    smode_enum_t::DesignateTrafficNormal,
    smode_enum_t::DesignateTrafficLow, 
    smode_enum_t::DesignateTrafficRestricted,
    smode_enum_t::DesignateRemoveConstruction,
};

IMPLEMENT_VMETHOD_INTERPOSE(dwarfmode_hook, feed);
IMPLEMENT_VMETHOD_INTERPOSE(dwarfmode_hook, render);

// Main loop logic

struct dfhack_exception : std::exception {
    virtual const char* what() const noexcept {
        return "DFHACK ERROR";
    }
};

bool active = false;
const unsigned int DEFAULT_SPEED = 5;
unsigned int speed = DEFAULT_SPEED;

typedef command_result (*op_func_t)(color_ostream&, const std::vector<std::string>&);

unsigned int read_uint(color_ostream& out, const std::vector<std::string>& params, unsigned int p_index) {
    int ret;
    bool error = false;
    try {
        ret = str_to_int(params[p_index]);
    } catch (std::ios_base::failure& e) {
        error = true;
    }
    if (ret < 0) error = true;
    if (error) {
        out.printerr("Parameter #");
        out.printerr(int_to_str(p_index).c_str());
        out.printerr(" is not a nonnegative integer.\n");
        throw dfhack_exception();
    }
    return ret;
}

command_result start_system(color_ostream& out, const std::vector<std::string>& params) {
	if (params.size() != 1) {
		out.printerr("Invalid number of parameters!\n");
		return CR_FAILURE;
	}
	if (active) return CR_OK;
    INTERPOSE_HOOK(wheel_hook, updateAction).apply(true);
    INTERPOSE_HOOK(wheel_hook, getMachineInfo).apply(true);
    INTERPOSE_HOOK(wheel_hook, getPowerInfo).apply(true);
    INTERPOSE_HOOK(wheel_hook, isUnpowered).apply(true);
    INTERPOSE_HOOK(wheel_hook, categorize).apply(true);
    INTERPOSE_HOOK(wheel_hook, uncategorize).apply(true);
    INTERPOSE_HOOK(dwarfmode_hook, feed).apply(true);
    INTERPOSE_HOOK(dwarfmode_hook, render).apply(true);
	active = true;
	return CR_OK;
}

command_result stop_system(color_ostream& out, const std::vector<std::string>& params) {
	if (params.size() != 1) {
		out.printerr("Invalid number of parameters!\n");
		return CR_FAILURE;
	}
	if (!active) return CR_OK;
    INTERPOSE_HOOK(wheel_hook, updateAction).remove();
    INTERPOSE_HOOK(wheel_hook, getMachineInfo).remove();
    INTERPOSE_HOOK(wheel_hook, getPowerInfo).remove();
    INTERPOSE_HOOK(wheel_hook, isUnpowered).remove();
    INTERPOSE_HOOK(dwarfmode_hook, feed).remove();
    INTERPOSE_HOOK(dwarfmode_hook, render).remove();
	active = false;
	return CR_OK;
}

command_result read_status(color_ostream& out, const std::vector<std::string>& params) {
    MovingMachine* m;
    switch (params.size()) {
        case 1:
            out.print(active ? "ACTIVE\n" : "INACTIVE\n");
            out.print("update speed: ");
            out.print(int_to_str(speed).c_str());
            out.print("\n");
            out.print("# moving machines: ");
            out.print(int_to_str(machines.size()).c_str());
            out.print("\n");
            out.print("# old locs: ");
            out.print(int_to_str(oldlocs.size()).c_str());
            out.print("\n");
            out.print("# new locs: ");
            out.print(int_to_str(newlocs.size()).c_str());
            out.print("\n");
            return CR_OK;
        case 2:
            unsigned int id;
            try {
                id = read_uint(out, params, 1);
            } catch (dfhack_exception& e) {
                return CR_FAILURE;
            }
            if (id >= machines.size()) {
                out.printerr("Invalid moving machine id!\n");
                return CR_FAILURE;
            }
            m = machines[id];
            out.print("moving machine #");
            out.print(int_to_str(id).c_str());
            out.print("\n====================\n");
            out.print("ref pos: ");
            out.print(int_to_str(m->ref.x).c_str());
            out.print(" ");
            out.print(int_to_str(m->ref.y).c_str());
            out.print(" ");
            out.print(int_to_str(m->ref.z).c_str());
            out.print("\n");
            out.print("momentum: ");
            out.print(int_to_str(m->momentumx).c_str());
            out.print(" ");
            out.print(int_to_str(m->momentumy).c_str());
            out.print(" ");
            out.print(int_to_str(m->momentumz).c_str());
            out.print("\n");
            out.print("# cmembers: ");
            out.print(int_to_str(m->cmembers.size()).c_str());
            out.print("\n");
            out.print("# bmembers: ");
            out.print(int_to_str(m->bmembers.size()).c_str());
            out.print("\n");
            out.print("# umembers: ");
            out.print(int_to_str(m->umembers.size()).c_str());
            out.print("\n");
            return CR_OK;
        default:
            out.printerr("Invalid number of parameters!\n");
            return CR_FAILURE;
    }
}

command_result change_speed(color_ostream& out, const std::vector<std::string>& params) {
	if (params.size() != 2) {
		out.printerr("Invalid number of parameters!\n");
		return CR_FAILURE;
	}
	try {
        speed = read_uint(out, params, 1);
    } catch (dfhack_exception& e) {
        return CR_FAILURE;
    }
	return CR_OK;
}

command_result register_platform(color_ostream& out, const std::vector<std::string>& params) {
    if (params.size() != 4) {
        out.printerr("Invalid number of parameters!\n");
        return CR_FAILURE;
    }
    df::coord pos;
    try {
        pos.x = read_uint(out, params, 1);
        pos.y = read_uint(out, params, 2);
        pos.z = read_uint(out, params, 3);
    } catch (dfhack_exception& e) {
        return CR_FAILURE;
    }
    out.print(params[1].c_str());
    out.print(" ");
    out.print(params[2].c_str());
    out.print(" ");
    out.print(params[3].c_str());
    out.print("\n");
    df::construction* c = df::construction::find(pos);
    
    std::stringstream ss;
    std::string pointer_str;
    ss << c;
    ss >> pointer_str;
    out.print(pointer_str.c_str());
    out.print("\n");
    
    c->flags.whole |= C_FLAG_PLATFORM;
    handle_new_platform(c);
    return CR_OK;
}

command_result link_building(color_ostream& out, const std::vector<std::string>& params) {
    if (params.size() != 4) {
        out.printerr("Invalid number of parameters!\n");
        return CR_FAILURE;
    }
    df::coord pos;
    try {
        pos.x = read_uint(out, params, 1);
        pos.y = read_uint(out, params, 2);
        pos.z = read_uint(out, params, 3);
    } catch (dfhack_exception& e) {
        return CR_FAILURE;
    }
    df::building* b = Buildings::findAtTile(pos);
    out.print(int_to_str(b->x1).c_str());
    out.print(" ");
    out.print(int_to_str(b->y1).c_str());
    out.print(" ");
    out.print(int_to_str(b->z).c_str());
    out.print("\n");
    handle_new_building(b);
    return CR_OK;
}    

command_result accel_machine(color_ostream& out, const std::vector<std::string>& params) {
    if (params.size() != 5) {
        out.printerr("Invalid number of parameters!\n");
        return CR_FAILURE;
    }
    unsigned int id;
    df::coord dv;
    try {
        id = read_uint(out, params, 1);
        dv.x = read_uint(out, params, 2);
        dv.y = read_uint(out, params, 3);
        dv.z = read_uint(out, params, 4);
    } catch (dfhack_exception& e) {
        return CR_FAILURE;
    }
    machines[id]->momentumx += dv.x;
    machines[id]->momentumy += dv.y;
    machines[id]->momentumz += dv.z;
    return CR_OK;
}

command_result update_machines(color_ostream& out, const std::vector<std::string>& params) {
    if (params.size() != 1) {
        out.printerr("Invalid number of parameters!\n");
        return CR_FAILURE;
    }
    std::for_each(
		machines.begin(),
		machines.end(),
		[&out](MovingMachine* ptr) { ptr->update(speed); }
    );
    return CR_OK;
}

typedef std::map<std::string, op_func_t> op_map_t;
op_map_t op_map {
	std::make_pair("start", &start_system),
	std::make_pair("stop", &stop_system),
	std::make_pair("status", &read_status),
	std::make_pair("speed", &change_speed),
	std::make_pair("platform", &register_platform),
	std::make_pair("link", &link_building),
	std::make_pair("accel", &accel_machine),
	std::make_pair("update", &update_machines),
};

command_result proc_command(color_ostream& out, std::vector<std::string>& params) {
	if (params.size() == 0) {
		out.printerr("Operation not specified!\n");
		return CR_FAILURE;
	}
	std::string operation = params[0];
	op_map_t::iterator i = op_map.find(operation);
	if (i == op_map.end()) {
		out.printerr("Operation invalid!\n");
		return CR_FAILURE;
	}
	CoreSuspender suspender;
	return (i->second)(out, params);
}

DFhackCExport command_result plugin_init(
	color_ostream& out, 
	std::vector<PluginCommand>& commands
) {
	commands.push_back(PluginCommand(
		"moving-machines",
	       	"Control moving machine settings.",
	       	proc_command,
	       	false,
		"=====TODO====\n"
	));
	return start_system(out, { "start" });
}

DFhackCExport command_result plugin_shutdown(color_ostream& out) {
	command_result result = stop_system(out, { "stop" });
	std::for_each(
		machines.begin(),
		machines.end(),
		[](MovingMachine* ptr) { delete ptr; }
	);
	return result;
}

DFhackCExport command_result plugin_onupdate(color_ostream& out) {
    CoreSuspender suspender;
	if (!active) return CR_OK;
    if (*pause_state) return CR_OK;
	if (world->frame_counter % speed == 0) {
		update_machines(out, { "update" });
        for (df::building* b : df::building::get_vector()) {
            if (b->flags.whole & B_FLAG_CLEARED) continue;
            if (b->getBuildStage() < b->getMaxBuildStage()) continue;
            handle_new_building(b);
            b->flags.whole &= B_FLAG_CLEARED;
        }
	}
	return CR_OK;
}
