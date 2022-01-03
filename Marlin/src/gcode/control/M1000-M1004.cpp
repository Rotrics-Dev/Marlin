#include "../gcode.h"
#include "../../module/dexarm/dexarm_air_pump.h"
#include "../../module/planner.h"
void GcodeSuite::M1000()
{
	planner.synchronize(); 
	dexarm_air_pump.suction();
}

void GcodeSuite::M1001()
{
	planner.synchronize(); 
	dexarm_air_pump.blow();
}

void GcodeSuite::M1002()
{
	planner.synchronize(); 
	dexarm_air_pump.aerofluxus();
}

void GcodeSuite::M1003()
{
	planner.synchronize(); 
	dexarm_air_pump.turn_off();
}

void GcodeSuite::M1004()
{
	planner.synchronize(); 
	dexarm_air_pump.show_status();
}
