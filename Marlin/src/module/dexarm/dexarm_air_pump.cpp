#include "dexarm_air_pump.h"
#include "HAL.h"

DexarmAirPump dexarm_air_pump;

void DexarmAirPump::init() {
  MYSERIAL2.begin(115200);
  cur_status = AIR_PUMP_OFF;
}

void DexarmAirPump::suction() {
  MYSERIAL2.println("QB00");
  cur_status = AIR_PUMP_IN;
}

void DexarmAirPump::blow() {
  MYSERIAL2.println("QB01");
  cur_status = AIR_PUMP_OUT;
}

void DexarmAirPump::aerofluxus() {
  MYSERIAL2.println("QB02");
  cur_status = AIR_PUMP_NEUTRAL;
}

void DexarmAirPump::turn_off() {
  MYSERIAL2.println("QB03");
  cur_status = AIR_PUMP_OFF;
}

void DexarmAirPump::show_status() {
  SERIAL_ECHO("Pump status:");
  switch (cur_status) {
    case AIR_PUMP_OFF:
      SERIAL_ECHO("off");
      break;
    case AIR_PUMP_OUT:
      SERIAL_ECHO("out");
      break;
    case AIR_PUMP_IN:
      SERIAL_ECHO("in");
      break;
    case AIR_PUMP_NEUTRAL:
      SERIAL_ECHO("neutral");
      break;
    default:
      SERIAL_ECHO("off");
  }
  SERIAL_ECHOLN();
}
