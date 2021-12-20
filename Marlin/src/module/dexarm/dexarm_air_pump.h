#ifndef AIR_DEXARM_PUMP_H
#define AIR_DEXARM_PUMP_H

#include <stdint.h>

enum AirPumpStatus : uint8_t {
  AIR_PUMP_OFF,
  AIR_PUMP_IN,
  AIR_PUMP_OUT,
  AIR_PUMP_NEUTRAL,
};

class DexarmAirPump {
  public:
    void init();
    void suction();
    void blow();
    void aerofluxus();
    void turn_off();
    void show_status();
  private:
    AirPumpStatus cur_status = AIR_PUMP_OFF;
};
extern DexarmAirPump dexarm_air_pump;
#endif /* AIR_DEXARM_PUMP_H */
