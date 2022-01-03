#ifndef CONVEYOR_BELT_H
#define CONVEYOR_BELT_H

class ConveyorBelt {
  public:
    void turn_on(float feedRate_t, bool direction);
    void turn_off();
    void report_status();
  private:
    float feedrate = 0;
    bool dir;
};

extern ConveyorBelt conveyor_belt;

#endif
