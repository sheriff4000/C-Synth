#ifndef knob_hh
#define knob_hh

#include "knob.hh"
#include <Arduino.h>

class Knob
{
public:
    Knob();

    uint8_t get_rotation();

    void update_rotation(uint8_t keymatrix);

    void set_limits(uint8_t bottom_limit, uint8_t top_limit);


private:
    uint8_t rotation, max, min, previous;
};

#endif