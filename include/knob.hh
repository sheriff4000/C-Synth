#ifndef knob_hh
#define knob_hh

#include "knob.hh"
#include <Arduino.h>

class Knob
{
public:
    Knob();

    int8_t get_rotation();
    int8_t get_rotation_atomic();

    void update_rotation(uint8_t keymatrix);

    void set_limits(int8_t bottom_limit, int8_t top_limit);

private:
    int8_t rotation, max, min, previous;
};

#endif