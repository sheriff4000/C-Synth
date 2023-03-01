#include "../include/knob.hh" //for mac users
//#include "..\include\knob.hh" //for windows users (chumps)

// constructor
Knob::Knob()
{
    rotation = 0;
    max = 0;
    min = 0;
    previous = 0;
}

uint8_t Knob::get_rotation()
{
    return __atomic_load_n(&rotation, __ATOMIC_RELAXED);
}

void Knob::update_rotation(uint8_t keymatrix)
{
    // uint8_t current_rotation = get_rotation();

    uint8_t current_rotation = rotation;

    if ((previous == 0 && keymatrix == 1) || (previous == 3 && keymatrix == 2) || (previous == 3 && keymatrix == 0) || (previous == 0 && keymatrix == 1))
    {
        if (current_rotation < max)
        {
            current_rotation += 1;
        }
    }
    else if ((previous == 1 && keymatrix == 0) || (previous == 2 && keymatrix == 3))
    {
        if (current_rotation > min)
        {
            current_rotation -= 1;
        }
    }
    previous = keymatrix;

    __atomic_store_n(&rotation, current_rotation, __ATOMIC_RELAXED);
}

void Knob::set_limits(uint8_t bottom_limit, uint8_t top_limit)
{
    max = top_limit;
    min = bottom_limit;
}