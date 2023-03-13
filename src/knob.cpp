#include "../include/knob.hh"

// constructor
Knob::Knob()
{
    rotation = 0;
    max = 0;
    min = 0;
    previous = 0;
}

int8_t Knob::get_rotation()
{
    return rotation;
}
int8_t Knob::get_rotation_atomic()
{
    return __atomic_load_n(&rotation, __ATOMIC_RELAXED);
}

void Knob::update_rotation(uint8_t keymatrix)
{
    int8_t current_rotation = get_rotation_atomic();

    // uint8_t current_rotation = rotation;
    int8_t add = 0;

    if ((previous == 0 && keymatrix == 1) || (previous == 3 && keymatrix == 2) || (previous == 3 && keymatrix == 0) || (previous == 0 && keymatrix == 1))
    {
        if (current_rotation < max)
        {
            add = 1;
        }
    }
    else if ((previous == 1 && keymatrix == 0) || (previous == 2 && keymatrix == 3))
    {
        if (current_rotation > min)
        {
            add = -1;
        }
    }
    previous = keymatrix;

    if (add != 0) {
        __atomic_store_n(&rotation, current_rotation + add, __ATOMIC_RELAXED);
    }
}

void Knob::set_limits(int8_t bottom_limit, int8_t top_limit)
{
    max = top_limit;
    min = bottom_limit;
}