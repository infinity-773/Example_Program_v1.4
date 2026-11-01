#include "ReedSwitch.h"

#include <stddef.h>

#define REED_SWITCH_DEBOUNCE_MS 20U

void ReedSwitch_Init(ReedSwitch *sensor, ReedSwitchReadState read_state)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->read_state = read_state;
    sensor->previous_state = 0U;
    sensor->armed = 0U;
}

void ReedSwitch_Arm(ReedSwitch *sensor, uint32_t now)
{
    if (sensor == NULL || sensor->read_state == NULL)
    {
        return;
    }

    sensor->previous_state = sensor->read_state() ? 1U : 0U;
    Debouncer_Init(&sensor->debouncer, sensor->previous_state,
                   REED_SWITCH_DEBOUNCE_MS, DEBOUNCER_EDGE_RISING, now);
    sensor->armed = 1U;
}

void ReedSwitch_Disarm(ReedSwitch *sensor)
{
    if (sensor != NULL)
    {
        sensor->armed = 0U;
    }
}

uint8_t ReedSwitch_RisingEdge(ReedSwitch *sensor, uint32_t now)
{
    if (sensor == NULL || !sensor->armed || sensor->read_state == NULL)
    {
        return 0U;
    }

    uint8_t state = Debouncer_calc(&sensor->debouncer,
                                   sensor->read_state(), now);
    uint8_t rising_edge = (!sensor->previous_state && state) ? 1U : 0U;
    sensor->previous_state = state;
    return rising_edge;
}