#include "ParkingSensor.h"

#include <stddef.h>

void ParkingSensor_Init(ParkingSensor *sensor, ParkingSensorRead read_state,
                        ParkingSensorRead second_read_state)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->read_state = read_state;
    sensor->second_read_state = second_read_state;
}

uint8_t ParkingSensor_IsParking(const ParkingSensor *sensor)
{
    if (sensor == NULL)
    {
        return 0U;
    }

    uint8_t first_is_parking =
        (sensor->read_state != NULL && sensor->read_state()) ? 1U : 0U;
    uint8_t second_is_parking =
        (sensor->second_read_state != NULL && sensor->second_read_state()) ?
            1U : 0U;

    return (first_is_parking || second_is_parking) ? 1U : 0U;
}