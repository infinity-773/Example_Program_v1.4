#ifndef _PARKINGSENSOR_H
#define _PARKINGSENSOR_H

#include <stdint.h>

typedef uint8_t (*ParkingSensorRead)(void);

typedef struct
{
    ParkingSensorRead read_state;
    ParkingSensorRead second_read_state;
} ParkingSensor;

void ParkingSensor_Init(ParkingSensor *sensor, ParkingSensorRead read_state,
                        ParkingSensorRead second_read_state);
uint8_t ParkingSensor_IsParking(const ParkingSensor *sensor);

#endif