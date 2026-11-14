#include "VehicleCommand.h"

#include <stddef.h>

void VehicleCommand_Arm(VehicleCommand *command, uint8_t initial_state,
                        uint32_t debounce_ms, uint32_t now)
{
    if (command == NULL)
    {
        return;
    }

    initial_state = (initial_state != 0U) ? 1U : 0U;
    Debouncer_Init(&command->debouncer, initial_state, debounce_ms,
                   DEBOUNCER_EDGE_BOTH, now);
    command->previous_state = initial_state;
}

uint8_t VehicleCommand_Triggered(VehicleCommand *command, uint8_t condition,
                                 uint32_t now)
{
    if (command == NULL)
    {
        return 0U;
    }

    uint8_t stable_state = Debouncer_calc(&command->debouncer,
                                          (condition != 0U) ? 1U : 0U, now);
    uint8_t rising_edge =
        (!command->previous_state && stable_state) ? 1U : 0U;

    command->previous_state = stable_state;
    return rising_edge;
}
