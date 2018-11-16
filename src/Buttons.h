/* Mouse buttons SR latch debounce
 * https://github.com/R3M0H/
 */

#pragma once
#include <stdint.h>

// Functions
void buttonsInit(void);
uint8_t buttonsCheck(void);
