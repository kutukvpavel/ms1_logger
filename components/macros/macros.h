#pragma once

#include <stdint.h>

#define _BV(i) (1u << (i))
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#ifndef _LB
    #define _LB(val) ( (uint8_t)((val) & 0xFF) )
#endif
#ifndef _HB
    #define _HB(val) ( (uint8_t)(((val) >> (8 * (sizeof(val) - 1))) & 0xFF) )
#endif