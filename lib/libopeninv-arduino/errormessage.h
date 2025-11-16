/*
 * Simplified error message handling for Arduino port
 */
#ifndef ERRORMESSAGE_H
#define ERRORMESSAGE_H

#include <stdint.h>

class ErrorMessage
{
public:
   static uint32_t GetErrorNum(uint8_t index) { return 0; }
   static uint32_t GetErrorTime(uint8_t index) { return 0; }
};

#endif // ERRORMESSAGE_H
