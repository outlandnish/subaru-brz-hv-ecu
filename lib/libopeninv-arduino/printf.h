/*
 * Simple printf interface for libopeninv
 * Used by CANOpen for debug output
 */
#ifndef PRINTF_H
#define PRINTF_H

// Interface for outputting characters
class IPutChar
{
public:
   virtual void PutChar(char c) = 0;
};

#endif // PRINTF_H
