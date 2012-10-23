#ifndef communcation_h
#define communication_h

#include <SoftwareSerial.h>

// Communication protocol
#define BYTE_IS_FRAME_HEADER(byte) (byte == 0xFF)
#define BYTE_IS_FRAME_FOOTER(byte) (byte >= 0xCC)
#define PARITY_BIT(byte) (byte & 0x03)
#define FRAME_DATA_LENGTH 8

// readFrame() return values
#define FRAME_INCOMPLETE 0
#define FRAME_COMPLETE 1
#define FRAME_DISCARDED 2

int countOnes(unsigned char);
int readFrame(SoftwareSerial *, unsigned char *);

#endif
