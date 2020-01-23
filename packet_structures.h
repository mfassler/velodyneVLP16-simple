#ifndef __PACKET_STRUCTURE_H
#define __PACKET_STRUCTURE_H


#include <stdint.h>


// Remember typical boundaries:
#pragma pack(push)

// set 1-byte boundaries
#pragma pack(1)

struct data_point {
	uint16_t distance;
	uint8_t intensity;  // this requires 1-byte boundaries
};

struct data_block {
	uint16_t flag;
	uint16_t azimuth;  // degrees * 100

	struct data_point dpoints_a[16];
	struct data_point dpoints_b[16];
};

// restore original boundaries
#pragma pack(pop)

#endif // __PACKET_STRUCTURE_H
