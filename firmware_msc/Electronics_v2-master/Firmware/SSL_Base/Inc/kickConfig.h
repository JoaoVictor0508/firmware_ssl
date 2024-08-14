/*
 * kickConfig.h
 *
 *  Created on: 9 de out de 2022
 *      Author: leonardo
 */

#ifndef INC_KICKCONFIG_H_
#define INC_KICKCONFIG_H_

#define MAX_KICK_ACTIVATION 40.f // [ms]
#define MIN_KICK_ACTIVATION 1.f // [ms]

#define MAX_KICK_STRENGTH 100.f // [%]
#define MIN_KICK_STRENGTH 10.f  // [%]

#define CYCLES_BETWEEN_KICKS 400 // 400 ciclos -> 5ms por ciclo -> 2s de delay

typedef enum
{
    KICK_NONE = 0x00,
    DIRECT_SOFT = 0x01,
    DIRECT_CUSTOM = 0x02,
    DIRECT_STRONG = 0x03,
    CHIP_SOFT = 0x04,
    CHIP_CUSTOM = 0x08,
    CHIP_STRONG = 0x0C,
    ANGLE_KICK = 0xFF /// TODO: A ser definido
} KICK_TYPE_t;

#endif /* INC_KICKCONFIG_H_ */
