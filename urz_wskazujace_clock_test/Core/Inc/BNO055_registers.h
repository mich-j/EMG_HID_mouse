/*
 * BNO055_registers.h
 *
 *  Created on: Nov 14, 2021
 *      Author: koteczeqqq
 */

#ifndef SRC_BNO055_REGISTERS_H_
#define SRC_BNO055_REGISTERS_H_

#define BNO_ADDRESS (0x29 << 1)
#define CHIP_ID 0x00
#define BNO_OPR_MODE 0x3D
#define CONFIGMODE 0x00
#define IMUMODE 0x08
#define BNO_UNIT_SEL 0x3B
#define BNO_SYS_TRIGGER 0x3f
#define EXT_CLK_SEL 0x80
#define CALIB_STAT 0x35
#define RST_SYS 0x20
#define BNO_AXIS_MAP_CONFIG 0x41
#define BNO_AXIS_MAP_SIGN 0x42

#define QUA_DATA_W_LSB 0x20
#define QUA_LSB 16384 //2^14

#define NDOF_MODE 0x0C

#define EUL_DATA_X_LSB 0x1A
#define EUL_DATA_X_MSB 0x1B
#define EUL_LSB 16.0


#endif /* SRC_BNO055_REGISTERS_H_ */





