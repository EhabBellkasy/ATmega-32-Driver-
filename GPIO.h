/*
 * GPIO.h
 *
 *  Created on: 2018/06/07
 *      Author: Ehab Bellkasy
 */
 
 #ifndef GPIO_H_
#define GPIO_H_
#include "hw_types.h"
#include "AVR_GPIO_Reg.h"


typedef enum dioStdErr {
	NOT_DEFINED			= 0,
	NO_ERRORS 			= 1,
	PIN_NUM_FAULT 		= 3,
	PIN_TYPE_FAULT 		= 4,
	DIR_ERROR 			= 5,
	INT_SENSE_FAULT 	= 6,
	PORT_FAULT 			= 7,
	NOT_DEFINED_VALUE 	= 8,
	INT_NUM_FAULT 		= 9
} gDioStdErr;

typedef enum {
	INT_DISABLE = 0 , INT_ENABLE = 1 
} gIntStatus_t;

typedef enum {
	LOW_LEVEL =0 ,  ANY_LOGICAL_CHANGE = 1 , FALLING_EDGE = 2, RAISING_EDGE = 3
} gIntSenseControl_t;

typedef enum {
	INT_0, INT_1, INT_2
} gIntNum_t;

typedef struct {
	gIntStatus_t 		intStatus;
	gIntSenseControl_t 	intSenseNum;
	gIntNum_t 			IntNum;
	void (*intFu)(void);
} gIntPinCfg_t;


typedef enum {
	PORT_A=0x39, PORT_B=0x36, PORT_C=0x33, PORT_D=0x30
} gPortNum_t;

typedef enum {
	pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7
} gPinNum_t;

typedef enum {
	INPUT=0x00, OUTPUT=0xFF
} gDirectionType_t;

typedef struct pinCfg {
	gPortNum_t			port;
	gPinNum_t 			pin_no;
	gDirectionType_t 	type;
	
} gPinCfg_t;

typedef struct portCfg {
	gPortNum_t			port;
	uint8_t 			direction;
	uint8_t 			mask;
	uint8_t 			write;
	uint8_t *			read;
	
} gPortCfg_t;



gDioStdErr DIOInitPin(gPinCfg_t * obj) ;
gDioStdErr DIOWritePin(gPinCfg_t * obj, uint8_t value) ;
gDioStdErr DIOReadPin(gPinCfg_t * obj, uint8_t *value);
gDioStdErr DIOInitPort(gPortCfg_t * obj);
gDioStdErr DIOwritePortByMask(gPortCfg_t * obj);
gDioStdErr DIOreadPortByMask(gPortCfg_t * obj);
gDioStdErr intConfig(gIntPinCfg_t * obj);


#endif /* GPIO_H_ */