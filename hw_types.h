/*
* hw_types.h
*
*  Create on: 2018/06/06
*	Author: Ehab Bellkasy
*/

#ifndef HW_TYPES_H_
#define HW_TYPES_H_


#define F_CPU	8000000ul

#define TURE	1
#define FALSE	0
#define HIGH	1
#define LOW		0

typedef unsigned char		uint8_t;
typedef unsigned short int 	uint16_t;
typedef unsigned long int 	uint32_t;

typedef char				int8_t;
typedef short int			int16_t;
typedef long int 			int32_t;

typedef unsigned char 		u8_t;
typedef unsigned short int 	u16_t;
typedef unsigned long int 	u32_t;


#define HW_REG(address)  		(*(volatile uint8_t*)(address))

#define Set_Bit(REG,BIT) 		(REG |= (1<<BIT))
#define Clear_Bit(REG,BIT)		(REG &= (~(1<<BIT)))
#define Toggle_Bit(REG,BIT)		(REG ^= (1<<BIT))
#define ROR(REG,NUM) 			(REG = ((REG >> NUM) | (REG << (8-NUM))))
#define ROL(REG,NUM) 			(REG = ((REG << NUM) | (REG >> (8-NUM))))
#define Bit_Is_Set(REG,BIT) 	(REG & (1<<BIT))
#define Bit_Is_Clear(REG,BIT) 	(!(REG & (1<<BIT)))

#define Set_Mask(REG,MASK) 		(REG |= MASK)
#define Clear_Mask(REG,MASK) 	(REG &= MASK)
#define Toggle_Mask(REG,MASK) 	(REG ^= MASK)
#define Mask_Is_Set(REG,MASK) 	((REG & MASK) == MASK)
#define Mask_Is_Clear(REG,MASK) (!((REG & MASK) == MASK))

#endif /* HW_TYPES_H_ */