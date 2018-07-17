/*
 * GPIO.c
 *
 *  Created on: 2018/06/09
 *      Author: Ehab Bellkasy
 */
#include "types.h"
#include "hw_types.h"
#include "AVR_GPIO_Reg.h"
#include "GPIO.h"

void static intConfig(gIntPinCfg_t * obj);
static void (*ptrInt0)(void);
static void (*ptrInt1)(void);
static void (*ptrInt2)(void);


/// GPIO Configuration by pin
/**
 ** @brief this function will configure GPIO by pin number
 ** @param gPinCfg_t * obj
 ** @return   gDioStdErr
 */ 
gDioStdErr DIOInitPin(gPinCfg_t * obj) {
	gDioStdErr ret = NO_ERRORS;

	
	if (obj->pin_no > MAX_PINS) {
		ret = PIN_NUM_FAULT;
	} else if (!(obj->port == PORT_A | obj->port == PORT_B | obj->port == PORT_C | obj->port == PORT_D)) {
		ret = PORT_FAULT;
	
	
	} else {
		if (obj->type == INPUT) {
			HW_REG(obj->port + DIR_OFFSET) &= ~(1 << obj->pin_no);
		} else if (obj->type == OUTPUT) {
			HW_REG(obj->port + DIR_OFFSET) |= (1 << obj->pin_no);
		} else {
			ret = PIN_TYPE_FAULT;
		}
	}
	return ret;
}

//==================================================================
/// GPIO write by pin
/**
 ** @brief this function will write output GPIO port by pin number
 ** @param gPortCfg_t * obj
 ** @param uint8_t value
 ** @return   gDioStdErr
 */ 
gDioStdErr DIOWritePin(gPinCfg_t * obj, uint8_t value) {
	
	gDioStdErr ret = NO_ERRORS;
	if (obj->pin_no > MAX_PINS) {
		ret = PIN_NUM_FAULT;
	
	} else if (!(obj->port == PORT_A | obj->port == PORT_B | obj->port == PORT_C | obj->port == PORT_D)) {
		ret = PORT_FAULT;
	
	} else {
		if (obj->type == OUTPUT) {
			if (value == HIGH) {
				HW_REG(obj->port + OUTPUT_OFFSET) |= (1 << obj->pin_no);
			} else if (value == LOW) {
				HW_REG(obj->port + OUTPUT_OFFSET) &= ~(1 << obj->pin_no);
			} else {
				ret = NOT_DEFINED_VALUE;
			}
		} else {
			ret = DIR_ERROR;
		}
	}
	return ret;
}

//==================================================================
/// GPIO read by pin
/**
 ** @brief this function will read input GPIO port by pin number
 ** @param gPortCfg_t * obj
 ** @param uint8_t * value
 ** @return   gDioStdErr
 */ 
gDioStdErr DIOReadPin(gPinCfg_t * obj, uint8_t *value) {
	
	gDioStdErr ret = NO_ERRORS;
	if (obj->pin_no > MAX_PINS) {
		ret = PIN_NUM_FAULT;
	} else if (!(obj->port == PORT_A | obj->port == PORT_B | obj->port == PORT_C | obj->port == PORT_D)) {
		ret = PORT_FAULT;
	
	
	} else {
		*value = (HW_REG(obj->port + INPUT_OFFSET) & (1 << obj->pin_no))	>> obj->pin_no;
	}
	return ret;
}

//==================================================================
/// GPIO Configuration by mask
/**
 ** @brief this function will configure GPIO port by mask
 ** @param gPortCfg_t * obj
 ** @return   gDioStdErr
 */ 
gDioStdErr DIOInitPort(gPortCfg_t * obj) {
	gDioStdErr ret = NO_ERRORS;
	uint8_t old_dir = HW_REG(obj->port + DIR_OFFSET);
	uint8_t new_dir = obj->direction;
	uint8_t change_mask = (old_dir ^ new_dir) &  obj->mask ;
	uint8_t set_mask = (old_dir ^ change_mask) &  change_mask ;
	uint8_t clear_mask = (set_mask ^ change_mask) &  obj->mask ;
	
	if (!(obj->port == PORT_A | obj->port == PORT_B | obj->port == PORT_C | obj->port == PORT_D)) {
		ret = PORT_FAULT;
	
	
	} else {
		
			HW_REG(obj->port + DIR_OFFSET) &= ~(clear_mask);
			HW_REG(obj->port + DIR_OFFSET) |= (set_mask);
			obj->direction = HW_REG(obj->port + DIR_OFFSET);
	}
	return ret;
}
//==================================================================
/// GPIO write on port by mask
/**
 ** @brief this function will write GPIO port by mask without Edge noise
 ** @param gPortCfg_t * obj
 ** @return   gDioStdErr
 */ 

gDioStdErr DIOwritePortByMask(gPortCfg_t * obj) {
	
	gDioStdErr ret = NO_ERRORS;
	uint8_t old_dir = HW_REG(obj->port + DIR_OFFSET);
	uint8_t new_dir = obj->direction;
	uint8_t change_mask = (old_dir ^ new_dir) &  obj->mask ;
	uint8_t set_mask = (old_dir ^ change_mask) &  change_mask ;
	uint8_t clear_mask = (set_mask ^ change_mask) &  obj->mask ;
	
	if (!(obj->port == PORT_A | obj->port == PORT_B | obj->port == PORT_C | obj->port == PORT_D)) {
		ret = PORT_FAULT;
	
	
	} else {
		
			HW_REG(obj->port + OUTPUT_OFFSET) &= ~(clear_mask);
			HW_REG(obj->port + OUTPUT_OFFSET) |= (set_mask);
			obj->write = HW_REG(obj->port + OUTPUT_OFFSET);
	}
	return ret;
}
//==================================================================
/// GPIO read from port by mask
/**
 ** @brief this function will read from GPIO port by mask 
 ** @param gPortCfg_t * obj
 ** @return   gDioStdErr
 */ 
 gDioStdErr DIOreadPortByMask(gPortCfg_t * obj) {
	/*Task*/
	gDioStdErr ret = NO_ERRORS;
	if (!(obj->port == PORT_A | obj->port == PORT_B | obj->port == PORT_C | obj->port == PORT_D)) {
		ret = PORT_FAULT;
	
	
	} else {
	*obj->read = (HW_REG(reg+ INPUT_OFFSET) & (obj->mask));
			
	}			

	
	return ret;
}
//==================================================================
/// GPIO Interrupt Configuration
/**
 ** @brief this function will configure GPIO Interrupt
 ** @param gIntPinCfg_t * obj
 ** @return   gDioStdErr
 */ 

gDioStdErr intConfig(gIntPinCfg_t * obj) {
	/*config all INT. reg*/
	gDioStdErr ret = NO_ERRORS;
	if (obj->IntNum == INT_0) {
		HW_REG(GICR) |= (1 << INT0);
		if (obj->intSenseNum == FALLING_EDGE) {
			HW_REG(MCUSR) |= (1 << ISC01);
			HW_REG(MCUSR) &= ~(1 << ISC00);
		} else if (obj->intSenseNum == RAISING_EDGE) {
			HW_REG(MCUSR) |= (1 << ISC01);
			HW_REG(MCUSR) |= (1 << ISC00);
		} else if (obj->intSenseNum == ANY_LOGICAL_CHANGE) {
			HW_REG(MCUSR) &= ~(1 << ISC01);
			HW_REG(MCUSR) |= (1 << ISC00);
		} else if (obj->intSenseNum == LOW_LEVEL) {
			HW_REG(MCUSR) &= ~(1 << ISC01);
			HW_REG(MCUSR) &= ~(1 << ISC00);
		
		} else {
			ret = INT_SENSE_FAULT;
		}
		ptrInt0 = obj->intFu;
		HW_REG(SREG) |= (1 << GI); //Enable Golobal Int.
		
	} else if (obj->IntNum == INT_1) {
		HW_REG(GICR) |= (1 << INT1);
		if (obj->intSenseNum == FALLING_EDGE) {
			HW_REG(MCUSR) |= (1 << ISC11);
			HW_REG(MCUSR) &= ~(1 << ISC10);
		} else if (obj->intSenseNum == RAISING_EDGE) {
			HW_REG(MCUSR) |= (1 << ISC11);
			HW_REG(MCUSR) |= (1 << ISC10);
		} else if (obj->intSenseNum == ANY_LOGICAL_CHANGE) {
			HW_REG(MCUSR) &= ~(1 << ISC11);
			HW_REG(MCUSR) |= (1 << ISC10);
		} else if (obj->intSenseNum == LOW_LEVEL) {
			HW_REG(MCUSR) &= ~(1 << ISC11);
			HW_REG(MCUSR) &= ~(1 << ISC10);
		} else {
			ret = INT_SENSE_FAULT;
		}
		ptrInt1 = obj->intFu;
		HW_REG(SREG) |= (1 << GI); //Enable Golobal Int.
		
	} else if(obj->IntNum == INT_2){
		HW_REG(GICR) |= (1<<INT2);
		if(obj->intSenseNum == FALLING_EDGE){
			HW_REG(MCUCSR) &= ~(1<<ISC2);
		} else if(obj->intSenseNum == RAISING_EDGE){
			HW_REG(MCUCSR) |= (1<<ISC2);
		} else {
			ret = INT_SENSE_FAULT;
		}
		ptrInt2 = obj->intFu;
		HW_REG(SREG) |= (1 << GI); //Enable Golobal Int.
	} else {
		ret = INT_NUM_FAULT;
	}
	
	return ret;
	
}

//==================================================================
/// GPIO interrupt service routine for INT0
/**
 ** @brief this function will executed when Interrupt trigger
 ** @param void
 ** @return void
 */ 

ISR(INT0_vect){
	(*ptrInt0)();
}
//==================================================================
/// GPIO interrupt service routine for INT1
/**
 ** @brief this function will executed when Interrupt trigger
 ** @param void
 ** @return void
 */ 

ISR(INT1_vect){
	(*ptrInt1)();
}
//==================================================================
/// GPIO interrupt service routine for INT2
/**
 ** @brief this function will executed when Interrupt trigger
 ** @param void
 ** @return void
 */ 

ISR(INT2_vect){
	(*ptrInt2)();
}