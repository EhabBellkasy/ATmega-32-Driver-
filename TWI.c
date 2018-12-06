/*
 * TWI.c
 *
 *  Created on: 2018/11/24
 *      Author: Ehab Bellkasy
 */

   
#include "types.h"
#include "hw_types.h"
#include "AVR_TWI_Reg.h"
#include "TWI.h"
#include <avr/interrupt.h>


/*
================================================================
Functions Defination
================================================================
*/



//===============================================================

gTWIStdErr_t 	TWIInitCfg(gIntTWICfg_t * obj){
	/// TWI Configuration 
	/**
	 ** @brief this function will configure TWI
	 ** @param gIntTWICfg_t * obj
	 ** @return   gTWIStdErr_t
	 */
	
	gTWIStdErr_t 	ret 	= NO_ERRORS;
	
	
	//-------------------------
	// set/reset TWI Enable Acknowledge Bit 
		if (obj -> enableAcknowledgeBit == ENABLE) {
			HW_REG(TWCR) |= (1 << TWEA);
		}
		else if (obj -> enableAcknowledgeBit == DISABLE) {
			HW_REG(TWCR) &= ~(1 << TWEA);
		}
		else {
			ret |= TWI_ENABLE_ERR;
		} // End of if statement
    
	
	//-------------------------
	// set/reset TWI Interrupt Enable 
		if (obj -> enableInterrupt == ENABLE) {
			HW_REG(TWCR) |= (1 << TWIE);
		}
		else if (obj -> enableInterrupt == DISABLE) {
			HW_REG(TWCR) &= ~(1 << TWIE);
		}
		else {
			ret |= TWI_ENABLE_ERR;
		} // End of if statement
    
	
	//-------------------------
	// set/reset TWI General Call Recognition Enable Bit
		if (obj -> enableGeneralCallRecognition == ENABLE) {
			HW_REG(TWAR) |= (1 << TWGCE);
		}
		else if (obj -> enableGeneralCallRecognition == DISABLE) {
			HW_REG(TWAR) &= ~(1 << TWGCE);
		}
		else {
			ret |= TWI_ENABLE_ERR;
		} // End of if statement
    
	//-------------------------
	// Select   TWI Clock  Prescaler
		switch (obj-> clockPrescaler ) {
				case DIVISION_FACTOR_1   :
				case DIVISION_FACTOR_4   :
				case DIVISION_FACTOR_16  :
				case DIVISION_FACTOR_64  :
					HW_REG(TWSR) &= ~((1 << TWPS1) | (1 << TWPS0)  ); // clear all selected bits
					HW_REG(TWSR) |= obj-> clockPrescaler &((1 << TWPS1) | (1 << TWPS0)  ); // Set as define
					break;
				
				default :
					ret |= CLOCK_PRESCALER_ERR ;
		
		} // End of switch statement
	
	
	//-------------------------
	// set TWI Bit Rate Register
		HW_REG(TWBR) = obj-> bitRate ;
		
    //-------------------------
	// set TWI (Slave) Address
		if (obj -> slaveAddress < 0b10000000) {
			HW_REG(TWAR) = ((obj -> slaveAddress) << 1);
		}
		else {
			ret |= SLAVE_ADDRESS_ERR;
		} // End of if statement
    
	
	//-------------------------
	// set/reset TWI  Enable 
		if (obj -> enableTWI == ENABLE) {
			HW_REG(TWCR) |= (1 << TWEN);
		}
		else if (obj -> enableTWI == DISABLE) {
			HW_REG(TWCR) &= ~(1 << TWEN);
		}
		else {
			ret |= TWI_ENABLE_ERR;
		} // End of if statement
    
	
	
	
	return ret ; 
}// End of Function [gTWIStdErr_t 	TWIInitCfg(gIntTWICfg_t * obj)]


//===============================================================


gTWIStdErr_t 	TWIReadCfg(gIntTWICfg_t * obj) {
	/// TWI Read Configuration 
	/**
	 ** @brief this function will Read configuration of TWI
	 ** @param gIntTWICfg_t * obj
	 ** @return   gTWIStdErr_t
	 */
	gTWIStdErr_t 	ret 	= NO_ERRORS;

		
	//-------------------------
	// Read  TWI Interrupt Enable
		if ((HW_REG(TWCR) & (1 << TWIE))) {
			obj -> enableInterrupt = ENABLE;
		}
		else {
			obj -> enableInterrupt = DISABLE;
		} // End of if statement
	
		
	//-------------------------
	// Read  TWI  Enable
		if ((HW_REG(TWCR) & (1 << TWEN))) {
			obj -> enableTWI = ENABLE;
		}
		else {
			obj -> enableTWI = DISABLE;
		} // End of if statement
	
		
	//-------------------------
	// Read  TWI  Enable Acknowledge Bit
		if ((HW_REG(TWCR) & (1 << TWEA))) {
			obj -> enableAcknowledgeBit = ENABLE;
		}
		else {
			obj -> enableAcknowledgeBit = DISABLE;
		} // End of if statement
	
		
	//-------------------------
	// Read  TWI  General Call Recognition Enable Bit
		if ((HW_REG(TWAR) & (1 << TWGCE))) {
			obj -> enableGeneralCallRecognition = ENABLE;
		}
		else {
			obj -> enableGeneralCallRecognition = DISABLE;
		} // End of if statement
	
	//-------------------------
	// Read  TWI   Clock  Prescaler
		obj -> clockPrescaler  = 0 ; // clear all
		obj -> clockPrescaler |= HW_REG(TWSR) &((1 << TWPS1) | (1 << TWPS0)  ); // Set as define

	//-------------------------
	// Read  TWI Bit Rate Register
		obj-> bitRate = HW_REG(TWBR) ;
	
	//-------------------------
	// Read  TWI (Slave) Address
		obj -> slaveAddress = ((HW_REG(TWAR)) >> 1);;
	
	
	
	
	return ret ; 
}// End of Function [gTWIStdErr_t 	TWIReadCfg(gIntTWICfg_t * obj)]

//===============================================================


gTWITXErr_t TWIMasterTransmitRequest( uint8_t slaveAddress , uint8_t* userData ){
/// TWI Send Frame or String vi TWI 
/**
 ** @brief this function will Send Frame or String vi TWI 
 ** @param uint8_t * userData is the first Address that will TX  for Frame it will take the size as it sitting in global variable  [extern uint16_t TWIframeSize].
 ** @param uint16_t slaveAddress Address of slave you want to send the Frame/String to
 ** @return   gTWITXErr_t 
 */	
	gTWITXErr_t ret =  NO_ERRORS ;
	//-------------------------
	// Check if  TWI  is Enable
		if ((HW_REG(TWCR) & (1 << TWEN))) {
			// TWI  is Enable
			//-------------------------
			//Check if  TWI  is Taken
				if (TWIuserDataPtrMTX == 0) {
					// TWI  is  Not Taken
					TWIslvWAddress = slaveAddress ;
					TWIuserDataPtrMTX = userData ;
					//-------------------------
					//Check if  TWI  is Busy
						if (TWIuserDataPtrMRX == 0) {
							// TWI  is  Not Busy
							HW_REG(TWCR) |= (1 << TWSTA); //START Condition
						}
						else {
							// TWI  is   Busy
							ret =  TWI_BUSY ;
						} // End of if statement
						
				}
				else {
					// TWI  is   Taken
					ret =  TWI_TAKEN ;
				} // End of if statement
		}
		else {
			// TWI  is  Disable
			ret =  TWI_IS_DISABLE_ERR ;
		} // End of if statement
	
	
	
	
	return ret ;
}// End of Function [gTWITXErr_t TWIMasterTransmitRequest( uint8_t slaveAddress , uint8_t* userData )]


//===============================================================


gTWITXErr_t TWIMasterReceiveRequest( uint8_t slaveAddress , uint8_t* userData ){
/// TWI Receive Frame or String vi TWI 
/**
 ** @brief this function will Receive Frame or String vi TWI 
 ** @param uint8_t * userData is the first Address that will RX  for Frame it will take the size as it sitting in global variable  [extern uint16_t TWIframeSize].
 ** @param uint16_t slaveAddress Address of slave you want to Receive the Frame/String to
 ** @return   gTWITXErr_t 
 */	
	gTWITXErr_t ret =  NO_ERRORS ;
	//-------------------------
	// Check if  TWI  is Enable
		if ((HW_REG(TWCR) & (1 << TWEN))) {
			// TWI  is Enable
			//-------------------------
			//Check if  TWI  is Taken
				if (TWIuserDataPtrMRX == 0) {
					// TWI  is  Not Taken
					TWIslvRAddress = slaveAddress ;
					TWIuserDataPtrMRX = userData ;
					//-------------------------
					//Check if  TWI  is Busy
						if (TWIuserDataPtrMTX == 0) {
							// TWI  is  Not Busy
							HW_REG(TWCR) |= (1 << TWSTA); //START Condition
						}
						else {
							// TWI  is   Busy
							ret =  TWI_BUSY ;
						} // End of if statement
						
				}
				else {
					// TWI  is   Taken
					ret =  TWI_TAKEN ;
				} // End of if statement
		}
		else {
			// TWI  is  Disable
			ret =  TWI_IS_DISABLE_ERR ;
		} // End of if statement
	
	
	
	
	return ret ;
}// End of Function [gTWITXErr_t TWIMasterReceiveRequest( uint8_t slaveAddress , uint8_t* userData )]


//===============================================================


gTWITXErr_t TWIslaveTransmitResponse( uint8_t* userData ){
/// TWI Send Frame or String vi TWI 
/**
 ** @brief this function will Send Frame or String vi TWI 
 ** @param uint8_t * userData is the first Address that will TX  for Frame it will take the size as it sitting in global variable  [extern uint16_t TWIframeSize].
 ** @return   gTWITXErr_t 
 */	
	gTWITXErr_t ret =  NO_ERRORS ;
	//-------------------------
	// Check if  TWI  is Enable
		if ((HW_REG(TWCR) & (1 << TWEN))) {
			// TWI  is Enable
			//-------------------------
			//Check if  TWI  is Taken
				if (TWIuserDataPtrSTX == 0) {
					// TWI  is  Not Taken
					TWIuserDataPtrSTX = userData ;
						
				}
				else {
					// TWI  is   Taken
					ret =  TWI_TAKEN ;
				} // End of if statement
		}
		else {
			// TWI  is  Disable
			ret =  TWI_IS_DISABLE_ERR ;
		} // End of if statement
	
	
	
	
	return ret ;
}// End of Function [gTWITXErr_t TWIslaveTransmitResponse( uint8_t slaveAddress , uint8_t* userData )]


//===============================================================


uint16_t 	TWIBufferReadLastFrame(uint8_t * userFrame []){
	/// TWIBufferReadLastFrame 
	/**
	 ** @brief this function will read the last frame that recived vi TWI in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set frame size to be zero
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex = TWIbufferPreviousIndex;
	uint16_t tempIteration =0 ;
	if (((TWIbufferStats == READY) || (TWIbufferStats == FULL)) && (TWIbufferCurrentIndex > 0 ) ){
			
			//-------------------------------------------
			// lock the Buffer
			TWIbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((returnDataSize < TWIframeSize) && (tempIndex < TWIbufferCurrentIndex)){
				userFrame [returnDataSize] = TWIbufferArry [tempIndex] ;
				tempIndex ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// reset the index:-
			TWIbufferCurrentIndex = TWIbufferPreviousIndex ;
			TWIbufferPreviousIndex -= TWIframeSize;
			if (TWIbufferPreviousIndex < 0){
				TWIbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (TWIbufferCurrentIndex > 0){
				TWIbufferStats = READY;
			} else if (TWIbufferCurrentIndex == 0){
				TWIbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	TWIBufferReadLastFrame(uint8_t * userFrame [])]
	


//===============================================================


uint16_t 	TWIBufferReadFirstFrame(uint8_t * userFrame []){
	/// TWIBufferReadFirstFrame 
	/**
	 ** @brief this function will read the First frame that recived vi TWI in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set frame size to be zero
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex1 = 0;
	uint16_t tempIndex2 = 0;
	if (((TWIbufferStats == READY) || (TWIbufferStats == FULL)) && (TWIbufferCurrentIndex > 0 ) ){
			
			//-------------------------------------------
			// lock the Buffer
			TWIbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((returnDataSize < TWIframeSize) && (tempIndex1 < TWIbufferCurrentIndex)){
				userFrame [returnDataSize] = TWIbufferArry [tempIndex1] ;
				tempIndex1 ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// start Shiftin data Up in TWIbufferArry:-
			tempIndex2 = tempIndex1;
			tempIndex1 = 0;
			while( (tempIndex2 <= TWIbufferCurrentIndex)){
				TWIbufferArry [tempIndex1] = TWIbufferArry [tempIndex2] ;
				tempIndex1 ++;
				tempIndex2 ++;
			} // End of while Loop
			
			
			//-------------------------------------------
			// reset the index:-
			TWIbufferCurrentIndex -= TWIframeSize ;
			TWIbufferPreviousIndex -= TWIframeSize;
			if (TWIbufferPreviousIndex < 0){
				TWIbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (TWIbufferCurrentIndex > 0){
				TWIbufferStats = READY;
			} else if (TWIbufferCurrentIndex <= 0){
				TWIbufferCurrentIndex = 0 ;
				TWIbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	TWIBufferReadLastFrame(uint8_t * userFrame [])]
	


//===============================================================


uint16_t 	TWIBufferReadLastString(uint8_t * userFrame []){
	/// TWIBufferReadLastString 
	/**
	 ** @brief this function will read the last String that recived vi TWI in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set a value to frame size
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex = TWIbufferPreviousIndex;
	uint16_t tempIteration =0 ;
	if (((TWIbufferStats == READY) || (TWIbufferStats == FULL)) && (TWIbufferCurrentIndex > 0 ) && (TWIframeSize == 0)){
			
			//-------------------------------------------
			// lock the Buffer
			TWIbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((tempIndex < BUFFER_SIZE) && (tempIndex < TWIbufferCurrentIndex)){
				userFrame [returnDataSize] = TWIbufferArry [tempIndex] ;
				tempIndex ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// reset the index:-
			TWIbufferCurrentIndex = TWIbufferPreviousIndex ;
			tempIndex = TWIbufferPreviousIndex - 2 ;
			while((TWIbufferArry [tempIndex] != '/0' ) && ( tempIndex > 0 )){
				tempIndex-- ;
			} // End of while Loop
			if (tempIndex <= 0){
				TWIbufferPreviousIndex =0 ;
			} else {
				TWIbufferPreviousIndex =tempIndex + 1 ;
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (TWIbufferCurrentIndex > 0){
				TWIbufferStats = READY;
			} else if (TWIbufferCurrentIndex == 0){
				TWIbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	TWIBufferReadLastFrame(uint8_t * userFrame [])]
	
	


//===============================================================


uint16_t 	TWIBufferReadFirstString(uint8_t * userFrame []){
	/// TWIBufferReadFirstString 
	/**
	 ** @brief this function will read the First String that recived vi TWI in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set a value to frame size 
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex1 = 0;
	uint16_t tempIndex2 = 0;
	if (((TWIbufferStats == READY) || (TWIbufferStats == FULL)) && (TWIbufferCurrentIndex > 0 ) && (TWIframeSize == 0) ){
			
			//-------------------------------------------
			// lock the Buffer
			TWIbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while( (tempIndex1 < BUFFER_SIZE) && (TWIbufferArry [tempIndex1] != '/0' ) && (tempIndex1 < TWIbufferCurrentIndex)){
				userFrame [returnDataSize] = TWIbufferArry [tempIndex1] ;
				tempIndex1 ++;
				returnDataSize ++;
			} // End of while Loop
			userFrame [returnDataSize] = '/0' ;
			
			//-------------------------------------------
			// start Shiftin data Up in TWIbufferArry:-
			tempIndex2 = tempIndex1 + 1;
			tempIndex1 = 0;
			while( (tempIndex2 < BUFFER_SIZE) && (tempIndex2 <= TWIbufferCurrentIndex)){
				TWIbufferArry [tempIndex1] = TWIbufferArry [tempIndex2] ;
				tempIndex1 ++;
				tempIndex2 ++;
			} // End of while Loop
			
			
			//-------------------------------------------
			// reset the index:-
			TWIbufferCurrentIndex -= returnDataSize ;
			TWIbufferPreviousIndex -= returnDataSize;
			if (TWIbufferPreviousIndex < 0){
				TWIbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (TWIbufferCurrentIndex > 0){
				TWIbufferStats = READY;
			} else if (TWIbufferCurrentIndex <= 0){
				TWIbufferCurrentIndex = 0 ;
				TWIbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	TWIBufferReadLastFrame(uint8_t * userFrame [])]
	


//===============================================================

	
void TWIBufferEmpty (void) {
		/// TWIBufferEmpty 
		/**
		 ** @brief this function will Clear the buffer
		 ** @param    void : this function will take nothing 
		 ** @return   void : this function will return nothing  
		 */
		
		//-------------------------
		TWIbufferCurrentIndex= 0 ;
		TWIbufferPreviousIndex= 0 ;
		TWIframeIndex = 0 ;
		TWIbufferStats = EMPTY;
	}// End of Function[void TWIBufferEmpty (void)]
	
	


//===============================================================

	
void TWIBufferIgnoreLastRX (void) {
		/// TWIBufferEmpty 
		/**
		 ** @brief this function will Clear the current reciving data buffer
		 ** @param    void : this function will take nothing 
		 ** @return   void : this function will return nothing  
		 */
		
		//-------------------------
		TWIbufferCurrentIndex -= TWIframeIndex ;
		TWIframeIndex = 0 ;
		if (TWIbufferCurrentIndex > 0){
				TWIbufferStats = READY;
		} else if (TWIbufferCurrentIndex <= 0){
				TWIbufferCurrentIndex = 0 ;
				TWIbufferStats = EMPTY;
		} else {
				// Do Nothing
		}// End of if statement
	}// End of Function[void TWIBufferIgnoreLastRX (void)]	


//===============================================================


ISR(TWI_vect){
	TWIStatus = (HW_REG(TWAR) & 0b11111100) ;
	switch ( TWIStatus ) {
				case M_TX_START_CONDITION   :
					/*	Next Action Taken 
					>> 	SLA+W will be transmitted; ACK or NOT ACK will be received.
					>> 	SLA+R will be transmitted; ACK or NOT ACK will be received.
					*/
					if (TWIuserDataPtrMTX){
						// Send the address of SLV_W
						HW_REG(TWDR) = TWIslvWAddress ;
						TWImasterTXstatus = START ;
						
					}
					else if (TWIuserDataPtrMRX){
						// Send the address of SLV_R
						HW_REG(TWDR) = TWIslvRAddress ;
						TWImasterRXstatus = START ;
					}
					else {
						// the START condition start by Accidant Send STOP condition
						HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
					}// End of if statement
					
					break;
					
				case M_TX_REPEATED_START   :
					/*	Next Action Taken 
					>> 	SLA+W will be transmitted; Logic will switch to Master Transmitter mode.
					>> 	SLA+R will be transmitted; Logic will switch to Master Receiver mode.
					*/
					if (TWIuserDataPtrMTX){
						// Send the address of SLV_W
						HW_REG(TWDR) = TWIslvWAddress ;
						TWImasterTXstatus = START ;
						
					}
					else if (TWIuserDataPtrMRX){
						// Send the address of SLV_R
						HW_REG(TWDR) = TWIslvRAddress ;
						TWImasterRXstatus = START ;
					}
					else {
						// the START condition start by Accidant Send STOP condition
						HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
					}// End of if statement
					
					break;
					
				case M_TX_SLA_W__ACK   :
					/*	Next Action Taken :-
					>>	Data byte will be transmitted and ACK or NOT ACK will be received.
					*	Repeated START will be transmitted.
					*	STOP condition will be transmitted and TWSTO Flag will be Reset.
					*	STOP condition followed by a START condition will be transmitted and TWSTO Flag will be Reset.
					*/
					TWIsendIndex = 0;
					HW_REG(TWDR) = TWIuserDataPtrMTX[TWIsendIndex] ;
					TWIsendIndex++ ;
					TWImasterTXstatus = TRANSFER ;
					break;
					
				case M_TX_SLA_W__NOT_ACK   :
					/*	Next Action Taken :-
					* 	Data byte will be transmitted and ACK or NOT ACK will be received.
					*	Repeated START will be transmitted.
					*	STOP condition will be transmitted and TWSTO Flag will be reset.
					>>	STOP condition followed by a START condition will be transmitted and TWSTO Flag will be reset.
					*/
					TWImasterTXstatus = NO_RESPONSE ;
					if (TWIreTXtrialsCount < TWIreTXtrialsMax ){
						// Send the STOP Condition
						HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
						// Send the START Condition
						HW_REG(TWCR) |= (1 << TWSTA); //START Condition
						TWIreTXtrialsCount++;
						
					}
					else {
							// Send the STOP Condition
							HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
							// Make the data pointer point to NULL to stop transmition 
							TWIuserDataPtrMTX = 0 ;
							// Reset Trials Count 
							TWIreTXtrialsCount = 0 ;
							//-------------------------
							//Check if  TWI  have anther Request
								if (TWIuserDataPtrMRX != 0) {
									// TWI  have  Request
									HW_REG(TWCR) |= (1 << TWSTA); //START Condition
								}
								else {
									// Do Nothing
								} // End of if statement
					} // End of if statement
					break;
					
				case M_TX_DATA__ACK   :
					/*	Next Action Taken :-
					>>	Data byte will be transmitted and ACK or NOT ACK will be received.
					*	Repeated START will be transmitted.
					*	STOP condition will be transmitted and TWSTO Flag will be reset.
					*	STOP condition followed by a START condition will be transmitted and TWSTO Flag will be reset.
					*/
					if (((TWIframeSize > 0 )&&(TWIsendIndex < TWIframeSize)) || ((TWIframeSize == 0 )&&(TWIuserDataPtrMTX[TWIsendIndex - 1] != '/0') && (TWIsendIndex >= 0) )  ) {
						TWImasterTXstatus = TRANSFER ;
						HW_REG(TWDR) = TWIuserDataPtrMTX[TWIsendIndex] ;
						TWIsendIndex++ ;
					} // #0 if
					if else (((TWIframeSize > 0 )&&(TWIsendIndex >= TWIframeSize)) || ( (TWIframeSize == 0 )&&(TWIuserDataPtrMTX[TWIsendIndex - 1] == '/0') && (TWIsendIndex >= 0) )){
						// the transmition is finish
							TWImasterTXstatus = FINISH ;
						// Make the data pointer point to NULL to stop transmition 
							TWIuserDataPtrMTX = 0 ;
						// Send the STOP Condition
							HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
						//Check if  TWI  have anther Request
								if (TWIuserDataPtrMRX != 0) {
									// TWI  have  Request
									HW_REG(TWCR) |= (1 << TWSTA); //START Condition
								}
								else {
									// Do Nothing
								} // End of if statement
					} //#1 if
					else {	
						// Do Nothing
					}// End of if statement
					break;
					
				case M_TX_DATA__NOT_ACK   :
					/*	Next Action Taken :-
					*	Data byte will be transmitted and ACK or NOT ACK will be received.
					*	Repeated START will be transmitted.
					*	STOP condition will be transmitted and TWSTO Flag will be reset.
					>>	STOP condition followed by a START condition will be transmitted and TWSTO Flag will be reset.
					*/
					
					TWImasterTXstatus = NO_RESPONSE ;
					if (TWIreTXtrialsCount < TWIreTXtrialsMax ){
						// Send the STOP Condition
						HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
						// Send the START Condition
						HW_REG(TWCR) |= (1 << TWSTA); //START Condition
						TWIreTXtrialsCount++;
						
					}
					else {
							// Send the STOP Condition
							HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
							// Make the data pointer point to NULL to stop transmition 
							TWIuserDataPtrMTX = 0 ;
							// Reset Trials Count 
							TWIreTXtrialsCount = 0 ;
							//-------------------------
							//Check if  TWI  have anther Request
								if (TWIuserDataPtrMRX != 0) {
									// TWI  have  Request
									HW_REG(TWCR) |= (1 << TWSTA); //START Condition
								}
								else {
									// Do Nothing
								} // End of if statement
					} // End of if statement
					
					break;
					
				case M_ARBITRATION_LOST   :
					/*	Next Action Taken :-
					*	Two-wire Serial Bus will be released and not addressed slave mode entered.
					>>	A START condition will be transmitted when the bus becomes free.
					*/
					
					// Send the START Condition
						HW_REG(TWCR) |= (1 << TWSTA); //START Condition
					
					break;
					
				case M_TX_SLA_R__ACK   :
					/*	Next Action Taken :-
					*	Data byte will be received and NOT ACK will be returned.
					>>	Data byte will be received and ACK will be returned.
					*/
					
					TWImasterTXstatus = TRANSFER ;
					TWIreceiveIndex = 0;
					TWIuserDataPtrMRX[TWIreceiveIndex] = HW_REG(TWDR)  ;
					TWIreceiveIndex++ ;
					
					
					break;
					
				case M_TX_SLA_R__NOT_ACK   :
					/*	Next Action Taken :-
					*	Repeated START will be transmitted.
					*	STOP condition will be transmitted and TWSTO Flag will be reset.
					>>	STOP condition followed by a START condition will be transmitted and TWSTO Flag will be reset.
					*/
					
					TWImasterRXstatus = NO_RESPONSE ;
					if (TWIreTXtrialsCount < TWIreTXtrialsMax ){
						// Send the STOP Condition
						HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
						// Send the START Condition
						HW_REG(TWCR) |= (1 << TWSTA); //START Condition
						TWIreTXtrialsCount++;
						
					}
					else {
							// Send the STOP Condition
							HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
							// Make the data pointer point to NULL to stop transmition 
							TWIuserDataPtrMRX = 0 ;
							// Reset Trials Count 
							TWIreTXtrialsCount = 0 ;
							//-------------------------
							//Check if  TWI  have anther Request
								if (TWIuserDataPtrMTX != 0) {
									// TWI  have  Request
									HW_REG(TWCR) |= (1 << TWSTA); //START Condition
								}
								else {
									// Do Nothing
								} // End of if statement
					} // End of if statement
					
					break;
					
				case M_RX_DATA__ACK   :
					/*	Next Action Taken :-
					*	Data byte will be received and NOT ACK will be returned.
					>>	Data byte will be received and ACK will be returned.
					*/
					
					if (((TWIframeSize > 0 )&&(TWIreceiveIndex < TWIframeSize)) || ((TWIframeSize == 0 )&&(TWIuserDataPtrMRX[TWIreceiveIndex - 1] != '/0') && (TWIreceiveIndex >= 0) )  ) {
						TWImasterRXstatus = TRANSFER ;
						TWIuserDataPtrMRX[TWIreceiveIndex] = HW_REG(TWDR)  ;
						TWIreceiveIndex++ ;
					} // #0 if
					if else (((TWIframeSize > 0 )&&(TWIreceiveIndex >= TWIframeSize)) || ( (TWIframeSize == 0 )&&(TWIuserDataPtrMRX[TWIreceiveIndex - 1] == '/0') && (TWIreceiveIndex >= 0) )){
						// the transmition is finish
							TWImasterRXstatus = FINISH ;
						// Make the data pointer point to NULL to stop transmition 
							TWIuserDataPtrMRX = 0 ;
						// Send the STOP Condition
							HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
						//Check if  TWI  have anther Request
								if (TWIuserDataPtrMTX != 0) {
									// TWI  have  Request
									HW_REG(TWCR) |= (1 << TWSTA); //START Condition
								}
								else {
									// Do Nothing
								} // End of if statement
					} //#1 if
					else {	
						// Do Nothing
					}// End of if statement
					
					break;
					
				case M_RX_DATA__NOT_ACK   :
					/*	Next Action Taken :-
					*	Repeated START will be transmitted.
					*	STOP condition will be transmitted and TWSTO Flag will be reset.
					>>	STOP condition followed by a START condition will be transmitted and TWSTO Flag will be reset.
					*/
					
					TWImasterRXstatus = NO_RESPONSE ;
					if (TWIreTXtrialsCount < TWIreTXtrialsMax ){
						// Send the STOP Condition
						HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
						// Send the START Condition
						HW_REG(TWCR) |= (1 << TWSTA); //START Condition
						TWIreTXtrialsCount++;
						
					}
					else {
							// Send the STOP Condition
							HW_REG(TWCR) |= (1 << TWSTO); // STOP Condition
							// Make the data pointer point to NULL to stop transmition 
							TWIuserDataPtrMRX = 0 ;
							// Reset Trials Count 
							TWIreTXtrialsCount = 0 ;
							//-------------------------
							//Check if  TWI  have anther Request
								if (TWIuserDataPtrMTX != 0) {
									// TWI  have  Request
									HW_REG(TWCR) |= (1 << TWSTA); //START Condition
								}
								else {
									// Do Nothing
								} // End of if statement
					} // End of if statement
					
					break;
					
				case S_RX_SLA_W__ACK   :
					/*	Next Action Taken :-
					*	Data byte will be received and NOT ACK will be returned.
					>>	Data byte will be received and ACK will be returned.
					*/	
				case S_ARBITRATION_LOST__OWN_SLA_W__ACK   :
					/*	Next Action Taken :-
					*	Data byte will be received and NOT ACK will be returned.
					>>	Data byte will be received and ACK will be returned.
					*/					
				case S_GENERAL_CALL__ACK   :
					/*	Next Action Taken :-
					*	Data byte will be received and NOT ACK will be returned.
					>>	Data byte will be received and ACK will be returned.
					*/					
				case S_ARBITRATION_LOST__GENERAL_CALL__ACK   :
					/*	Next Action Taken :-
					*	Data byte will be received and NOT ACK will be returned.
					>>	Data byte will be received and ACK will be returned.
					*/
					TWIslaveRXstatus = START ;
					// perpare the Buffer and clean out any RX processing		
					if (TWIbufferStats == BUSY_RX ){
						// ignor last RX
						TWIbufferCurrentIndex -= TWIframeIndex ;
						TWIframeIndex = 0 ;
						if (TWIbufferCurrentIndex > 0){
							TWIbufferStats = READY;
						}
						else if (TWIbufferCurrentIndex <= 0){
							TWIbufferCurrentIndex = 0 ;
							TWIbufferStats = EMPTY;
						} 
						else {
							// Do Nothing
						}// End of if statement
					}
					else {
						// Do Nothing
					}// End of if statement

					break;
					
				case S_RX_DATA__SLA_W__ACK   :
					/*	Next Action Taken :-
					*	Data byte will be received and NOT ACK will be returned.
					>>	Data byte will be received and ACK will be returned.
					*/
				case S_RX_DATA__GENERAL_CALL__ACK   :
					/*	Next Action Taken :-
					*	Data byte will be received and NOT ACK will be returned.
					>>	Data byte will be received and ACK will be returned.
					*/	
									
					if ((TWIbufferStats != OVERFLOW) && (TWIbufferStats != BUSY_READ)){ //set busy if not overflow
						TWIslaveRXstatus 	= TRANSFER ;		
						TWIbufferStats = BUSY_RX ;
						TWIbufferArry [TWIbufferCurrentIndex] = HW_REG(TWDR)  ;
						TWIbufferCurrentIndex++ ;
						TWIframeIndex++ ;
								
						if (TWIbufferCurrentIndex > BUFFER_SIZE){
							TWIbufferStats = OVERFLOW ;
							TWIslaveRXstatus 	= DISCONNECTION ;
						} 
						else if ((TWIframeSize == 0) && (TWIbufferArry [TWIbufferCurrentIndex-1] =='/0')){
							TWIbufferStats = READY ;
							TWIbufferPreviousIndex = TWIbufferCurrentIndex - TWIframeIndex ;
							TWIframeIndex = 0 ;
							TWIslaveRXstatus 	= FINISH ;
						} 
						else if ( (TWIframeSize > 0) && (TWIframeIndex >= TWIframeSize)){
							TWIbufferStats = READY ;
							TWIbufferPreviousIndex = TWIbufferCurrentIndex - TWIframeIndex ;
							TWIframeIndex = 0 ;
							TWIslaveRXstatus 	= FINISH ;
						}
						else {
							//Do Nothing
						}// End of if statement
					} 
					else {
						// falier RX
					}// End of if statement
					
					break;
					
				case S_RX_DATA__SLA_W__NOT_ACK   :
					/*	Next Action Taken :-
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA.
					*	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”.
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA; a START condition will be transmitted when the bus becomes free.
					>>	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”; a START condition will be transmitted when the bus becomes free.
					*/																								
				case S_RX_DATA__GENERAL_CALL__NOT_ACK   :
					/*	Next Action Taken :-
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA .
					*	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”.
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA; a START condition will be transmitted when the bus becomes free.
					>>	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”; a START condition will be transmitted when the bus becomes free.
					*/					
				case S_RX_STOP_OR_REPEATED_START_CONDITION   :
					/*	Next Action Taken :-
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA.
					*	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”.
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA; a START condition will be transmitted when the bus becomes free.
					>>	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”; a START condition will be transmitted when the bus becomes free.
					*/
					
					// perpare the Buffer and clean out any RX processing		
					if (TWIbufferStats == BUSY_RX ){
						TWIslaveRXstatus 	= DISCONNECTION ;
						// ignor last RX
						TWIbufferCurrentIndex -= TWIframeIndex ;
						TWIframeIndex = 0 ;
						if (TWIbufferCurrentIndex > 0){
							TWIbufferStats = READY;
						}
						else if (TWIbufferCurrentIndex <= 0){
							TWIbufferCurrentIndex = 0 ;
							TWIbufferStats = EMPTY;
						} 
						else {
							// Do Nothing
						}// End of if statement
					}
					else {
						// Do Nothing
					}// End of if statement

					//Check if  TWI  have anther Request
								if ((TWIuserDataPtrMTX != 0) || (TWIuserDataPtrMRX != 0)) {
									// TWI  have  Request
									HW_REG(TWCR) |= (1 << TWSTA); //START Condition
								}
								else {
									// Do Nothing
								} // End of if statement
					
					break;
					
				case S_RX_SLA_R__ACK   :
					/*	Next Action Taken :-
					*	Last data byte will be transmitted and NOT ACK should be received.
					>>	Data byte will be transmitted and ACK should be received.
					*/					
				case S_ARBITRATION_LOST__OWN_SLA_R__ACK   :
					/*	Next Action Taken :-
					*	Last data byte will be transmitted and NOT ACK should be received.
					>>	Data byte will be transmitted and ACK should be received.
					*/
					if (TWIuserDataPtrSTX != 0){ // Data is avaliable to TX
						TWIresponseIndex = 0;
						HW_REG(TWDR) = TWIuserDataPtrSTX[TWIresponseIndex] ;
						TWIresponseIndex++ ;
						TWIslaveTXstatus = TRANSFER ;
					}
					else {
						// NO data Avaliable to send back
						// Do nothing 
					}// End of if statement
					break;
					
				case S_TX_DATA__ACK   :
					/*	Next Action Taken :-
					*	Last data byte will be transmitted and NOT ACK should be received.
					>>	Data byte will be transmitted and ACK should be received.
					*/
					if (TWIuserDataPtrSTX != 0){ // Data is avaliable to TX
						if (((TWIframeSize > 0 )&&(TWIresponseIndex < TWIframeSize)) || ((TWIframeSize == 0 )&&(TWIuserDataPtrSTX[TWIresponseIndex - 1] != '/0') && (TWIresponseIndex >= 0) )  ) {
							TWIslaveTXstatus = TRANSFER ;
							HW_REG(TWDR) = TWIuserDataPtrSTX[TWIresponseIndex] ;
							TWIresponseIndex++ ;
						} // #0 if
						if else (((TWIframeSize > 0 )&&(TWIresponseIndex >= TWIframeSize)) || ( (TWIframeSize == 0 )&&(TWIuserDataPtrSTX[TWIresponseIndex - 1] == '/0') && (TWIresponseIndex >= 0) )){
							// the transmition is finish
								TWIslaveTXstatus = FINISH ;
							// Make the data pointer point to NULL to stop transmition 
								TWIuserDataPtrSTX = 0 ;
							
						} //#1 if
						else {	
							// Do Nothing
						}// End of if statement
					}
					else {
						// NO data Avaliable to send back
						// Do nothing 
					}// End of if statement
					break;
					
				case S_TX_DATA__NOT_ACK   :
					/*	Next Action Taken :-
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA .
					*	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”.
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA; a START condition will be transmitted when the bus  becomes free.
					>>	Switched to the not addressed Slave mode;own SLA will be recognized; GCA will be recognized if TWGCE = “1”;a START condition will be transmitted when the bus becomes free.
					*/					
				case S_TX_LAST_DATA   :
					/*	Next Action Taken :-
					*	Switched to the not addressed Slave mode;no recognition of own SLA or GCA.
					*	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”.
					*	Switched to the not addressed Slave mode; no recognition of own SLA or GCA;a START condition will be transmitted when the bus becomes free.
					>>	Switched to the not addressed Slave mode; own SLA will be recognized; GCA will be recognized if TWGCE = “1”; a START condition will be transmitted when the bus  becomes free.
					*/
					
					//Check if  TWI  have anther Request
								if ((TWIuserDataPtrMTX != 0) || (TWIuserDataPtrMRX != 0)) {
									// TWI  have  Request
									HW_REG(TWCR) |= (1 << TWSTA); //START Condition
								}
								else {
									// Do Nothing
								} // End of if statement
					
					break;
					
				
					
				
				default :
					// Do Nothing
					break;
		
		} // End of switch statement
} // End of Function[ISR(TWI_vect)]

//===============================================================


//===============================================================

