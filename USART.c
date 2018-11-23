/*
 * UART.c
 *
 *  Created on: 2018/08/12
 *      Author: Ehab Bellkasy
 */

 
  
#include "types.h"
#include "hw_types.h"
#include "AVR_USART_Reg.h"
#include "USART.h"
#include <avr/interrupt.h>


gUSARTStdErr 	USARTInitCfg(gUSARTCfg_t * obj){

	/// USART Configuration 
	/**
	 ** @brief this function will configure USART
	 ** @param gUSARTCfg_t * obj
	 ** @return   gUSARTStdErr
	 */
	gUSARTStdErr 	ret 	= NO_ERRORS;
	uint16_t		UBRRVAL = 0 ;
	 
	 //-------------------------
	 // set/reset  Double the USART Transmission Speed
		if (obj -> doubleTransmission == Enable) {
			HW_REG(UCSRA) |= (1 << U2X);
		}
		else if (obj -> doubleTransmission == DISABLE) {
			HW_REG(UCSRA) &= ~(1 << U2X);
		}
		else {
			ret |= USART_ENABLE_ERR;
		} // End of if statement
	 
	 //-------------------------
	 // set/reset   Multi-processor Communication Mode
		if (obj -> multiProcessor == Enable) {
			HW_REG(UCSRA) |= (1 << MPCM);
		}
		else if (obj -> multiProcessor == DISABLE) {
			HW_REG(UCSRA) &= ~(1 << MPCM);
		}
		else {
			ret |= USART_ENABLE_ERR;
		} // End of if statement
	 
	 //-------------------------
	 // set/reset   RX Complete Interrupt Enable
		if (obj -> interruptRXEnable == Enable) {
			HW_REG(UCSRB) |= (1 << RXCIE);
		}
		else if (obj -> interruptRXEnable == DISABLE) {
			HW_REG(UCSRB) &= ~(1 << RXCIE);
		}
		else {
			ret |= USART_RX_INTERRUPT_ERR;
		} // End of if statement
	 
	 //-------------------------
	 // set/reset   TX Complete Interrupt Enable
		if (obj -> interruptTXEnable == Enable) {
			HW_REG(UCSRB) |= (1 << TXCIE);
		}
		else if (obj -> interruptTXEnable == DISABLE) {
			HW_REG(UCSRB) &= ~(1 << TXCIE);
		}
		else {
			ret |= USART_TX_INTERRUPT_ERR;
		} // End of if statement
	 
	 //-------------------------
	 // set/reset   USART Data Register Empty Interrupt Enable
		if (obj -> interruptDataEmptyEnable == Enable) {
			HW_REG(UCSRB) |= (1 << UDRIE);
		}
		else if (obj -> interruptDataEmptyEnable == DISABLE) {
			HW_REG(UCSRB) &= ~(1 << UDRIE);
		}
		else {
			ret |= USART_DATA_EMPTY_INTERRUPT_ERR;
		} // End of if statement
	 
	 //-------------------------
	 // set/reset   Receiver Enable
		if (obj -> receiverEnable == Enable) {
			HW_REG(UCSRB) |= (1 << RXEN);
		}
		else if (obj -> receiverEnable == DISABLE) {
			HW_REG(UCSRB) &= ~(1 << RXEN);
		}
		else {
			ret |= USART_ENABLE_ERR;
		} // End of if statement
	 
	 //-------------------------
	 // set/reset   Transmitter Enable
		if (obj -> transmitterEnable == Enable) {
			HW_REG(UCSRB) |= (1 << TXEN);
		}
		else if (obj -> transmitterEnable == DISABLE) {
			HW_REG(UCSRB) &= ~(1 << TXEN);
		}
		else {
			ret |= USART_ENABLE_ERR;
		} // End of if statement
	 
	 //-------------------------
	 // Select   USART Register selects between accessing the UCSRC orthe UBRRH Register.
	 // The URSEL must be one when reading or writing the UCSRC.
		HW_REG(UCSRC) |= (1 << URSEL);
	 
	 //-------------------------
	 // Select   USART Mode Select Asynchronous and Synchronous
		switch (obj-> selectMode ) {
				case ASYNCHRONOUS  :
				case SYNCHRONOUS  :
					HW_REG(UCSRC) &= ~((1 << UMSEL)   ); // clear all selected bits
					HW_REG(UCSRC) |= obj-> selectMode &((1 << UMSEL)  ); // Set as define
					break;
				default :
					ret |= SELECT_MODE_ERR ;
		} // End of switch statement
		
	 //-------------------------
	 // Select   USART Parity Mode 
		switch (obj-> parityMode ) {
				case DISABLED  :
				case EVEN_PARITY  :
				case ODD_PARITY  :
					HW_REG(UCSRC) &= ~((1 << UPM1) | (1 << UPM0)  ); // clear all selected bits
					HW_REG(UCSRC) |= obj-> parityMode &((1 << UPM1) | (1 << UPM0)  ); // Set as define
					break;
				default :
					ret |= PARITY_MODE_ERR ;
		} // End of switch statement
		
	 //-------------------------
	 // Select   USART  Stop Bit
		switch (obj-> selectStopBit ) {
				case ONE_BIT  :
				case TWO_BIT  :
					HW_REG(UCSRC) &= ~((1 << USBS)   ); // clear all selected bits
					HW_REG(UCSRC) |= obj-> selectStopBit &((1 << USBS)  ); // Set as define
					break;
				default :
					ret |= SELECT_STOP_BIT_ERR ;
		} // End of switch statement	
	
	 //-------------------------
	 // Select   USART  Clock Polarity
		switch (obj-> clockPolarity ) {
				case FALLING_EDGE  :
				case RISING_EDGE  :
					HW_REG(UCSRC) &= ~((1 << UCPOL)   ); // clear all selected bits
					HW_REG(UCSRC) |= obj-> clockPolarity &((1 << UCPOL)  ); // Set as define
					break;
				default :
					ret |= USART_ENABLE_ERR ;
		} // End of switch statement
	
	  
	 //-------------------------
	 // Select   USART Register selects between accessing the UCSRC orthe UBRRH Register.
	 // The URSEL must be zero when reading or writing the UBRRH.
		HW_REG(UCSRC) &= ~(1 << URSEL);	
	 
	 //-------------------------
	 // set		 USART Baud Rate Register Setting
		if ((obj->selectMode == ASYNCHRONOUS) & (obj->doubleTransmission == DISABLE)){
			UBRRVAL=(FCLK_SYSTEM/(obj->baudrate*16UL))-1; // Asynchronous Normal Mode
		}
		else if ((obj->selectMode == ASYNCHRONOUS) & (obj->doubleTransmission == Enable)){
			UBRRVAL=(FCLK_SYSTEM/(obj->baudrate* 8UL))-1; // Asynchronous Double Speed Mode 
		}
		else if ((obj->selectMode == SYNCHRONOUS) ){
			UBRRVAL=(FCLK_SYSTEM/(obj->baudrate* 2UL))-1; // Synchronous Master Mode
		}
		HW_REG(UBRRL)=UBRRVAL; 			//low byte
		HW_REG(UBRRH)=(UBRRVAL>>8); 	//high byte
	//-------------------------
	return ret ; 
} // End of Function [gUSARTStdErr 	USARTInitCfg(gUSARTCfg_t * obj)]





gUSARTStdErr 	USARTReadCfg(gUSARTCfg_t * obj){

	/// USART Read Configuration 
	/**
	 ** @brief this function will Read configuration of USART
	 ** @param gUSARTCfg_t * obj
	 ** @return   gUSARTStdErr
	 */
	gUSARTStdErr 	ret 	= NO_ERRORS;
	uint16_t		UBRRVAL = 0 ;
	 
	 //-------------------------
	 // Read  Double the USART Transmission Speed
		if ((HW_REG(UCSRA) & (1 << U2X))) {
			obj -> doubleTransmission = Enable;
		}
		else {
			obj -> doubleTransmission = DISABLE;
		} // End of if statement
	 
	 //-------------------------
	 // Read   Multi-processor Communication Mode
		if (HW_REG(UCSRA) & (1 << MPCM)) {
			obj -> multiProcessor = Enable;
		}
		else {
			obj -> multiProcessor = DISABLE;
		} // End of if statement
	 
	 //-------------------------
	 // Read   RX Complete Interrupt Enable
		if (HW_REG(UCSRB) & (1 << RXCIE)) {	
			obj -> interruptRXEnable = Enable;
		}
		else {
			obj -> interruptRXEnable = DISABLE;
		} // End of if statement
	 
	 //-------------------------
	 // Read   TX Complete Interrupt Enable
		if (HW_REG(UCSRB) & (1 << TXCIE)) {
			obj -> interruptTXEnable = Enable;
		}
		else {
			obj -> interruptTXEnable = DISABLE;
		} // End of if statement
	 
	 //-------------------------
	 // Read   USART Data Register Empty Interrupt Enable
		if (HW_REG(UCSRB) & (1 << UDRIE)) {	
			obj -> interruptDataEmptyEnable = Enable;
		}
		else {
			obj -> interruptDataEmptyEnable = DISABLE;
		} // End of if statement
	 
	 //-------------------------
	 // Read   Receiver Enable
		if (HW_REG(UCSRB) & (1 << RXEN)) {
			obj -> receiverEnable = Enable;
		}
		else {
			obj -> receiverEnable = DISABLE;
		} // End of if statement
	 
	 //-------------------------
	 // Read   Transmitter Enable
		if (HW_REG(UCSRB) & (1 << TXEN)) {
			obj -> transmitterEnable = Enable;
		}
		else {
			obj -> transmitterEnable = DISABLE;
		} // End of if statement
	 
	 //-------------------------
	 // Select   USART Register selects between accessing the UCSRC orthe UBRRH Register.
	 // The URSEL must be one when reading or writing the UCSRC.
		HW_REG(UCSRC) |= (1 << URSEL);
	 
	 //-------------------------
	 // Read   USART Mode Select Asynchronous and Synchronous
		
		obj-> selectMode &= ~((1 << UMSEL)   ); // clear all selected bits
		obj-> selectMode |= HW_REG(UCSRC) &((1 << UMSEL)  ); // Set as define
					
		
	 //-------------------------
	 // Read   USART Parity Mode 
		obj-> parityMode &= ~((1 << UPM1) | (1 << UPM0)  ); // clear all selected bits
		obj-> parityMode |= HW_REG(UCSRC) &((1 << UPM1) | (1 << UPM0)  ); // Set as define
		
	 //-------------------------
	 // Reaad   USART  Stop Bit
		
		obj-> selectStopBit &= ~((1 << USBS)   ); // clear all selected bits
		obj-> selectStopBit |= HW_REG(UCSRC) &((1 << USBS)  ); // Set as define
						
	
	 //-------------------------
	 // Read   USART  Clock Polarity
		obj-> clockPolarity &= ~((1 << UCPOL)   ); // clear all selected bits
		obj-> clockPolarity |= HW_REG(UCSRC) &((1 << UCPOL)  ); // Set as define
					
	
	 //-------------------------
	 // Read   USART Character Size 
					obj-> characterSize &= ~((1 << (UCSZ2 + 1)) ); // clear all selected bits
					obj-> characterSize |= ((HW_REG(UCSRB) & (1 << UCSZ2)) << 1) ; // Set as define
					
					obj-> characterSize &= ~((1 << UCSZ1) | (1 << UCSZ0)  ); // clear all selected bits
					obj-> characterSize |= HW_REG(UCSRC) &((1 << UCSZ1) | (1 << UCSZ0)  ); // Set as define
					
	
	 //-------------------------
	 // Select   USART Register selects between accessing the UCSRC orthe UBRRH Register.
	 // The URSEL must be zero when reading or writing the UBRRH.
		HW_REG(UCSRC) &= ~(1 << URSEL);	
	 
	 //-------------------------
	 // set		 USART Baud Rate Register Setting
		UBRRVAL = (HW_REG(UBRRH)<<8) + HW_REG(UBRRL) ;
		if ((obj->selectMode == ASYNCHRONOUS) & (obj->doubleTransmission == DISABLE)){
			obj->baudrate=(FCLK_SYSTEM/((UBRRVAL + 1)*16UL)); // Asynchronous Normal Mode
		}
		else if ((obj->selectMode == ASYNCHRONOUS) & (obj->doubleTransmission == Enable)){
			obj->baudrate=(FCLK_SYSTEM/((UBRRVAL + 1)* 8UL)); // Asynchronous Double Speed Mode 
		}
		else if ((obj->selectMode == SYNCHRONOUS) ){
			obj->baudrate=(FCLK_SYSTEM/((UBRRVAL + 1)* 2UL)); // Synchronous Master Mode
		}
	//-------------------------
	return ret ; 
} // End of Function [gUSARTStdErr 	USARTReadCfg(gUSARTCfg_t * obj)]





gUSARTStdErr 	USARTPutChr( uint8_t data ){
	/// USART Send Character vi USART 
	/**
	 ** @brief this function will Send Character vi USART
	 ** @param uint8_t data
	 ** @return   gUSARTStdErr
	 */
	
	//-------------------------
	while ( !( HW_REG(UCSRA) & (1<<UDRE))); // Wait for empty transmit buffer
	HW_REG(UDR) = data; //Put data into buffer, sends the data
} // End of Function [gUSARTStdErr 	USARTPutChr( uint8_t data )]



gUSARTStdErr 	USARTPutStr( uint8_t * str ){
	/// USART Send String vi USART 
	/**
	 ** @brief this function will Send String vi USART
	 ** @param uint8_t * str
	 ** @return   gUSARTStdErr
	 */
	
	//-------------------------
	
	uint8_t index=0;
	while(str[index])
	{
		USARTPutChr(str[index]);
		index++;
	} // End of while Loop
	
} // End of Function [gUSARTStdErr 	USARTPutStr( uint8_t * str )]


gUSARTStdErr 	USARTPutFrame( uint8_t * frame[] , uint16_t frameSize ){
	/// USART Send USARTPutFrame vi USART 
	/**
	 ** @brief this function will Send USARTPutFrame vi USART
	 ** @param uint8_t * frame[]
	 ** @param  uint16_t frameSize
	 ** @return   gUSARTStdErr
	 */
	
	//-------------------------
	
	uint8_t index=0;
	while(index < frameSize)
	{
		USARTPutChr(frame[index]);
		index++;
	} // End of while Loop
	
} // End of Function [gUSARTStdErr 	USARTPutFrame( uint8_t * frame[] , uint16_t frameSize )]



uint16_t 	USARTBufferReadLastFrame(uint8_t * userFrame []){
	/// USARTBufferReadLastFrame 
	/**
	 ** @brief this function will read the last frame that recived vi USART in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set frame size to be zero
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex = USARTbufferPreviousIndex;
	uint16_t tempIteration =0 ;
	if (((USARTbufferStats == READY) || (USARTbufferStats == FULL)) && (USARTbufferCurrentIndex > 0 ) ){
			
			//-------------------------------------------
			// lock the Buffer
			USARTbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((returnDataSize < USARTframeSize) && (tempIndex < USARTbufferCurrentIndex)){
				userFrame [returnDataSize] = USARTbufferArry [tempIndex] ;
				tempIndex ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// reset the index:-
			USARTbufferCurrentIndex = USARTbufferPreviousIndex ;
			USARTbufferPreviousIndex -= USARTframeSize;
			if (USARTbufferPreviousIndex < 0){
				USARTbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (USARTbufferCurrentIndex > 0){
				USARTbufferStats = READY;
			} else if (USARTbufferCurrentIndex == 0){
				USARTbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	USARTBufferReadLastFrame(uint8_t * userFrame [])]
	
	
//======================================================================================
	

uint16_t 	USARTBufferReadFirstFrame(uint8_t * userFrame []){
	/// USARTBufferReadFirstFrame 
	/**
	 ** @brief this function will read the First frame that recived vi USART in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set frame size to be zero
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex1 = 0;
	uint16_t tempIndex2 = 0;
	if (((USARTbufferStats == READY) || (USARTbufferStats == FULL)) && (USARTbufferCurrentIndex > 0 ) ){
			
			//-------------------------------------------
			// lock the Buffer
			USARTbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((returnDataSize < USARTframeSize) && (tempIndex1 < USARTbufferCurrentIndex)){
				userFrame [returnDataSize] = USARTbufferArry [tempIndex1] ;
				tempIndex1 ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// start Shiftin data Up in USARTbufferArry:-
			tempIndex2 = tempIndex1;
			tempIndex1 = 0;
			while( (tempIndex2 <= USARTbufferCurrentIndex)){
				USARTbufferArry [tempIndex1] = USARTbufferArry [tempIndex2] ;
				tempIndex1 ++;
				tempIndex2 ++;
			} // End of while Loop
			
			
			//-------------------------------------------
			// reset the index:-
			USARTbufferCurrentIndex -= USARTframeSize ;
			USARTbufferPreviousIndex -= USARTframeSize;
			if (USARTbufferPreviousIndex < 0){
				USARTbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (USARTbufferCurrentIndex > 0){
				USARTbufferStats = READY;
			} else if (USARTbufferCurrentIndex <= 0){
				USARTbufferCurrentIndex = 0 ;
				USARTbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	USARTBufferReadLastFrame(uint8_t * userFrame [])]
	
	
	//======================================================================================
	
	

uint16_t 	USARTBufferReadLastString(uint8_t * userFrame []){
	/// USARTBufferReadLastString 
	/**
	 ** @brief this function will read the last String that recived vi USART in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set a value to frame size
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex = USARTbufferPreviousIndex;
	uint16_t tempIteration =0 ;
	if (((USARTbufferStats == READY) || (USARTbufferStats == FULL)) && (USARTbufferCurrentIndex > 0 ) && (USARTframeSize == 0)){
			
			//-------------------------------------------
			// lock the Buffer
			USARTbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((tempIndex < BUFFER_SIZE) && (tempIndex < USARTbufferCurrentIndex)){
				userFrame [returnDataSize] = USARTbufferArry [tempIndex] ;
				tempIndex ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// reset the index:-
			USARTbufferCurrentIndex = USARTbufferPreviousIndex ;
			tempIndex = USARTbufferPreviousIndex - 2 ;
			while((USARTbufferArry [tempIndex] != '/0' ) && ( tempIndex > 0 )){
				tempIndex-- ;
			} // End of while Loop
			if (tempIndex <= 0){
				USARTbufferPreviousIndex =0 ;
			} else {
				USARTbufferPreviousIndex =tempIndex + 1 ;
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (USARTbufferCurrentIndex > 0){
				USARTbufferStats = READY;
			} else if (USARTbufferCurrentIndex == 0){
				USARTbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	USARTBufferReadLastFrame(uint8_t * userFrame [])]
	
	
	//======================================================================================
	
	
	
//======================================================================================
	

uint16_t 	USARTBufferReadFirstString(uint8_t * userFrame []){
	/// USARTBufferReadFirstString 
	/**
	 ** @brief this function will read the First String that recived vi USART in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set a value to frame size 
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex1 = 0;
	uint16_t tempIndex2 = 0;
	if (((USARTbufferStats == READY) || (USARTbufferStats == FULL)) && (USARTbufferCurrentIndex > 0 ) && (USARTframeSize == 0) ){
			
			//-------------------------------------------
			// lock the Buffer
			USARTbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while( (tempIndex1 < BUFFER_SIZE) && (USARTbufferArry [tempIndex1] != '/0' ) && (tempIndex1 < USARTbufferCurrentIndex)){
				userFrame [returnDataSize] = USARTbufferArry [tempIndex1] ;
				tempIndex1 ++;
				returnDataSize ++;
			} // End of while Loop
			userFrame [returnDataSize] = '/0' ;
			
			//-------------------------------------------
			// start Shiftin data Up in USARTbufferArry:-
			tempIndex2 = tempIndex1 + 1;
			tempIndex1 = 0;
			while( (tempIndex2 < BUFFER_SIZE) && (tempIndex2 <= USARTbufferCurrentIndex)){
				USARTbufferArry [tempIndex1] = USARTbufferArry [tempIndex2] ;
				tempIndex1 ++;
				tempIndex2 ++;
			} // End of while Loop
			
			
			//-------------------------------------------
			// reset the index:-
			USARTbufferCurrentIndex -= returnDataSize ;
			USARTbufferPreviousIndex -= returnDataSize;
			if (USARTbufferPreviousIndex < 0){
				USARTbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (USARTbufferCurrentIndex > 0){
				USARTbufferStats = READY;
			} else if (USARTbufferCurrentIndex <= 0){
				USARTbufferCurrentIndex = 0 ;
				USARTbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	USARTBufferReadLastFrame(uint8_t * userFrame [])]
	
	
	//======================================================================================
	
	
	
void USARTBufferEmpty (void) {
		/// USARTBufferEmpty 
		/**
		 ** @brief this function will Clear the buffer
		 ** @param    void : this function will take nothing 
		 ** @return   void : this function will return nothing  
		 */
		
		//-------------------------
		USARTbufferCurrentIndex= 0 ;
		USARTbufferPreviousIndex= 0 ;
		USARTframeIndex = 0 ;
		USARTbufferStats = EMPTY;
	}// End of Function[void USARTBufferEmpty (void)]
	
	
	
	
	
	//======================================================================================
	
	
	
void USARTBufferIgnoreLastRX (void) {
		/// USARTBufferEmpty 
		/**
		 ** @brief this function will Clear the current reciving data buffer
		 ** @param    void : this function will take nothing 
		 ** @return   void : this function will return nothing  
		 */
		
		//-------------------------
		USARTbufferCurrentIndex -= USARTframeIndex ;
		USARTframeIndex = 0 ;
		if (USARTbufferCurrentIndex > 0){
				USARTbufferStats = READY;
		} else if (USARTbufferCurrentIndex <= 0){
				USARTbufferCurrentIndex = 0 ;
				USARTbufferStats = EMPTY;
		} else {
				// Do Nothing
		}// End of if statement
	}// End of Function[void USARTBufferIgnoreLastRX (void)]	
	
	
	

ISR (USART_RXC_vect)
{
	/// interrupt service routine
	/**
	 ** @brief this function will Send USARTPutFrame vi USART
	 */
	
	//-------------------------
	
	if ((USARTbufferStats != OVERFLOW) && (USARTbufferStats != BUSY_READ)) //set busy if not overflow
		{
			/*
			if (usartbufferStats != BUSY_RX){
				USARTbufferPreviousIndex = USARTbufferCurrentIndex ;
				USARTframeIndex = 0 ;
			} else {//Do Nothing
			}// End of if statement
			*/
			USARTbufferStats = BUSY_RX ;
			USARTbufferArry [USARTbufferCurrentIndex] = HW_REG(UDR)  ;
			USARTbufferCurrentIndex++ ;
			USARTframeIndex++ ;
			
			if (USARTbufferCurrentIndex > BUFFER_SIZE){
				USARTbufferStats = OVERFLOW ;
			} else if ((USARTframeSize == 0) && (USARTbufferArry [USARTbufferCurrentIndex-1] =='/0')){
				USARTbufferStats = READY ;
				USARTbufferPreviousIndex = USARTbufferCurrentIndex - USARTframeIndex ;
				USARTframeIndex = 0 ;
			} else if ( (USARTframeSize > 0) && (USARTframeIndex >= USARTframeSize)){
				USARTbufferStats = READY ;
				USARTbufferPreviousIndex = USARTbufferCurrentIndex - USARTframeIndex ;
				USARTframeIndex = 0 ;
			}else {//Do Nothing
			}// End of if statement
	} else {// falier RX
	}// End of if statement
	
}//ISR(USART_RXC_vect)





