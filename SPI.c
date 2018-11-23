/*
 * SPI.c
 *
 *  Created on: 2018/11/11
 *      Author: Ehab Bellkasy
 */

 
  
#include "types.h"
#include "hw_types.h"
#include "AVR_SPI_Reg.h"
#include "SPI.h"
#include <avr/interrupt.h>

gSPIStdErr_t 	SPIInitCfg(gIntSPICfg_t * obj){
	/// SPI Configuration 
	/**
	 ** @brief this function will configure SPI
	 ** @param gIntSPICfg_t * obj
	 ** @return   gSPIStdErr_t
	 */
	
	gSPIStdErr_t 	ret 	= NO_ERRORS;
	
	//-------------------------
	// set/reset SPIInterrupt Enable 
		if (obj -> enableInterrupt == ENABLE) {
			HW_REG(SPCR) |= (1 << SPIE);
		}
		else if (obj -> enableInterrupt == DISABLE) {
			HW_REG(SPCR) &= ~(1 << SPIE);
		}
		else {
			ret |= SPI_ENABLE_ERR;
		} // End of if statement
     
	//-------------------------
	// Select   SPI Mode Data Order LSB or MSB
		switch (obj-> dataOrder ) {
				case LSB  :
				case MSB  :
					HW_REG(SPCR) &= ~((1 << DORD)   ); // clear all selected bits
					HW_REG(SPCR) |= obj-> dataOrder &((1 << DORD)  ); // Set as define
					break;
				default :
					ret |= DATA_ORDER_ERR ;
		} // End of switch statement
	 
	 //-------------------------
	// Select   SPI Mode Master or Slave
		switch (obj-> selectMasterSlave ) {
				case SLAVE  :
				case MASTER  :
					HW_REG(SPCR) &= ~((1 << MSTR)   ); // clear all selected bits
					HW_REG(SPCR) |= obj-> selectMasterSlave &((1 << MSTR)  ); // Set as define
					break;
				default :
					ret |= MASTER_SLAVE_SELECT_ERR ;
		} // End of switch statement
		
	//-------------------------
	// Select   SPI Mode Clock Polarity idle low or idle high.
		switch (obj-> clockPolarity ) {
				case IDLE_LOW  :
				case IDLE_HIGH  :
					HW_REG(SPCR) &= ~((1 << CPOL)   ); // clear all selected bits
					HW_REG(SPCR) |= obj-> clockPolarity &((1 << CPOL)  ); // Set as define
					break;
				default :
					ret |= CLOCK_POLARITY_ERR ;
		} // End of switch statement
		
	//-------------------------
	// Select   SPI Mode Clock Phase leading edge or trailing edge.
		switch (obj-> clockPhase ) {
				case LEADING_EDGE  :
				case TRAILING_EDGE  :
					HW_REG(SPCR) &= ~((1 << CPHA)   ); // clear all selected bits
					HW_REG(SPCR) |= obj-> clockPhase &((1 << CPHA)  ); // Set as define
					break;
				default :
					ret |= CLOCK_PHASE_ERR ;
		} // End of switch statement
	
	//-------------------------
	// Select   USART Character Size 
		switch (obj-> selectClockRate ) {
				case DIVISION_FACTOR_4  :
				case DIVISION_FACTOR_16  :
				case DIVISION_FACTOR_64  :
				case DIVISION_FACTOR_128  :
					HW_REG(SPSR) &= ~((1 << SPI2X) );
					HW_REG(SPCR) &= ~((1 << SPR1) | (1 << SPR0)  ); // clear all selected bits
					HW_REG(SPCR) |= obj-> selectClockRate &((1 << SPR1) | (1 << SPR0)  ); // Set as define
					break;
				case DIVISION_FACTOR_2  :
				case DIVISION_FACTOR_8  :
				case DIVISION_FACTOR_32  :
				case DIVISION_FACTOR_D64  :
					HW_REG(SPSR) |= ((1 << SPI2X) );
					HW_REG(SPCR) &= ~((1 << SPR1) | (1 << SPR0)  ); // clear all selected bits
					HW_REG(SPCR) |= obj-> selectClockRate &((1 << SPR1) | (1 << SPR0)  ); // Set as define
					break;
				default :
					ret |= CLOCK_RATE_SELECT_ERR ;
		
		} // End of switch statement
	
	//-------------------------
	// set/reset SPI Enable 
		if (obj -> enableSPI == ENABLE) {
			HW_REG(SPCR) |= (1 << SPE);
		}
		else if (obj -> enableSPI == DISABLE) {
			HW_REG(SPCR) &= ~(1 << SPE);
		}
		else {
			ret |= SPI_ENABLE_ERR;
		} // End of if statement
     return ret ; 
}// End of Function [gSPIStdErr_t 	SPIInitCfg(gIntSPICfg_t * obj)]

gSPIStdErr_t 	SPIReadCfg(gIntSPICfg_t * obj){
	/// SPI Read Configuration 
	/**
	 ** @brief this function will Read configuration of SPI
	 ** @param gIntSPICfg_t * obj
	 ** @return   gSPIStdErr_t
	 */
	gUSARTStdErr 	ret 	= NO_ERRORS;
	
	//-------------------------
	// Read  SPI Interrupt Enable
		if ((HW_REG(SPCR) & (1 << SPIE))) {
			obj -> enableInterrupt = ENABLE;
		}
		else {
			obj -> enableInterrupt = DISABLE;
		} // End of if statement
	
	//-------------------------
	// Read  SPI  Enable
		if ((HW_REG(SPCR) & (1 << SPE))) {
			obj -> enableSPI = ENABLE;
		}
		else {
			obj -> enableSPI = DISABLE;
		} // End of if statement
	
	//-------------------------
	// Read  SPI  Data Order
		if ((HW_REG(SPCR) & (1 << DORD))) {
			obj -> dataOrder = MSB;
		}
		else {
			obj -> dataOrder = LSB;
		} // End of if statement
	 
	//-------------------------
	// Read  SPI  Master/Slave Select
		if ((HW_REG(SPCR) & (1 << MSTR))) {
			obj -> selectMasterSlave = MASTER;
		}
		else {
			obj -> selectMasterSlave = SLAVE;
		} // End of if statement
		
	//-------------------------
	// Read  SPI  Clock Polarity
		if ((HW_REG(SPCR) & (1 << CPOL))) {
			obj -> clockPolarity = IDLE_HIGH;
		}
		else {
			obj -> clockPolarity = IDLE_LOW;
		} // End of if statement	
		
	//-------------------------
	// Read  SPI   Clock Phase
		if ((HW_REG(SPCR) & (1 << CPHA))) {
			obj -> clockPhase = TRAILING_EDGE;
		}
		else {
			obj -> clockPhase = LEADING_EDGE;
		} // End of if statement
	//-------------------------
	// Read  SPI   Clock Rate
		obj -> selectClockRate  = 0 ; // clear all
		obj -> selectClockRate |= HW_REG(SPCR) &((1 << SPR1) | (1 << SPR0)  ); // Set as define
		obj -> selectClockRate |= ((HW_REG(SPSR) & (1 << SPI2X)) << 2) ; // Set as define
	
	return ret ; 
}// End of Function [gSPIStdErr_t 	SPIReadCfg(gIntSPICfg_t * obj)]
/*
gSPITXErr_t 	SPIPutChr( uint8_t data ) {
	//-------------------------
	// DISABLE  SPI   
		HW_REG(SPCR) &= ~(1 << SPE);
	//-------------------------
	// Start wirting in Buffer or SPI Data Register (SPDR) 
		if (SPIbufferCurrentIndex > SPIbufferSendIndex){ 
			// No data to send in Buffer 
			HW_REG(SPDR) = data;
			SPIbufferSendIndex = SPIbufferCurrentIndex ;
		}
		else{				
			// data are exist in Buffer 
			SPIbufferArry [SPIbufferSendIndex] = data ;
			SPIbufferSendIndex ++ ;
			
		}
	
	//-------------------------
	// DISABLE  SPI   
		HW_REG(SPCR) |= (1 << SPE);
		
}// End of Function [gSPIStdErr_t 	SPIPutChr( uint8_t data )]
*/


	
//======================================================================================







gSPITXErr_t 	SPIPutChr( uint8_t data ) {
/// SPI Send Character vi SPI 
/**
 ** @brief this function will Send Character vi SPI
 ** @param uint8_t data
 ** @return   gSPITXErr_t
*/
	gSPITXErr_t ret = NO_ERRORS ;
	if (HW_REG(SPCR) & (1 << SPE)) {
		//-------------------------
		// SPI is Enable
		
		//-------------------------
		// DISABLE  SPI   
			HW_REG(SPCR) &= ~(1 << SPE);
		//-------------------------
		// Start wirting in Buffer or SPI Data Register (SPDR) 
			if (SPIbufferCurrentIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Current Index is overflow send SPI_BUFFER_CURRENT_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_CURRENT_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferPreviousIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Previous Index is overflow send SPI_BUFFER_PREVIOUS_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_PREVIOUS_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferSendIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Send Index is overflow send SPI_BUFFER_SEND_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_SEND_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferCurrentIndex > SPIbufferSendIndex){ 
				// No data to send in Buffer 
				HW_REG(SPDR) = data;
				SPIbufferSendIndex = SPIbufferCurrentIndex ;
			}
			else if (SPIbufferCurrentIndex <= SPIbufferSendIndex){				
				// data are exist in Buffer 
				SPIbufferArry [SPIbufferSendIndex] = data ;
				SPIbufferSendIndex ++ ;
				
			}
		
		//-------------------------
		// Enable  SPI   
			HW_REG(SPCR) |= (1 << SPE);
	}
	else {
		//-------------------------
		// SPI is Disable send SPI_IS_DISABLE_ERR
		ret = SPI_IS_DISABLE_ERR ;
		
	}
	return ret ; 
}// End of Function [gSPIStdErr_t 	SPIPutChr( uint8_t data )]
	
//======================================================================================







gSPITXErr_t 	SPIPutStr( uint8_t * str ) {
/// SPI Send String vi SPI 
/**
 ** @brief this function will Send String vi SPI
 ** @param uint8_t * str
 ** @return   gSPITXErr_t
 */
	gSPITXErr_t ret = NO_ERRORS ;
	uint16_t strIndex = 0 ;
	uint16_t oldSendIndex = SPIbufferSendIndex ;
	if (HW_REG(SPCR) & (1 << SPE)) {
		//-------------------------
		// SPI is Enable
		
		//-------------------------
		// DISABLE  SPI   
			HW_REG(SPCR) &= ~(1 << SPE);
		//-------------------------
		// Start wirting in Buffer or SPI Data Register (SPDR) 
			if (SPIbufferCurrentIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Current Index is overflow send SPI_BUFFER_CURRENT_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_CURRENT_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferPreviousIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Previous Index is overflow send SPI_BUFFER_PREVIOUS_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_PREVIOUS_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferSendIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Send Index is overflow send SPI_BUFFER_SEND_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_SEND_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferCurrentIndex > SPIbufferSendIndex){ 
				// No data to send in Buffer 				
				SPIbufferSendIndex = SPIbufferCurrentIndex ;
				strIndex = 1;
				while( (str[strIndex]) && (SPIbufferSendIndex < SPI_BUFFER_SIZE) ){
					// Load the string into the Buffer
					SPIbufferArry [SPIbufferSendIndex] = str[strIndex] ;
					SPIbufferSendIndex ++ ;
					strIndex ++ ;
				} // End of While Loop
				if (SPIbufferSendIndex < SPI_BUFFER_SIZE){
					// str is saved in buffer start send the first Byte
					SPIbufferArry [SPIbufferSendIndex] = str[strIndex] ;
					HW_REG(SPDR) = str[0] ;
				}
				else {
					// No free space enough in buffer for strig sendNO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR
					SPIbufferSendIndex = SPIbufferCurrentIndex ;
					ret = NO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR ;
				}
				
			}
			else if (SPIbufferCurrentIndex <= SPIbufferSendIndex){				
				// data are exist in Buffer 
				strIndex = 0;
				while( (str[strIndex]) && (SPIbufferSendIndex < SPI_BUFFER_SIZE) ){
					// Load the string into the Buffer
					SPIbufferArry [SPIbufferSendIndex] = str[strIndex] ;
					SPIbufferSendIndex ++ ;
					strIndex ++ ;
				} // End of While Loop
				if (SPIbufferSendIndex < SPI_BUFFER_SIZE){
					// str is saved in buffer start send the first Byte
					SPIbufferArry [SPIbufferSendIndex] = str[strIndex] ;
					
				}
				else {
					// No free space enough in buffer for strig sendNO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR
					SPIbufferSendIndex = oldSendIndex ;
					ret = NO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR ;
				}
				
			}
		
		//-------------------------
		// Enable  SPI   
			HW_REG(SPCR) |= (1 << SPE);
	}
	else {
		//-------------------------
		// SPI is Disable send SPI_IS_DISABLE_ERR
		ret = SPI_IS_DISABLE_ERR ;
		
	}
 
	return ret ;  
 }// End of Function [gSPIStdErr_t 	SPIPutStr( uint8_t * str )]

	
//======================================================================================








gSPITXErr_t 	SPIPutFrame( uint8_t * frame[] , uint16_t frameSize ) {
/// SPI Send Frame vi SPI 
/**
 ** @brief this function will Send Frame vi SPI
 ** @param uint8_t * frame[]
 ** @param uint16_t frameSize
 ** @return   gSPITXErr_t
 */
	gSPITXErr_t ret = NO_ERRORS ;
	uint16_t frameIndex = 0 ;
	uint16_t oldSendIndex = SPIbufferSendIndex ;
	if (HW_REG(SPCR) & (1 << SPE)) {
		//-------------------------
		// SPI is Enable
		
		//-------------------------
		// DISABLE  SPI   
			HW_REG(SPCR) &= ~(1 << SPE);
		//-------------------------
		// Start wirting in Buffer or SPI Data Register (SPDR) 
			if (SPIbufferCurrentIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Current Index is overflow send SPI_BUFFER_CURRENT_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_CURRENT_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferPreviousIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Previous Index is overflow send SPI_BUFFER_PREVIOUS_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_PREVIOUS_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferSendIndex > SPI_BUFFER_SIZE){
				// SPI Buffer Send Index is overflow send SPI_BUFFER_SEND_INDEX_OVERFLOW_ERR
				ret = SPI_BUFFER_SEND_INDEX_OVERFLOW_ERR ;
			}
			else if (SPIbufferCurrentIndex > SPIbufferSendIndex){ 
				// No data to send in Buffer 				
				SPIbufferSendIndex = SPIbufferCurrentIndex ;
				frameIndex = 1;
				while( (frameIndex < frameSize) && (SPIbufferSendIndex < SPI_BUFFER_SIZE) ){
					// Load the frame into the Buffer
					SPIbufferArry [SPIbufferSendIndex] = frame[frameIndex] ;
					SPIbufferSendIndex ++ ;
					frameIndex ++ ;
				} // End of While Loop
				if (SPIbufferSendIndex < SPI_BUFFER_SIZE){
					// frame is saved in buffer start send the first Byte
					
					HW_REG(SPDR) = frame[0] ;
				}
				else {
					// No free space enough in buffer for frame sendNO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR
					SPIbufferSendIndex = SPIbufferCurrentIndex ;
					ret = NO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR ;
				}
				
			}
			else if (SPIbufferCurrentIndex <= SPIbufferSendIndex){				
				// data are exist in Buffer 
				frameIndex = 0;
				while( (frameIndex < frameSize) && (SPIbufferSendIndex < SPI_BUFFER_SIZE) ){
					// Load the frameing into the Buffer
					SPIbufferArry [SPIbufferSendIndex] = frame[frameIndex] ;
					SPIbufferSendIndex ++ ;
					frameIndex ++ ;
				} // End of While Loop
				if (SPIbufferSendIndex < SPI_BUFFER_SIZE){
					// frame is saved in buffer 
					
					
				}
				else {
					// No free space enough in buffer for frame send NO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR
					SPIbufferSendIndex = oldSendIndex ;
					ret = NO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR ;
				}
				
			}
		
		//-------------------------
		// Enable  SPI   
			HW_REG(SPCR) |= (1 << SPE);
	}
	else {
		//-------------------------
		// SPI is Disable send SPI_IS_DISABLE_ERR
		ret = SPI_IS_DISABLE_ERR ;
		
	}
 
	return ret ;  
 }// End of Function [gSPIStdErr_t SPIPutFrame( uint8_t * frame[] , uint16_t frameSize )]

	
//======================================================================================








	
void SPIBufferEmpty (void) {
		/// SPIBufferEmpty 
		/**
		 ** @brief this function will Clear the buffer
		 ** @param    void : this function will take nothing 
		 ** @return   void : this function will return nothing  
		 */
		
		//-------------------------
		SPIbufferCurrentIndex= 0 ;
		SPIbufferPreviousIndex= 0 ;
		SPIbufferSendIndex = 0 ;
		SPIframeIndex = 0 ;
		SPIbufferStats = EMPTY;
	}// End of Function[void SPIBufferEmpty (void)]
	
	
//======================================================================================







void SPIBufferIgnoreLastRX (void) {
		/// SPIBufferIgnoreLastRX 
		/**
		 ** @brief this function will Clear the current reciving data buffer
		 ** @param    void : this function will take nothing 
		 ** @return   void : this function will return nothing  
		 */
		
		//-------------------------
		uint16_t shiftIndex1 = SPIbufferCurrentIndex - SPIframeIndex ;
		uint16_t shiftIndex2 = SPIbufferCurrentIndex ;
		SPIbufferCurrentIndex -= SPIframeIndex ;
		SPIframeIndex = 0 ;
		if (SPIbufferCurrentIndex > 0){
				SPIbufferStats = READY;
		} else if (SPIbufferCurrentIndex <= 0){
				SPIbufferCurrentIndex = 0 ;
				shiftIndex1 = 0 ;
				SPIbufferStats = EMPTY;
		} else {
				// Do Nothing
		}// End of if statement
		
		if ( SPIbufferSendIndex <= shiftIndex2 ){
			// No data to send in Buffer
			SPIbufferSendIndex = 0 ;
		} 
		else if (SPIbufferSendIndex > shiftIndex2){
				// data in buffer need Shift up 
				while (shiftIndex2 < SPIbufferSendIndex){
					SPIbufferArry[shiftIndex1] = SPIbufferArry[shiftIndex2] ;
					shiftIndex1 ++ ;
					shiftIndex2 ++ ;
				} // End of While Loop
				SPIbufferSendIndex = shiftIndex1 ;
		} 
		else {
				// Do Nothing
		}// End of if statement
	}// End of Function[void SPIBufferIgnoreLastRX (void)]	
	
//======================================================================================








uint16_t 	SPIBufferReadLastFrame(uint8_t * userFrame []) {
/// SPIBufferReadLastFrame 
/**
 ** @brief this function will read the last frame that recived vi SPI in the buffer and delete it from the buffer
 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set frame size to be zero
 */

	//-------------------------
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex = SPIbufferPreviousIndex;
	uint16_t tempIteration =0 ;
	uint16_t shiftIndex1 = SPIbufferPreviousIndex ;
	uint16_t shiftIndex2 = SPIbufferCurrentIndex ;
	if (((SPIbufferStats == READY) || (SPIbufferStats == FULL)) && (SPIbufferCurrentIndex > 0 ) ){
			
			//-------------------------------------------
			// lock the Buffer
			SPIbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((returnDataSize < SPIframeSize) && (tempIndex < SPIbufferCurrentIndex)){
				userFrame [returnDataSize] = SPIbufferArry [tempIndex] ;
				tempIndex ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// reset the index:-
			SPIbufferCurrentIndex = SPIbufferPreviousIndex ;
			SPIbufferPreviousIndex -= SPIframeSize;
			if (SPIbufferPreviousIndex < 0){
				SPIbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// replace the send data :-
			if (shiftIndex1 < 0){
				shiftIndex1 =0 ;
			}
			else {
				// Do Nothing
			}// End of if statement
			
			if ( SPIbufferSendIndex <= shiftIndex2 ){
			// No data to send in Buffer
			SPIbufferSendIndex = 0 ;
			} 
			else if (SPIbufferSendIndex > shiftIndex2){
					// data in buffer need Shift up 
					while (shiftIndex2 < SPIbufferSendIndex){
						SPIbufferArry[shiftIndex1] = SPIbufferArry[shiftIndex2] ;
						shiftIndex1 ++ ;
						shiftIndex2 ++ ;
					} // End of While Loop
					SPIbufferSendIndex = shiftIndex1 ;
			} 
			else {
					// Do Nothing
			}// End of if statement
			
			
			//-------------------------------------------
			// releas the Buffer :-
			if (SPIbufferCurrentIndex > 0){
				SPIbufferStats = READY;
			} else if (SPIbufferCurrentIndex == 0){
				SPIbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
			
			
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
 
}// End of Function  [uint16_t 	SPIBufferReadLastFrame(uint8_t * userFrame [])]

	
//======================================================================================







uint16_t 	SPIBufferReadFirstFrame(uint8_t * userFrame []) {
/// SPIBufferReadFirstFrame 
/**
 ** @brief this function will read the First frame that recived vi SPI in the buffer and delete it from the buffer
 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set frame size to be zero
 */

	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex1 = 0;
	uint16_t tempIndex2 = 0;
	uint16_t shiftIndex1 = SPIbufferCurrentIndex - SPIframeSize ;
	uint16_t shiftIndex2 = SPIbufferCurrentIndex ;
	if (((SPIbufferStats == READY) || (SPIbufferStats == FULL)) && (SPIbufferCurrentIndex > 0 ) ){
			
			//-------------------------------------------
			// lock the Buffer
			SPIbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((returnDataSize < SPIframeSize) && (tempIndex1 < SPIbufferCurrentIndex)){
				userFrame [returnDataSize] = SPIbufferArry [tempIndex1] ;
				tempIndex1 ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// start Shiftin data Up in SPIbufferArry:-
			tempIndex2 = tempIndex1;
			tempIndex1 = 0;
			while( (tempIndex2 <= SPIbufferCurrentIndex)){
				SPIbufferArry [tempIndex1] = SPIbufferArry [tempIndex2] ;
				tempIndex1 ++;
				tempIndex2 ++;
			} // End of while Loop
			
			
			//-------------------------------------------
			// reset the index:-
			SPIbufferCurrentIndex -= SPIframeSize ;
			SPIbufferPreviousIndex -= SPIframeSize;
			if (SPIbufferPreviousIndex < 0){
				SPIbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// replace the send data :-
			if (shiftIndex1 < 0){
				shiftIndex1 =0 ;
			}
			else {
				// Do Nothing
			}// End of if statement
			
			if ( SPIbufferSendIndex <= shiftIndex2 ){
			// No data to send in Buffer
			SPIbufferSendIndex = 0 ;
			} 
			else if (SPIbufferSendIndex > shiftIndex2){
					// data in buffer need Shift up 
					while (shiftIndex2 < SPIbufferSendIndex){
						SPIbufferArry[shiftIndex1] = SPIbufferArry[shiftIndex2] ;
						shiftIndex1 ++ ;
						shiftIndex2 ++ ;
					} // End of While Loop
					SPIbufferSendIndex = shiftIndex1 ;
			} 
			else {
					// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// releas the Buffer :-
			if (SPIbufferCurrentIndex > 0){
				SPIbufferStats = READY;
			} else if (SPIbufferCurrentIndex <= 0){
				SPIbufferCurrentIndex = 0 ;
				SPIbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
 
 
 
	return returnDataSize;	
	}// End of Function  [uint16_t 	SPIBufferReadLastFrame(uint8_t * userFrame [])]
	

	
//======================================================================================







uint16_t 	SPIBufferReadLastString(uint8_t * userFrame []){
	/// SPIBufferReadLastString 
	/**
	 ** @brief this function will read the last String that recived vi SPI in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set a value to frame size
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex = SPIbufferPreviousIndex;
	uint16_t tempIteration =0 ;
	uint16_t shiftIndex1 = SPIbufferPreviousIndex ;
	uint16_t shiftIndex2 = SPIbufferCurrentIndex ;
	if (((SPIbufferStats == READY) || (SPIbufferStats == FULL)) && (SPIbufferCurrentIndex > 0 ) && (SPIframeSize == 0)){
			
			//-------------------------------------------
			// lock the Buffer
			SPIbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while((tempIndex < BUFFER_SIZE) && (tempIndex < SPIbufferCurrentIndex)){
				userFrame [returnDataSize] = SPIbufferArry [tempIndex] ;
				tempIndex ++;
				returnDataSize ++;
			} // End of while Loop
			
			//-------------------------------------------
			// reset the index:-
			SPIbufferCurrentIndex = SPIbufferPreviousIndex ;
			tempIndex = SPIbufferPreviousIndex - 2 ;
			while((SPIbufferArry [tempIndex] != '/0' ) && ( tempIndex > 0 )){
				tempIndex-- ;
			} // End of while Loop
			if (tempIndex <= 0){
				SPIbufferPreviousIndex =0 ;
			} else {
				SPIbufferPreviousIndex =tempIndex + 1 ;
			}// End of if statement
			
			//-------------------------------------------
			// replace the send data :-
			if (shiftIndex1 < 0){
				shiftIndex1 =0 ;
			}
			else {
				// Do Nothing
			}// End of if statement
			
			if ( SPIbufferSendIndex <= shiftIndex2 ){
			// No data to send in Buffer
			SPIbufferSendIndex = 0 ;
			} 
			else if (SPIbufferSendIndex > shiftIndex2){
					// data in buffer need Shift up 
					while (shiftIndex2 < SPIbufferSendIndex){
						SPIbufferArry[shiftIndex1] = SPIbufferArry[shiftIndex2] ;
						shiftIndex1 ++ ;
						shiftIndex2 ++ ;
					} // End of While Loop
					SPIbufferSendIndex = shiftIndex1 ;
			} 
			else {
					// Do Nothing
			}// End of if statement
			
			
			//-------------------------------------------
			// releas the Buffer :-
			if (SPIbufferCurrentIndex > 0){
				SPIbufferStats = READY;
			} else if (SPIbufferCurrentIndex == 0){
				SPIbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	SPIBufferReadLastFrame(uint8_t * userFrame [])]


	
//======================================================================================








uint16_t 	SPIBufferReadFirstString(uint8_t * userFrame []){
	/// SPIBufferReadFirstString 
	/**
	 ** @brief this function will read the First String that recived vi SPI in the buffer and delete it from the buffer
	 ** @param uint8_t * frame[]  : user give first element address of array  or any address element ready to take the data 
	 ** @return   returnDataSize : if return zero then a fail happened in buffer or it is busy or the user set a value to frame size 
	 */
	
	//-------------------------
	
	uint16_t returnDataSize =0 ; // if return zero that mean that the Buffer is Empity
	uint16_t tempIndex1 = 0;
	uint16_t tempIndex2 = 0;
	uint16_t shiftIndex1 = 0 ;
	uint16_t shiftIndex2 = SPIbufferCurrentIndex ;
	if (((SPIbufferStats == READY) || (SPIbufferStats == FULL)) && (SPIbufferCurrentIndex > 0 ) && (SPIframeSize == 0) ){
			
			//-------------------------------------------
			// lock the Buffer
			SPIbufferStats = BUSY_READ; 
			
			//-------------------------------------------
			// start Coping data to userFrame:-
			while( (tempIndex1 < BUFFER_SIZE) && (SPIbufferArry [tempIndex1] != '/0' ) && (tempIndex1 < SPIbufferCurrentIndex)){
				userFrame [returnDataSize] = SPIbufferArry [tempIndex1] ;
				tempIndex1 ++;
				returnDataSize ++;
			} // End of while Loop
			userFrame [returnDataSize] = '/0' ;
			
			//-------------------------------------------
			// start Shiftin data Up in SPIbufferArry:-
			tempIndex2 = tempIndex1 + 1;
			tempIndex1 = 0;
			while( (tempIndex2 < BUFFER_SIZE) && (tempIndex2 <= SPIbufferCurrentIndex)){
				SPIbufferArry [tempIndex1] = SPIbufferArry [tempIndex2] ;
				tempIndex1 ++;
				tempIndex2 ++;
			} // End of while Loop
			
			
			//-------------------------------------------
			// reset the index:-
			SPIbufferCurrentIndex -= returnDataSize ;
			SPIbufferPreviousIndex -= returnDataSize;
			if (SPIbufferPreviousIndex < 0){
				SPIbufferPreviousIndex =0 ;
			} else {
				// Do Nothing
			}// End of if statement
			
			//-------------------------------------------
			// replace the send data :-
			shiftIndex1 = SPIbufferCurrentIndex ; 
			if (shiftIndex1 < 0){
				shiftIndex1 =0 ;
			}
			else {
				// Do Nothing
			}// End of if statement
			
			if ( SPIbufferSendIndex <= shiftIndex2 ){
			// No data to send in Buffer
			SPIbufferSendIndex = 0 ;
			} 
			else if (SPIbufferSendIndex > shiftIndex2){
					// data in buffer need Shift up 
					while (shiftIndex2 < SPIbufferSendIndex){
						SPIbufferArry[shiftIndex1] = SPIbufferArry[shiftIndex2] ;
						shiftIndex1 ++ ;
						shiftIndex2 ++ ;
					} // End of While Loop
					SPIbufferSendIndex = shiftIndex1 ;
			} 
			else {
					// Do Nothing
			}// End of if statement
			
			
			
			//-------------------------------------------
			// releas the Buffer :-
			if (SPIbufferCurrentIndex > 0){
				SPIbufferStats = READY;
			} else if (SPIbufferCurrentIndex <= 0){
				SPIbufferCurrentIndex = 0 ;
				SPIbufferStats = EMPTY;
			} else {
				// Do Nothing
			}// End of if statement
	} else {
		// failling to read the Buffer maybe Busy , Overflow or Empity
		// the return will be zero
	}// End of if statement
	
	
	
	
	return returnDataSize;
	}// End of Function  [uint16_t 	SPIBufferReadLastFrame(uint8_t * userFrame [])]
	


	
//======================================================================================







ISR (SPI_vect){
	/// interrupt service routine
	/**
	 ** @brief this function will Save  RX String or Frame vi SPI in Buffer And prepare the next byte you want to send and put it in PDR
	 */
	
	//-------------------------
	
	if ((SPIbufferStats != OVERFLOW) && (SPIbufferStats != BUSY_READ)) //set busy if not overflow
		{
			uint8_t tempdata = SPIbufferArry [SPIbufferCurrentIndex] 
			SPIbufferStats = BUSY_RX ;
			SPIbufferArry [SPIbufferCurrentIndex] = HW_REG(SPDR)  ;
			SPIbufferCurrentIndex++ ;
			SPIframeIndex++ ;
			
			if (SPIbufferCurrentIndex > SPI_BUFFER_SIZE){
				SPIbufferStats = OVERFLOW ;
			} else if ((SPIframeSize == 0) && (SPIbufferArry [SPIbufferCurrentIndex-1] =='/0')){
				SPIbufferStats = READY ;
				SPIbufferPreviousIndex = SPIbufferCurrentIndex - SPIframeIndex ;
				SPIframeIndex = 0 ;
			} else if ( (SPIframeSize > 0) && (SPIframeIndex >= SPIframeSize)){
				SPIbufferStats = READY ;
				SPIbufferPreviousIndex = SPIbufferCurrentIndex - SPIframeIndex ;
				SPIframeIndex = 0 ;
			}else {//Do Nothing
			}// End of if statement
			
			// Look if there are Data need to send in the Buffer put it in SPDR
			if ((SPIbufferCurrentIndex < SPIbufferSendIndex) && (SPIbufferSendIndex <= SPI_BUFFER_SIZE )){
				// send data from buffer
				 HW_REG(SPDR) =  tempdata ;
			}else {
				//Do Nothing
				// No data in Buffer to send
			}// End of if statement
			
	} else {// falier RX
	}// End of if statement
	
}//ISR(SPI_RXC_vect)


	
//======================================================================================


	
//======================================================================================













