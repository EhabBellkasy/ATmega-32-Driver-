/*
 * AVR_USART_Reg.h
 *
 *  Created on: 2018/06/17
 *      Author: Ehab Bellkasy
 */
 
 #ifndef AVR_USART_REG_H_
#define AVR_USART_REG_H_


//USART		registers 	addresses	BITS
#define 	UDR			0x2C		//The USART Transmit Data Buffer Register and USART Receive Data Buffer Registers share the same I/O address referred to as USART Data Register or UDR.
#define 	UBRRH		0x40		//The UBRRH Register shares the same I/O location as the UCSRC Register. The bit 15 selects between accessing the UBRRH or the UCSRC Register. It is read as zero when reading UBRRH. The URSEL must bezero when writing the UBRRH.
#define 	UBRRL		0x29		//USART Baud Rate Registers – UBRRL and UBRRH
#define 	UCSRA		0x2B		//USART Control and Status Register A .
									#define MPCM	0	// 	MPCM: Multi-processor Communication Mode , the incoming frames received by the USART receiver that do not contain address information will be ignored.
									#define U2X		1	// 	U2X: Double the USART Transmission Speed , This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous operation.
									#define PE		2	// 	PE: Parity Error , This bit is set if the next character in the receive buffer had a Parity Error when received .
									#define DOR		3	// 	DOR: Data OverRun , This bit is set if a Data OverRun condition is detected.
									#define FE		4	// 	FE: Frame Error , This bit is set if the next character in the receive buffer had a Frame Error when received.
									#define UDRE	5	// 	USART Data Register Empty , The UDRE Flag indicates if the transmit buffer (UDR) is ready to receive new data.
									#define TXC		6	// 	USART Transmit Complete , This flag bit is set when the entire frame in the transmit Shift Register has been shifted out .
									#define RXC		7	// 	USART Receive Complete , This flag bit is set when there are unread data in the receive buffer .
									
#define 	UCSRB		0x2A		//USART Control and Status Register B.
									#define TXB8	0	// 	TXB8: Transmit Data Bit 8 , TXB8 is the ninth data bit in the character tobe transmitted when operating with serial frames with nine data bits.
									#define RXB8	1	// 	RXB8: Receive Data Bit 8 , RXB8 is the ninth data bit of the received character when operating with serial frames with nine data bits.
									#define UCSZ2	2	// 	UCSZ2: Character Size
									#define TXEN	3	// 	TXEN: Transmitter Enable
									#define RXEN	4	// 	RXEN: Receiver Enable
									#define UDRIE	5	// 	UDRIE: USART Data Register Empty Interrupt Enable
									#define TXCIE	6	// 	TXCIE: TX Complete Interrupt Enable
									#define RXCIE	7	// 	RXCIE: RX Complete Interrupt Enable
									
#define 	UCSRC		0x40		//USART Control and Status Register C.
									#define UCPOL	0	// 	 UCPOL: Clock Polarity , used for Synchronous mode only. sets the relationship between data output change and data input sample, and the synchronous clock (XCK).
									#define UCSZ0	1	// 	UCSZ1:0: Character Size.
									#define UCSZ1	2	// 	UCSZ1:0: Character Size.
									#define USBS	3	// 	USBS: Stop Bit Select.
									#define UPM0	4	// 	UPM1:0: Parity Mode.
									#define UPM1	5	// 	UPM1:0: Parity Mode.
									#define UMSEL	6	// 	UMSEL: USART Mode Select , This bit selects between Asynchronousand Synchronous mode of operation.
									#define URSEL	7	// 	URSEL: Register Select , This bit selects between accessing the UCSRC orthe UBRRH Register. It is read as one when reading UCSRC. The URSEL must beone when writing the UCSRC.
									







#endif /* AVR_USART_REG_H_ */