// Kevin Kemper
#ifndef BOOM_IO_H
#define BOOM_IO_H


// RS422/485
#define ENC_BAUD		9600

#define PORT_ENC0		PORTC
#define ENC0_USART		USARTC0
#define ENC0_RTS		1
#define ENC0_RX			2
#define ENC0_TX			3
#define ENC0_RTS_bm		(1<<ENC0_RTS)
#define ENC0_RX_bm		(1<<ENC0_RX)
#define ENC0_TX_bm		(1<<ENC0_TX)

#define PORT_ENC1		PORTC
#define ENC1_USART		USARTC1
#define ENC1_RTS		5
#define ENC1_RX			6
#define ENC1_TX			7
#define ENC1_RTS_bm		(1<<ENC1_RTS)
#define ENC1_RX_bm		(1<<ENC1_RX)
#define ENC1_TX_bm		(1<<ENC1_TX)

#define PORT_ENC2		PORTD
#define ENC2_USART		USARTD0
#define ENC2_RTS		1
#define ENC2_RX			2
#define ENC2_TX			3
#define ENC2_RTS_bm		(1<<ENC2_RTS)
#define ENC2_RX_bm		(1<<ENC2_RX)
#define ENC2_TX_bm		(1<<ENC2_TX)

// The hardstop
#define PORT_HS		PORTK
#define HS_bm		(1<<7)


#define BOOM_TILT_LOWER		11000
#define BOOM_TILT_UPPER		39000

#define BOOM_ROLL_LOWER		0
#define BOOM_ROLL_UPPER		65000

#define BOOM_PITCH_LOWER	0
#define BOOM_PITCH_UPPER	65000

#endif





