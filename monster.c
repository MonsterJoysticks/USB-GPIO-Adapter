#define F_CPU   12000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "gamepad.h"
#include "monster.h"

#define GAMEPAD_BYTES	8	/* 2 byte per snes controller * 4 controllers */

/******** IO port definitions **************/
#define SNES_LATCH_DDR	DDRC
#define SNES_LATCH_PORT	PORTC
#define SNES_LATCH_BIT	(1<<4)

#define SNES_CLOCK_DDR	DDRC
#define SNES_CLOCK_PORT	PORTC
#define SNES_CLOCK_BIT	(1<<5)

#define SNES_DATA_PORT	PORTC
#define SNES_DATA_DDR	DDRC
#define SNES_DATA_PIN	PINC
#define SNES_DATA_BIT1	(1<<3)	/* controller 1 */

/********* IO port manipulation macros **********/
#define SNES_LATCH_LOW()	do { SNES_LATCH_PORT &= ~(SNES_LATCH_BIT); } while(0)
#define SNES_LATCH_HIGH()	do { SNES_LATCH_PORT |= SNES_LATCH_BIT; } while(0)
#define SNES_CLOCK_LOW()	do { SNES_CLOCK_PORT &= ~(SNES_CLOCK_BIT); } while(0)
#define SNES_CLOCK_HIGH()	do { SNES_CLOCK_PORT |= SNES_CLOCK_BIT; } while(0)

#define SNES_GET_DATA1()	(SNES_DATA_PIN & SNES_DATA_BIT1)

/*********** prototypes *************/
static void snesInit(void);
static void snesUpdate(void);
static char snesChanged(unsigned char report_id);
static char snesBuildReport(unsigned char *reportBuffer, char report_id);


// the most recent bytes we fetched from the controller
static unsigned char last_read_controller_bytes[GAMEPAD_BYTES];

// the most recently reported bytes
static unsigned char last_reported_controller_bytes[GAMEPAD_BYTES];

static void snesInit(void)
{
	unsigned char sreg;
	sreg = SREG;
	cli();
	
	// clock and latch as output
	SNES_LATCH_DDR |= SNES_LATCH_BIT;
	SNES_CLOCK_DDR |= SNES_CLOCK_BIT;
	
	// data as input
	SNES_DATA_DDR &= ~(SNES_DATA_BIT1 );
	// enable pullup. This should prevent random toggling of pins
	// when no controller is connected.
	SNES_DATA_PORT |= (SNES_DATA_BIT1 );

	// clock is normally high
	SNES_CLOCK_PORT |= SNES_CLOCK_BIT;

	// LATCH is Active HIGH
	SNES_LATCH_PORT &= ~(SNES_LATCH_BIT);

	snesUpdate();

	SREG = sreg;
}

/*
 *
       Clock Cycle     Button Reported
        ===========     ===============
        1               B
        2               Y
        3               Select
        4               Start
        5               Up on joypad
        6               Down on joypad
        7               Left on joypad
        8               Right on joypad
        9               A
        10              X
        11              L
        12              R
        13              Home Button // MJ additional button
        14              none (always high)
        15              none (always high)
        16              none (always high)
 *
 */

static void snesUpdate(void)
{
	int i;
	unsigned char tmp1=0;
		SNES_LATCH_HIGH();
		_delay_us(12);
		SNES_LATCH_LOW();

		for (i=0; i<8; i++)
		{
			_delay_us(6);
			SNES_CLOCK_LOW();
			
			tmp1 <<= 1;
			if (!SNES_GET_DATA1()) { tmp1 |= 1; }

			_delay_us(6);
			SNES_CLOCK_HIGH();
		}
		last_read_controller_bytes[0] = tmp1;

		for (i=0; i<8; i++)
		{
			_delay_us(6);

			SNES_CLOCK_LOW();

			// notice that this is different from above. We
			// want the bits to be in reverse-order
			tmp1 >>= 1;
			if ( !SNES_GET_DATA1() ) { tmp1 |= 0x80; }
			
			_delay_us(6);
			SNES_CLOCK_HIGH();
		}
    
        last_read_controller_bytes[1] = tmp1;

}

static char snesChanged(unsigned char report_id)
{
	report_id--; // first report is 1

	return memcmp(	&last_read_controller_bytes[report_id<<1], 
					&last_reported_controller_bytes[report_id<<1], 
					sizeof(&last_reported_controller_bytes[report_id<<1]));
}

static char getX(unsigned char nesByte1)
{
	char x = 128;
	if (nesByte1&0x1) { x = 255; }
	if (nesByte1&0x2) { x = 0; }
	return x;
}

static char getY(unsigned char nesByte1)
{
	char y = 128;
	if (nesByte1&0x4) { y = 255; }
	if (nesByte1&0x8) { y = 0; }
	return y;
}

/* Move the bits around so that identical NES and SNES buttons
 * use the same USB button IDs. */

static unsigned char snesReorderButtons(unsigned char bytes[2])
{
	unsigned char v;

	/* pack the snes button bits, which are on two bytes, in
	 * one single byte. */
    
    v =  (bytes[0]&0x80)>>7;
    v |= (bytes[0]&0x40)>>5;
    v |= (bytes[0]&0x20)>>3;
    v |= (bytes[0]&0x10)>>1;
    
    v |= (bytes[1]&0x1)<<4;
    v |= (bytes[1]&0x2)<<4;
    v |= (bytes[1]&0x4)<<4;
    v |= (bytes[1]&0x8)<<4;
	return v;
}

static unsigned char snesExtraButtons(unsigned char bytes[2])
{
    unsigned char v;
    v = (bytes[1]&0x10)>>4;
    
    return v;
}

static char snesBuildReport(unsigned char *reportBuffer, char id)
{
	int idx;

	if (id < 0 || id > 4)
		return 0;

	/* last_read_controller_bytes[] structure:
	 *
	 * [0] : controller 1, 8 first bits (dpad + start + sel + y|a + b)
	 * [1] : controller 1, 8 snes extra bits (4 lower bits are buttons)
	 *
	 *
	 */

	idx = id - 1;
	if (reportBuffer != NULL)
	{
		reportBuffer[0]=id;
		reportBuffer[1]=getX(last_read_controller_bytes[idx*2]);
		reportBuffer[2]=getY(last_read_controller_bytes[idx*2]);
        reportBuffer[3]=snesReorderButtons(&last_read_controller_bytes[idx*2]);
        reportBuffer[4]=snesExtraButtons(&last_read_controller_bytes[idx*2]);
	}

	memcpy(&last_reported_controller_bytes[idx*2],&last_read_controller_bytes[idx*2],sizeof(&last_read_controller_bytes[idx*2]));

	return 5;
}

const char fournsnes_usbHidReportDescriptor[] PROGMEM = {

	/* Controller and report_id 1 */
    0x05, 0x01,			// USAGE_PAGE (Generic Desktop)
    // 0x09, 0x04,			// USAGE (Joystick)
    0x09, 0x05,            // USAGE (Gamepad)
    0xa1, 0x01,			//	COLLECTION (Application)
    0x09, 0x01,			//		USAGE (Pointer)
    0xa1, 0x00,			//		COLLECTION (Physical)
	0x85, 0x01,			//			REPORT_ID (1)
	0x09, 0x30,			//			USAGE (X)
    0x09, 0x31,			//			USAGE (Y)
    0x15, 0x00,			//			LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,	//			LOGICAL_MAXIMUM (255)
    0x75, 0x08,			//			REPORT_SIZE (8)
    0x95, 0x02,			//			REPORT_COUNT (2)
    0x81, 0x02,			//			INPUT (Data,Var,Abs)

    0x05, 0x09,            //            USAGE_PAGE (Button)
    0x19, 1,            //           USAGE_MINIMUM (Button 1)
    0x29, 9,            //           USAGE_MAXIMUM (Button 8)
    0x15, 0x00,            //           LOGICAL_MINIMUM (0)
    0x25, 0x01,            //           LOGICAL_MAXIMUM (1)
    0x75, 1,            //             REPORT_SIZE (1)
    0x95, 8,            //            REPORT_COUNT (8)
    0x81, 0x02,            //            INPUT (Data,Var,Abs
    
    0x75, 1,            //             REPORT_SIZE (1)
    0x95, 1,            //            REPORT_COUNT (1)
    0x81, 0x02,            //            INPUT (Data,Var,Abs
    
    
	0xc0,				//		END_COLLECTION
    0xc0,				// END_COLLECTION

};

Gamepad SnesGamepad = {
	.num_reports 			= 1,
	.reportDescriptorSize	= sizeof(fournsnes_usbHidReportDescriptor),
	.init					= snesInit,
	.update					= snesUpdate,
	.changed				= snesChanged,
	.buildReport			= snesBuildReport
};

Gamepad *snesGetGamepad(void)
{
	SnesGamepad.reportDescriptor = (void*)fournsnes_usbHidReportDescriptor;

	return &SnesGamepad;
}

