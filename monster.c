#define F_CPU   12000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "joystick.h"
#include "monster.h"

#define GAMEPAD_BYTES	8	/* 2 byte per snes controller * 4 controllers */

/******** IO port definitions **************/
#define LATCH_DDR	DDRC
#define LATCH_PORT	PORTC
#define LATCH_BIT	(1<<4)

#define CLOCK_DDR	DDRC
#define CLOCK_PORT	PORTC
#define CLOCK_BIT	(1<<5)

#define DATA_PORT	PORTC
#define DATA_DDR	DDRC
#define DATA_PIN	PINC
#define DATA_BIT1	(1<<3)	/* controller 1 */

/********* IO port manipulation macros **********/
#define LATCH_LOW()	do { LATCH_PORT &= ~(LATCH_BIT); } while(0)
#define LATCH_HIGH()	do { LATCH_PORT |= LATCH_BIT; } while(0)
#define CLOCK_LOW()	do { CLOCK_PORT &= ~(CLOCK_BIT); } while(0)
#define CLOCK_HIGH()	do { CLOCK_PORT |= CLOCK_BIT; } while(0)

#define GET_DATA1()	(DATA_PIN & DATA_BIT1)

/*********** prototypes *************/
static void joystickInit(void);
static void joystickUpdate(void);
static char joystickChanged(unsigned char report_id);
static char joystickBuildReport(unsigned char *reportBuffer, char report_id);


// the most recent bytes we fetched from the controller
static unsigned char last_read_controller_bytes[GAMEPAD_BYTES];

// the most recently reported bytes
static unsigned char *last_reported_controller_bytes[GAMEPAD_BYTES];

static void joystickInit(void)
{
	unsigned char sreg;
	sreg = SREG;
	cli();
	
	// clock and latch as output
	LATCH_DDR |= LATCH_BIT;
	CLOCK_DDR |= CLOCK_BIT;
	
	// data as input
	DATA_DDR &= ~(DATA_BIT1 );
	// enable pullup. This should prevent random toggling of pins
	// when no controller is connected.
	DATA_PORT |= (DATA_BIT1 );

	// clock is normally high
	CLOCK_PORT |= CLOCK_BIT;

	// LATCH is Active HIGH
	LATCH_PORT &= ~(LATCH_BIT);

	joystickUpdate();

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

static void joystickUpdate(void)
{
	int i;
	unsigned char tmp1=0;
		LATCH_HIGH();
		_delay_us(12);
		LATCH_LOW();

		for (i=0; i<8; i++)
		{
			_delay_us(6);
			CLOCK_LOW();
			
			tmp1 <<= 1;
			if (!GET_DATA1()) { tmp1 |= 1; }

			_delay_us(6);
			CLOCK_HIGH();
		}
		last_read_controller_bytes[0] = tmp1;

		for (i=0; i<8; i++)
		{
			_delay_us(6);

			CLOCK_LOW();

			// notice that this is different from above. We
			// want the bits to be in reverse-order
			tmp1 >>= 1;
			if ( !GET_DATA1() ) { tmp1 |= 0x80; }
			
			_delay_us(6);
			CLOCK_HIGH();
		}
    
        last_read_controller_bytes[1] = tmp1;

}

static char joystickChanged(unsigned char report_id)
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

static unsigned char joystickReorderButtons(unsigned char bytes[2])
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

static unsigned char joystickExtraButtons(unsigned char bytes[2])
{
    unsigned char v;
    v = (bytes[1]&0x10)>>4;
    
    return v;
}

static char joystickBuildReport(unsigned char *reportBuffer, char id)
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
		// reportBuffer[0]=id;
		reportBuffer[0]=getX(last_read_controller_bytes[idx*2]);
		reportBuffer[1]=getY(last_read_controller_bytes[idx*2]);
        reportBuffer[5]=joystickReorderButtons(&last_read_controller_bytes[idx*2]);
        reportBuffer[6]=joystickExtraButtons(&last_read_controller_bytes[idx*2]);
	}

	memcpy(&last_reported_controller_bytes[idx*2],&last_read_controller_bytes[idx*2],sizeof(&last_read_controller_bytes[idx*2]));

	return 7;
}

const char usbHidReportDescriptor[] PROGMEM = {

	/* Controller and report_id 1 */

	0x05, 0x01,	//	USAGE_PAGE (Generic Desktop)
    	0x09, 0x04,	//	USAGE (Joystick)
    		0xa1, 0x01,	//	COLLECTION (Application)
    			0xa1, 0x02,	//	COLLECTION (Logical)
	    			0x75, 0x08,	//	REPORT_SIZE (8)
	    			0x95, 0x05,	//	REPORT_COUNT (5)
	    			0x15, 0x00,	//	LOGICAL_MINIMUM (0)
	    			0x26, 0xff, 0x00,	//	LOGICAL_MAXIMUM (255)

	    			0x35, 0x00,	//	PHYSICAL_MINIMUM (0)
	    			0x46, 0xff, 0x00,	//	PHYSICAL_MAXIMUM (255)
					0x09, 0x30,	//	USAGE (X)
	   				
	   				0x09, 0x31,	//	USAGE (Y)
					0x09, 0x31,	//	USAGE (Y)
	    			0x09, 0x31,	//	USAGE (Y)
	    			0x09, 0x31,	//	USAGE (Y)

	    			0x81, 0x02,	//	INPUT (Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bitfield)
					/*
					0x75, 0x04,	//	REPORT_SIZE (4)
	    			0x95, 0x01,	//	REPORT_COUNT (1)
	    			0x81, 0x01,	//	INPUT (Constant, Array, Absolute)
					*/
	    			0x75, 0x01,	//	REPORT_SIZE (1)
	   				0x95, 0x0C,	//	REPORT_COUNT (12)
	   				0x25, 0x01,	//	LOGICAL_MAXIMUM (1)
	   				0x45, 0x01,	//	PHYSICAL_MAXIMUM (1)
	    			
	    			0x05, 0x09,	//	USAGE_PAGE (Button)
	    			0x19, 0x01,	//	USAGE_MINIMUM (Button 1)
	    			0x29, 0x09,	//	USAGE_MAXIMUM (Button 9)
	    			0x81, 0x02,	//	INPUT (Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bitfield)
	    /*
	    			0x06, 0x00, 0x0ff,	//	Usage Page (65280)
	    			0x75, 0x01,	//	REPORT_SIZE (1)
	   				0x95, 0x08,	//	REPORT_COUNT (8)
	   				0x25, 0x01,	//	LOGICAL_MAXIMUM (1)
	   				
	   				0x45, 0x01,	//	PHYSICAL_MAXIMUM (1)
	   				0x09, 0x01,	//	Usage 1 (0x1)
	   				0x81, 0x02,	//	INPUT (Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bitfield)
*/
				0xc0,	//	END_COLLECTION

   				0xa1, 0x02,	//	COLLECTION (Logical)
   					0x75, 0x08,	//	REPORT_SIZE (8)
   					0x95, 0x07,	//	REPORT_COUNT (7)
   					0x46, 0xff, 0x00,	//	PHYSICAL_MAXIMUM (255)

   					0x26, 0xff, 0x00,	//	LOGICAL_MAXIMUM (255)
   					0x09, 0x02,	//	Usage 2 (0x2)
   					0x91, 0x02,	//	OUTPUT (Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bitfield)
				0xc0,	//	END_COLLECTION

    		0xc0,	//	END_COLLECTION
};

Joystick joystick = {
	.num_reports 			= 1,
	.reportDescriptorSize	= sizeof(usbHidReportDescriptor),
	.init					= joystickInit,
	.update					= joystickUpdate,
	.changed				= joystickChanged,
	.buildReport			= joystickBuildReport
};

Joystick *getJoystick(void)
{
	joystick.reportDescriptor = (void*)usbHidReportDescriptor;

	return &joystick;
}

