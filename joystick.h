#ifndef _joystick_h__
#define _joystick_h__

typedef struct {
	// size of reports built by buildReport
	char num_reports;

	int reportDescriptorSize;
	void *reportDescriptor; // must be in flash

	int deviceDescriptorSize; // if 0, use default
	void *deviceDescriptor; // must be in flash
	
	void (*init)(void);
	void (*update)(void);

	char (*changed)(unsigned char id);
	/**
	 * \param id Controller id (starting at 1 to match report IDs)
	 * \return The number of bytes written to buf.
	 * */
	char (*buildReport)(unsigned char *buf, char id);
} Joystick;

#endif // _joystick_h__


