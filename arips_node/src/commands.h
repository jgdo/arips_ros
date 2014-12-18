#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

/**	common commands for both sides **/
enum CommonCmd
{
	CMD_DROP = 0,	// drop current command
	CMD_END,	// end command
	CMD_SPACE,	// add new argument
	USER_COMMAND, // offset for user commands
};

/** commands from Mini2440 to ARIPS **/
enum AripsCmd
{
// set servo positions beginning from given offset. if no offset is given reset servos to default position
	ARIPS_SET_SERVO = USER_COMMAND,	// [<servo offset> <value #0> [<value #1> [...]]];

// get values beginning from specified servo. if no channels is given, all 8 values are sent
	ARIPS_GET_SERVO,	// [<servo #> [<# of values>]];

// save default servo positions beginning from given offset.
	ARIPS_SAVE_SERVO,	// <servo offset> <value #0> [<value #1> [...]];

// get values beginning from specified adc channel. if no channels is given, all 8 values are sent
	ARIPS_GET_ADC,	// [<adc channel> [<# of values>]];

// set the motor speed. stops motor if no parameters specified
	ARIPS_SET_SPEED,	// [<speed left> <speed right>];

// turn on periphery power by setting specified bits. if no parameter is given turn on everything
	ARIPS_POWER_ON,	// [<bitmask>];
#define ARIPS_POWERMASK_ADC		(1 << 0)	// Bit 0: ADC power
#define ARIPS_POWERMASK_SERVO	(1 << 1)	// Bit 1: servo power

// turn off periphery power by setting specified bits. if no parameter is given turn off everything
// Bit 0: ADC power
// Bit 1: servo power
	ARIPS_POWER_OFF,	// [<bitmask>];

// get encoder count
	ARIPS_GET_ENCODER, // ;

// reset encoder count
	ARIPS_RESET_ENCODER,	// [<left count> <right count>];

// get ARIPS position calculated by wheel movement
	ARIPS_GET_POS, // ;

// reset ARIPS position and angle
	ARIPS_RESET_POS,	// ;

// save wheel distance
	ARIPS_SAVE_WHEEL_DIST, // <dist>

// turn on/off counter autosend
    ARIPS_SET_ENCODER_AUTOSEND,
};

inline bool IsCommand(unsigned char b) { return (b >= USER_COMMAND) && (b < 225); }

/** commands from ARIPS to Mini2440 **/
enum MiniCmd
{
// transfers servo positions
	MINI_SERVO = USER_COMMAND, // <servo offset> <servo value #0> [<servo value #1> [...]];

// transfers adc values to Mini2440
	MINI_ADC,	// <adc offset> <adc value #0> [<adc value #1> [...]];

// transfers encoder values
	MINI_ENCODER, // <left encoder> <right encoder>;

// transfers x/y position
	MINI_POSITION,	// <x position> <y position> <angle>;
	
// tramsfers encoder delta values
	MINI_ENCODER_DELTA,
};


/** Funktionen für den Motorcontroller **/
#ifdef __AVR__

#include <stdint.h>

#include "containers.h"

extern Fifo<int32_t, 16> aripsParameters;

int ProcessCommandByte(uint8_t b); // returns command on full command, otherways -1

void TransferU8(uint8_t value);
void TransferS8(int8_t value);
void TransferU16(uint16_t value);
void TransferS16(int16_t value);
void TransferU32(uint32_t value);
void TransferS32(int32_t value);

#endif // __AVR__

#endif // COMMANDS_H_INCLUDED
