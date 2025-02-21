/*
 * mousePacket.h
 *
 *  Created on: Feb 1, 2025
 *      Author: nikod
 */

#ifndef INC_MOUSEPACKET_H_
#define INC_MOUSEPACKET_H_

							// BIT LOGIC VALUE MEANING or ADDITIONAL INFO
typedef struct {
	// FIRST BYTE
	uint8_t button_left;  	// bit nr. 0, "1" - if its pressed, "0" - if not
	uint8_t button_right;	// bit nr. 1, "1" - if its pressed, "0" - if not
	uint8_t button_mid;		// bit nr. 2, "1" - if its pressed, "0" - if not
	uint8_t alwaysOne;		// bit nr. 3, "1" - always, "0" - means data corruption
	uint8_t axis_X;			// bit nr. 4, "1" - mouse moving left, "0" - mouse moving right
	uint8_t axis_Y;			// bit nr. 5, "1" - mouse moving down, "0" - mouse moving up
	uint8_t overflow_X;		// bit nr. 6, "1" - third byte value overflowed , "0" - all good
	uint8_t overflow_Y;		// bit nr. 7, "1" - third byte value overflowed , "0" - all good

	// SECOND BYTE
	int8_t  move_X;			// max value for moving left is: -128, for right: 127

	// THIRD BYTE
	int8_t  move_Y;			// max value for moving down is: -128, for up: 127
} MousePacket;


#endif /* INC_MOUSEPACKET_H_ */
