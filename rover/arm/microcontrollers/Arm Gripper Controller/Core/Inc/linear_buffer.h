/*
 * linear_buffer.h
 *
 *  Created on: May 19, 2024
 *      Author: abhay
 */

#ifndef INC_LINEAR_BUFFER_H_
#define INC_LINEAR_BUFFER_H_

/*
 * Includes
 */
#include "main.h"

/*
 * Macro defines
 */
#define BUFF_SIZE 256

/*
 * Structures
 */
typedef struct {
	uint8_t curr_index;
	uint8_t data[BUFF_SIZE];
} linear_buffer;

/*
 * Function declarations
 */
void linear_buff_flush(linear_buffer *buff);
void linear_buff_add(linear_buffer *buff, char input);

#endif /* INC_LINEAR_BUFFER_H_ */
