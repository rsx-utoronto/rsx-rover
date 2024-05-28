/*
 * linear_buffer.c
 *
 *  Created on: May 19, 2024
 *      Author: abhay
 */

#include "linear_buffer.h"
#include <string.h>
#include <stdio.h>

/*
 * void linear_buff_flush(linear_buffer *buff)
 *
 * Sets the memory in the provided buffer to be 0
 * while also setting the current index of the buffer to
 * be 0.
 *
 * @parameters
 *
 * linear_buffer *buff: Pointer to the buffer that needs to be reset
 *
 * @returns
 * void: prints an error statement if the provided buffer pointer is NULL
 *
 */
void linear_buff_flush(linear_buffer *buff)
{
  // NULL check
  if (NULL == buff)
  {
	  fprintf(stderr, "ERROR: NULL buffer pointer\n");
	  return;
  }

  // Reset the buffer struct if not NULL
  buff->curr_index = 0;
  memset(buff->data, 0, BUFF_SIZE);
}

/*
 * void linear_buff_add(linear_buffer *buff, char input)
 *
 * Adds the provided input char variable to the provided
 * linear buffer buff.
 *
 * @parameters
 *
 * linear_buffer *buff: Pointer to the buffer to which input needs to be added
 * char input: Char variable that needs to be added to the buffer
 *
 * @returns
 * void: prints an error statement if the provided buffer pointer is NULL
 *
 */
void linear_buff_add(linear_buffer *buff, char input)
{
  // NULL check
  if (NULL == buff)
  {
	  fprintf(stderr, "ERROR: NULL buffer pointer\n");
	  return;
  }

  // Add the input char to the buffer and increase current index of the buffer
  buff->data[buff->curr_index] = (uint8_t) input;

  if(buff->curr_index < BUFF_SIZE - 1)
    buff->curr_index++;
}
