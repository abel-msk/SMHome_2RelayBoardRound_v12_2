/*
 * project_errors.c
 *
 *  Created on: Nov 9, 2023
 *      Author: abel
 */

#include "smhome_errors.h"
#include "main.h"

void SMHome_error(int error_num) {

	  while (1) {

	  }
}



#ifdef DEBUG_PRINT_UART
	/**
	  * @brief  Retargets the C library printf function to the USART.
	  * @param  None
	  * @retval None
	  */
#ifdef __GNUC__
	int __io_putchar(int ch)
#else
	int fputc(int ch, FILE *f)
#endif
	{
	  /* Place your implementation of fputc here */
	  /* e.g. write a character to the USART2 and Loop until the end of transmission */
	  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

	  return ch;
	}
#else
	#ifdef DEBUG_PRINT_NUCLEO
		int _write(int file, char *ptr, int len )
		{
			int DataIdx;
			for (DataIdx =0; DataIdx<len; DataIdx++) {
				ITM_SendChar(*ptr++);
			}
			return len;
		}
	#endif /*DEBUG_PRINT_NUCLEO*/
#endif /*UART_DEBUG_PRINT*/
