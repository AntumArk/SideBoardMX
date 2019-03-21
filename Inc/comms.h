 /* USER CODE END 10 */
#include "main.h"
#include "stdio.h"
#include "string.h"


volatile uint8_t uart_buf[100];
volatile int16_t ch_buf[8];

UART_HandleTypeDef huart2;
void setScopeChannel(uint8_t ch, int16_t val);
void consoleScope();
void consoleLog(char *message);
 /* USER CODE END 10 */