/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mousePacket.h"		// mouseData structure, explaining what mouse packet data means
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===================================
   GPIO DEFINITIONS (Ports and Pins)
   =================================== */
#define PS2_CLOCK_PORT    GPIOB        	// PS2 CLOCK PORT
#define PS2_CLOCK_PIN     GPIO_PIN_15   // PS2 CLOCK PIN
#define PS2_DATA_PORT     GPIOB        	// PS2 DATA PORT
#define PS2_DATA_PIN      GPIO_PIN_14   // PS2 DATA PIN


/* ================================
   TIMER DEFINITIONS
   ================================ */
#define US_DELAY_TIMER 	  htim17		// Timer with frequency = 1 MHz


/* ================================
   UART DEFINITIONS
   ================================ */
#define SENDING_UART 	  huart2		// USART that will send our processed data


/* ================================
   PS2 COMMUNICATION PARAMETERS
   ================================ */
#define PACKET_SIZE				3		// Data packet size
#define START_BIT_INDEX			0		// Index of start bit
#define PARITY_BIT_INDEX		9		// Index of parity bit
#define STOP_BIT_INDEX			10		// Index of stop bit

#define PS2_CLOCK_DELAY_US		120 	// Delay after setting PS2_CLOCK to LOW (100us minimum)


/* ================================
   PS2 HOST TO MOUSE (DEVICE) COMMANDS
   ================================ */
#define PS2_COM_RESET 			0xFF	// Reset device
#define PS2_COM_RESET_NO_SELF 	0xF6	// Reset without self-test
#define PS2_COM_RESEND_BYTE 	0xFE	// Resend the last byte
#define PS2_COM_DATA_ENABLE		0xF4	// Enable data reporting (mouse starts transmitting packets)
#define PS2_COM_DATA_DISABLE	0xF5	// Disable data reporting (mouse stops transmitting packets)


/* ================================
   OPTIONAL CONFIGURATIONS
   ================================ */
// comment if you want to send empty data packets (with no value) when no data is received from the mouse
#define DONT_PROCESS_EMPTY_PACKET



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	volatile uint8_t bit_count = 0;         	// counter for received bits
	uint8_t data_byte = 0;         				// data byte from the received byte
	volatile uint8_t packet[PACKET_SIZE];		// PACKET_SIZE-byte packet from the mouse
	volatile uint8_t packet_index = 0;     		// counter for received packets
	volatile uint8_t packet_ready = 0;			// flag indicating that the packet is ready to be processed

	volatile uint8_t sendCommandState = 0;		// flag indicating that we are in the state of sending commands
	uint8_t command = 0;		   				// variable storing the command value
	volatile uint8_t commandBitCount = 0;   	// counter for sent bits of the command
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&US_DELAY_TIMER);  // Initializing us delay timer

  // Initializing the communication with mouse
  send_command(PS2_COM_RESET);
  HAL_Delay(200);
  send_command(PS2_COM_DATA_ENABLE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // when we get data from mouse this loop process it and send it via UART
	  while(packet_ready)
	  {
		  	// creating mousePacket structure to get data from packets
			MousePacket mouseData;

			// more info about mouse packet data structure available in "mousePacket.h" header file

			// getting first byte
			mouseData.button_left = (packet[0]&0x01);
			mouseData.button_right = ((packet[0]&0x02)>>1);
			mouseData.button_mid = ((packet[0]&0x04)>>2);
			mouseData.alwaysOne = ((packet[0]&0x08)>>3);
			mouseData.axis_X = ((packet[0]&0x10)>>4);
			mouseData.axis_Y = ((packet[0]&0x20)>>5);
			mouseData.overflow_X = ((packet[0]&0x40)>>6);
			mouseData.overflow_Y = ((packet[0]&0x80)>>7);


		  // checking if data hasn't been corrupted
		  if (mouseData.alwaysOne != 0x01)
		  {
			  // if it has - we quit
			  printf("error - bit 4 in packet one isnt a '1'");
			  packet_ready = 0;
			  break;
		  }

		  // getting info from second and third packets
		  mouseData.move_X = packet[1];
		  mouseData.move_Y = packet[2];

		  // MSB in "move_X/Y" should have the same value as "axis_X/Y" bit
		  if ( (((mouseData.move_X&0x80)>>7) != ((mouseData.axis_X&0x01)) ) ||
				  (((mouseData.move_Y&0x80)>>7) != ((mouseData.axis_Y&0x01))) )
		  {
			  // if they don't match - we quit
			  printf("error - direction dont match with the value");
			  packet_ready = 0;
			  break;
		  }

		  // constructing data packet with data that we just processed to send
		  uint8_t data_packet[50];
		  uint8_t Dsize;
		  Dsize = sprintf(data_packet, "LB:%d RB:%d MB:%d X:%d Y:%d OX:%d OY:%d\n",
		                  mouseData.button_left, mouseData.button_right, mouseData.button_mid,
						  mouseData.move_X, mouseData.move_Y, mouseData.overflow_X, mouseData.overflow_Y);

		  // odbieramy paczke danych w 1.1 ms, bo f myszki ~ 30kHz, narazie Timeout ustawiony na nieskonczony
		  // transmitting our data packet via UART
		  HAL_UART_Transmit(&SENDING_UART, &data_packet, Dsize, HAL_MAX_DELAY);

		  // after sending - we reset the packet_ready flag, because it was send successfully
		  packet_ready = 0;

	  }

// function available after commenting DONT_PROCESS_EMPTY_PACKET
#ifndef DONT_PROCESS_EMPTY_PACKET
	  // sending "zero" value data packets when we don't receive data from mouse
	  if (!packet_ready)
	  {
		  // constructing data packet with data that we just processed to send
		  uint8_t data_packet[50];
		  uint8_t Dsize;
		  Dsize = sprintf(data_packet, "LB:%d RB:%d MB:%d X:%d Y:%d OX:%d OY:%d\n",
		                  0, 0, 0, 0, 0, 0, 0);

		  // transmitting our data packet via UART
		  HAL_UART_Transmit(&SENDING_UART, &data_packet, Dsize, HAL_MAX_DELAY);
	  }
#endif


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// function that generates us delay needed when sending commands to the mouse
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&US_DELAY_TIMER,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&US_DELAY_TIMER) < us);  // wait for the counter to reach the us input in the parameter
}

// function thats send a message to the device, all commands are defined in "user defines" section
void send_command (uint8_t commandToSend)
{
		GPIO_InitTypeDef GPIO_InitStruct = {0};

	  // set the clock pin as output for controlling
	  GPIO_InitStruct.Pin = PS2_CLOCK_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  // use the data line to send data
	  GPIO_InitStruct.Pin = PS2_DATA_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(PS2_DATA_PORT, &GPIO_InitStruct);

	  // pull the clock low
	  HAL_GPIO_WritePin(PS2_CLOCK_PORT, PS2_CLOCK_PIN, GPIO_PIN_RESET);
	  delay_us(120); // wait for at least 120 us

	  // pull data low – this sends the start bit
	  HAL_GPIO_WritePin(PS2_DATA_PORT, PS2_DATA_PIN, GPIO_PIN_RESET);
	  delay_us(20);

	  // set the clock high - to send it
	  HAL_GPIO_WritePin(PS2_CLOCK_PORT, PS2_CLOCK_PIN, GPIO_PIN_SET);

	  // reconfigure the clock pin to detect falling edge interrupts,
	  // data will be set when we receive interrupt, so it can be set before next clock rising
	  GPIO_InitStruct.Pin = PS2_CLOCK_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(PS2_CLOCK_PORT, &GPIO_InitStruct);

	  // set flag that we will send a command in the interrupt and store the command
	  sendCommandState = 1;
	  command = commandToSend;

}

// callback function for interruptions
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    // reading the interrupt from the clock port
    if (GPIO_Pin == PS2_CLOCK_PIN)
    {
        // if we are in the state of sending commands
        // we have an interrupt on the falling edge, data is read on the rising edge, so we can set
        // the data on the falling edge and it will work fine
        if (sendCommandState)
        {
            GPIO_InitTypeDef GPIO_InitStruct = {0};

            // 0. sending the first 8 bits of data
            if (commandBitCount >= 0 && commandBitCount <= 7)
            {
                // shifting to get the current bit to send, moving towards MSB
                uint8_t dataState = command;
                dataState = dataState >>= commandBitCount;

                // saving the state we want to send on the data line
                HAL_GPIO_WritePin(PS2_DATA_PORT, PS2_DATA_PIN, dataState & 0x01);
                commandBitCount++;
            }

            // 1. sending the parity bit, MINUS ONE because the start bit is sent in send_command function
            else if (commandBitCount == PARITY_BIT_INDEX - 1)
            {
                uint8_t parityBit = 0x00; // creating the parity bit to send
                uint8_t var = command;    // creating a new variable equal to the received frame
                uint8_t count = 0;        // variable to count the number of ones

                // loop until the entire var (command) is zeroed
                while (var)
                {
                    count += var & 0x01; // if the least significant bit is "1" - add it
                    var >>= 1;           // then shift the variable one bit to the right
                }

                // checking if the received parity bit is correct
                // if the count is odd, it will have a 1 at the end, so parityBit = "0"
                if ((count & 0x01) == 0x0)
                {
                	// if count is not odd, we change parityBit to "1"
                    parityBit = 0x01;
                }

                HAL_GPIO_WritePin(PS2_DATA_PORT, PS2_DATA_PIN, parityBit);
                commandBitCount++;
            }

            // 2. after sending all data, we release the data line back to the mouse – stop bit, same as with parity
            else if (commandBitCount == STOP_BIT_INDEX - 1)
            {

                GPIO_InitStruct.Pin = PS2_DATA_PIN;
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
                HAL_GPIO_Init(PS2_DATA_PORT, &GPIO_InitStruct);
                commandBitCount++;
            }

            // 3. wait for the clock to go low, then wait a moment and everything should work fine
            else if (commandBitCount == 10)
            {
                // reset the data command and allow reading of data
                command = 0;
                commandBitCount = 0;
                sendCommandState = 0;
                delay_us(200);
            }
        } // end of the function sending data to the device

        // function for reading data from the mouse
        else
        {
            // 0. reading the state of the DATA pin
            uint8_t data_bit = HAL_GPIO_ReadPin(PS2_DATA_PORT, PS2_DATA_PIN);

            // 1. the first bit is the start bit equal to "0"
            if (bit_count == START_BIT_INDEX)
            {
                // if it's different from 0 - error
                if (data_bit != 0)
                {
                    // printing the error and resetting the "bit count"
                    bit_count = 0;
                    printf("error - start bit != 0");
                    return;
                }
            }

            // 2. reading the first byte of the frame
            else if (bit_count >= 1 && bit_count <= 8)
            {
                // data is transmitted as LSB
                data_byte >>= 1;            // shifting to the right by one on each reception

                if (data_bit) // if the received bit is one
                {
                    data_byte |= 0x80;  // set the current MSB as "1"
                }
            }

            // 3. reading the parity bit (odd parity - odd number of ones)
            else if (bit_count == PARITY_BIT_INDEX)
            {
                uint8_t var = data_byte; // creating a new variable equal to the received frame
                uint8_t count = 0;       // variable to count the number of ones

                // loop until the entire var (data_byte) is zeroed
                while (var)
                {
                    count += var & 0x01; // if the least significant bit is "1" - add it
                    var >>= 1;           // then shift the variable one bit to the right
                }

                // checking if the received parity bit is correct
                // if count is odd, it will have a 1 at the end, so data_bit = "0"
                if ((count & 0x01) != data_bit) {}
                else
                {
                    // if it doesn't match – throw an error
                    printf("error - parity bit does not match");
                    bit_count = 0;
                    return;
                }
            }

            // 4. stop bit - always "1"
            else if (bit_count == STOP_BIT_INDEX)
            {
                // if it's correct
                if (data_bit == 1)
                {
                    // if it's correct – store it in the data packet
                    packet[packet_index] = data_byte;
                    packet_index++; // increment to accept the next byte

                    // if we've received 3 bytes of data
                    if (packet_index == PACKET_SIZE)
                    {
                        packet_ready = 1; // inform that the packet is ready to be processed
                        packet_index = 0;  // reset the packet index
                    }
                }
                else
                {
                    // if it doesn't match – we would throw an error
                    printf("error - stop bit does not match");
                }

                // in both cases, reset the bit count and exit the function
                bit_count = 0;
                return;
            }

            // 0. if the bit is correctly received – increment the count by 1
            bit_count++;

        }
    }
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
