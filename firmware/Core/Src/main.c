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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cli.h"
#include "cli_defs.h"
#include "hdc2021.h"
#include "hci.h"
#include "hci_tl.h"
#include "hci_le.h"
#include "bluenrg_utils.h"
#include "bluenrg_gap.h"
#include "bluenrg_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_gatt_server.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GRN_LED TIM_CHANNEL_2
#define RED_LED TIM_CHANNEL_3
#define BLU_LED TIM_CHANNEL_4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint16_t led_duty = 1;
uint8_t led_direction = 1;
uint8_t cdc_connection_message_sent = 0;
extern volatile uint8_t cdc_connection_open_flag;
volatile uint8_t button_pushed = 0;
float temperature = 0.0f;
float humidity = 0.0f;
volatile uint8_t can_rx_flag = 0;
uint8_t uart_rx_byte;
uint16_t ble_connection_flag = 0;
uint8_t ble_local_name[13]; // Array to hold: [AD_TYPE][C][A][N][n][o][n][-][X][X][X][X]
uint8_t ble_mac_addr[6];    // Array to store the factory MAC we read from the module
volatile uint8_t ble_initialized = 0;
uint16_t cli_serv_handle = 0;
uint16_t cli_rx_char_handle = 0; // Web Browser writes TO this
uint16_t cli_tx_char_handle = 0; // STM32 notifies FROM this

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void cli_println(char *string);
void read_temp(void);
void ble_init(void);
void HCI_Event_CB(void *pckt);

HAL_StatusTypeDef can_set_mode(uint32_t mode);
HAL_StatusTypeDef can_send(uint32_t id, uint8_t *data, uint32_t len);

cli_status_t help_func(int argc, char **argv);
cli_status_t can_pwr_func(int argc, char **argv);
cli_status_t temp_func(int argc, char **argv);
cli_status_t can_test_func(int argc, char **argv);
cli_status_t can_monitor_func(int argc, char **argv);
cli_status_t ble_func(int argc, char **argv);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
cli_port_t current_cli_port = CLI_PORT_UART;
cli_port_t target_cli_port = CLI_PORT_UART;

cli_t cli;	//creates instance of the CLI function

//table of commands and respective functions
cmd_t cmd_tbl[6] =
{
{ .cmd = "help", .func = help_func },
{ .cmd = "can_power", .func = can_pwr_func },
{ .cmd = "temp", .func = temp_func },
{ .cmd = "can_test", .func = can_test_func },
{ .cmd = "can_monitor", .func = can_monitor_func },
{ .cmd = "ble", .func = ble_func } };

cli_status_t help_func(int argc, char **argv)
{
	cli.println("-- HAL CLI Commands -- \r\n");
	cli.println("help \r\n");
	cli.println("can_power 	[ 1 | 0 | -help ] \r\n");
	cli.println("can_test 	[ -start | -help ] \r\n");
	cli.println("can_monitor 	[ -start | -help ] \r\n");
	cli.println("temp 		[ -read | -help ] \r\n");
	cli.println("ble 		[ -init | -help ] \r\n");
	return CLI_OK;
}

cli_status_t can_pwr_func(int argc, char **argv)
{
	if (argc > 0)
	{
		if (strcmp(argv[1], "-help") == 0)
		{
			cli.println("-- CAN Power help menu --\r\n");
			cli.println("can_power 1	//Enable CAN isolated PSU \r\n");
			cli.println("can_power 0	//Disabled CAN isolated PSU \r\n");
		}
		else if (strcmp(argv[1], "1") == 0)
		{
			cli.println("CAN power on \r\n");
			HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_SET);
		}
		else if (strcmp(argv[1], "0") == 0)
		{
			cli.println("CAN power off \r\n");
			HAL_GPIO_WritePin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin, GPIO_PIN_RESET);
		}
		else
		{
			cli.println("can_power invalid argument \r\n");
			return CLI_E_INVALID_ARGS;
		}
	}
	return CLI_OK;
}

cli_status_t temp_func(int argc, char **argv)
{
	if (argc > 0)
	{
		if (strcmp(argv[1], "-help") == 0)
		{
			cli.println("-- Temperature Sensor help menu --\r\n");
			cli.println("temp -read	//Print Temperature (C) and Humidity (%) on CLI \r\n");
		}
		else if (strcmp(argv[1], "-read") == 0)
		{
			read_temp();
		}
		else
		{
			cli.println("temp invalid argument \r\n");
			return CLI_E_INVALID_ARGS;
		}
	}
	return CLI_OK;
}

cli_status_t can_test_func(int argc, char **argv)
{
	if (argc > 0)
	{
		if (strcmp(argv[1], "-help") == 0)
		{
			cli.println("-- CAN Test help menu --\r\n");
			cli.println("can_test -start	//Start external CAN loopback test \r\n");
			cli.println("		//DISCONNECT FROM BUS BEFORE RUNNING \r\n");
		}
		else if (strcmp(argv[1], "-start") == 0)
		{
			uint8_t testData[] =
			{ 0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x86, 0x77 };

			if (HAL_GPIO_ReadPin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin) == GPIO_PIN_RESET)
			{
				cli.println("Error: CAN Power is OFF. Run 'can_power 1' first.\r\n");
				return CLI_E_IO;
			}

			if (can_set_mode(FDCAN_MODE_EXTERNAL_LOOPBACK) != HAL_OK)
			{
				return CLI_E_IO;
			}

			if (can_send(0x123, testData, 8) != HAL_OK)
			{
				cli.println("CAN TX Failed\r\n");
				return CLI_E_IO;
			}
			cli.println("External CAN Loopback Frame Sent (ID 0x123)\r\n");
		}
		else
		{
			cli.println("can_test invalid argument \r\n");
			return CLI_E_INVALID_ARGS;
		}
	}

	return CLI_OK;
}

cli_status_t can_monitor_func(int argc, char **argv)
{

	if (argc > 0)
	{
		if (strcmp(argv[1], "-help") == 0)
		{
			cli.println("-- CAN Monitor help menu --\r\n");
			cli.println("can_monitor -start	//Start continuous CAN monitoring \r\n");
		}
		else if (strcmp(argv[1], "-start") == 0)
		{
			if (HAL_GPIO_ReadPin(CAN_PWR_EN_GPIO_Port, CAN_PWR_EN_Pin) == GPIO_PIN_RESET)
			{
				cli.println("Error: CAN Power is OFF. Run 'can_power 1' first.\r\n");
				return CLI_E_IO;
			}

			cli.println("Entering Bus Monitor Mode (Listen Only)...\r\n");

			if (can_set_mode(FDCAN_MODE_BUS_MONITORING) != HAL_OK)
			{
				return CLI_E_IO;
			}
		}
		else
		{
			cli.println("can_monitor invalid argument \r\n");
			return CLI_E_INVALID_ARGS;
		}
	}
	return CLI_OK;
}

cli_status_t ble_func(int argc, char **argv)
{

	if (argc > 0)
	{
		if (strcmp(argv[1], "-help") == 0)
		{
			cli.println("-- BLE help menu --\r\n");
			cli.println("ble -init	//Initialize BLUE-NRG-M0L Module \r\n");
		}
		else if (strcmp(argv[1], "-init") == 0)
		{

			cli.println("Initializing BLE...\r\n");
			ble_init();
		}
		else
		{
			cli.println("BLE invalid argument \r\n");
			return CLI_E_INVALID_ARGS;
		}
	}
	return CLI_OK;
}

void cli_println(char *string)
{
	if (current_cli_port == CLI_PORT_USB)
	{
		CDC_Transmit_FS((uint8_t*) string, strlen(string)); //transmit CLI messages on USB CDC interface
	}
	else if (current_cli_port == CLI_PORT_BLE)
	{
		uint16_t len = strlen(string);
		uint8_t *ptr = (uint8_t*) string;

		/* Slice the string into 20-byte BLE packets */
		while (len > 0)
		{
			uint8_t chunk_len = (len > 20) ? 20 : len;

			/* Attempt to send the chunk */
			tBleStatus status = aci_gatt_update_char_value(cli_serv_handle, cli_tx_char_handle, 0, chunk_len, ptr);

			uint8_t retry_count = 0;
			while (status != BLE_STATUS_SUCCESS && retry_count < 20) //retry max 20 times if not successful (e.g. tx buffer full)
			{
				HAL_Delay(5);
				status = aci_gatt_update_char_value(cli_serv_handle, cli_tx_char_handle, 0, chunk_len, ptr);
				retry_count++;
			}

			ptr += chunk_len;
			len -= chunk_len;

			if (len > 0)
			{
				HAL_Delay(5);
			}
		}

		HAL_Delay(10);
	}
	else
	{
		HAL_UART_Transmit(&huart4, (uint8_t*) string, strlen(string), HAL_MAX_DELAY); //transmit CLI messages on UART interface
	}
	HAL_Delay(1);
}

void cli_rx(char c)
{
	cli_put(&cli, c);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART4)
	{
		if (current_cli_port == CLI_PORT_UART)
		{
			cli_rx(uart_rx_byte);

			if (uart_rx_byte == '\r') // If a Carriage Return (CR) is received, echo both CR and LF
			{
				uint8_t crlf[] = "\r\n";
				HAL_UART_Transmit(&huart4, crlf, 2, 5);
			}
			else
			{
				HAL_UART_Transmit(&huart4, &uart_rx_byte, 1, 5); // Echo back to the terminal
			}
		}
		HAL_UART_Receive_IT(&huart4, &uart_rx_byte, 1); //Re-arm the interrupt
	}
}

void breathe_LED(void)
{
	if (led_direction == 1) //counting up
	{
		led_duty++;
	}
	else if (led_direction == 0) //counting down
	{
		led_duty--;
	}

	if (led_duty == 2000)
	{
		led_direction = 0;
	}
	else if (led_duty == 10)
	{
		led_direction = 1;
	}
	__HAL_TIM_SET_COMPARE(&htim4, GRN_LED, led_duty);
	__HAL_TIM_SET_COMPARE(&htim4, RED_LED, led_duty);
	__HAL_TIM_SET_COMPARE(&htim4, BLU_LED, led_duty);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == USR_BTN_Pin)
	{
		button_pushed = 1;
	}
}

void read_temp(void)
{
	HDC2021_TriggerMeasurement(&hi2c2);
	temperature = HDC2021_ReadTemperature(&hi2c2);
	humidity = HDC2021_ReadHumidity(&hi2c2);

	char msg[30];
	sprintf(msg, "T: %.2fC, H: %.2f%%\r\n", temperature, humidity);
	cli.println(msg);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		can_rx_flag = 1;
	}
}

HAL_StatusTypeDef can_set_mode(uint32_t mode)
{
	HAL_FDCAN_Stop(&hfdcan1);
	hfdcan1.State = HAL_FDCAN_STATE_RESET;
	hfdcan1.Init.Mode = mode;

	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1,
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef can_send(uint32_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	// Configure common header settings
	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8; // Simplistic DLC conversion
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
}

void ble_init(void)
{
	// Extract the STM32's unique factory silicon ID
	uint32_t uid_word0 = *(uint32_t*) (UID_BASE);
	uint32_t uid_word1 = *(uint32_t*) (UID_BASE + 0x04);

	// Format the global broadcast name using the UID */
	ble_local_name[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
	sprintf((char*) &ble_local_name[1], "CANnon-%04X", (unsigned int) (uid_word0 & 0xFFFF));

	// Generate a Unique Static MAC Address using the bits from the UID
	ble_mac_addr[0] = (uid_word0 >> 0) & 0xFF;
	ble_mac_addr[1] = (uid_word0 >> 8) & 0xFF;
	ble_mac_addr[2] = (uid_word0 >> 16) & 0xFF;
	ble_mac_addr[3] = (uid_word0 >> 24) & 0xFF;
	ble_mac_addr[4] = (uid_word1 >> 0) & 0xFF;
	ble_mac_addr[5] = 0xC0; // Top 2 bits must be 11 (0xC0) for a valid Static Random MAC

	uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
	uint8_t hwVersion = 0;
	uint16_t fwVersion = 0;

	hci_init(HCI_Event_CB, NULL);
	hci_reset();
	HAL_Delay(100);

	// Write our dynamically generated unique MAC to the BlueNRG RAM (Overwriting the empty FF:FF:FF...)
	aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, 6, ble_mac_addr);

	aci_gatt_init();

	// Initialize GAP layer (Sets device as a Peripheral/Slave)
	aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

	// Set output power level (1=High Power)
	aci_hal_set_tx_power_level(1, 4);
	aci_gap_set_discoverable(ADV_IND, 0x0100, 0x0200, PUBLIC_ADDR, NO_WHITE_LIST_USE, 12, (const char*) ble_local_name, 0, NULL, 0, 0);

	/* 1. Define Nordic UART Service (NUS) 128-bit UUIDs (Little-Endian) */

	/* NUS Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
	uint8_t cli_serv_uuid[16] =
	{ 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e };

	/* NUS RX Char UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Browser -> Board) */
	uint8_t cli_rx_uuid[16] =
	{ 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e };

	/* NUS TX Char UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Board -> Browser) */
	uint8_t cli_tx_uuid[16] =
	{ 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e };

	/* 2. Add the Service to the BLE Stack */
	/* Max attributes = 7 (1 for service + 2 for each char + 2 for CCCD descriptors) */
	aci_gatt_add_serv(UUID_TYPE_128, cli_serv_uuid, PRIMARY_SERVICE, 7, &cli_serv_handle);

	/* 3. Add RX Characteristic (WRITE property, Max 20 bytes) */
	/* GATT_NOTIFY_ATTRIBUTE_WRITE tells the stack to fire an event when the PC writes to this! */
	aci_gatt_add_char(cli_serv_handle, UUID_TYPE_128, cli_rx_uuid, 20,
	CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
	ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &cli_rx_char_handle);

	/* 4. Add TX Characteristic (NOTIFY property, Max 20 bytes) */
	aci_gatt_add_char(cli_serv_handle, UUID_TYPE_128, cli_tx_uuid, 20,
	CHAR_PROP_NOTIFY,
	ATTR_PERMISSION_NONE, 0, 16, 1, &cli_tx_char_handle);

	if (getBlueNRGVersion(&hwVersion, &fwVersion) == BLE_UTIL_SUCCESS)
	{
		char msg[128];
		sprintf(msg, "\r\nBLE Init Success!\r\n - Name: %s\r\n - MAC:  %02X:%02X:%02X:%02X:%02X:%02X\r\n", &ble_local_name[1], ble_mac_addr[5], ble_mac_addr[4], ble_mac_addr[3],
				ble_mac_addr[2], ble_mac_addr[1], ble_mac_addr[0]);
		cli.println(msg);

		ble_initialized = 1;
	}
	else
	{
		cli.println("BLE Init Failed.\r\n");
	}
}

void HCI_Event_CB(void *pckt)
{
	hci_uart_pckt *hci_pckt = (hci_uart_pckt*) pckt;

	if (hci_pckt->type != HCI_EVENT_PKT)
	{
		return;
	}

	hci_event_pckt *event_pckt = (hci_event_pckt*) hci_pckt->data;

	switch (event_pckt->evt)
	{
	/* ------------------------------------------- */
	/* DISCONNECTION EVENT                         */
	/* ------------------------------------------- */
	case EVT_DISCONN_COMPLETE:
	{
		evt_disconn_complete *evt = (void*) event_pckt->data;

		char msg[64];
		sprintf(msg, "BLE Disconnected! Reason: 0x%02X\r\n", evt->reason);
		cli.println(msg);

		ble_connection_flag = 0;
		cli.println("Restarting BLE Advertising...\r\n");

		/* Restart the beacon */
		aci_gap_set_discoverable(ADV_IND, 0x0100, 0x0200, PUBLIC_ADDR, NO_WHITE_LIST_USE, 12, (const char*) ble_local_name, 0, NULL, 0, 0);
	}
		break;

		/* ------------------------------------------- */
		/* LE META EVENTS (Connections, etc.)          */
		/* ------------------------------------------- */
	case EVT_LE_META_EVENT:
	{
		evt_le_meta_event *evt = (void*) event_pckt->data;

		if (evt->subevent == EVT_LE_CONN_COMPLETE)
		{
			evt_le_connection_complete *cc = (void*) evt->data;
			ble_connection_flag = cc->handle;

			char msg[64];
			sprintf(msg, "BLE Connected! Handle: 0x%04X\r\n", ble_connection_flag);
			cli.println(msg);
		}
	}
		break;

		/* ------------------------------------------- */
		/* VENDOR EVENTS (Incoming GATT Data, etc.)    */
		/* ------------------------------------------- */
	case EVT_VENDOR:
	{
		evt_blue_aci *blue_evt = (evt_blue_aci*) event_pckt->data;

		if (blue_evt->ecode == EVT_BLUE_GATT_ATTRIBUTE_MODIFIED)
		{
			evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*) blue_evt->data;

			if (evt->attr_handle == (cli_rx_char_handle + 1))
			{
				/* Process each received byte */
				if (current_cli_port == CLI_PORT_BLE)
				{
					for (uint8_t i = 0; i < evt->data_length; i++)
					{
						char c = evt->att_data[i];

						cli_rx(c);

						if (c == '\r')
						{
							cli_println("\r\n");
						}
						else
						{
							char echo_str[2] =
							{ c, '\0' };
							cli_println(echo_str);
						}
					}
				}
			}
		}
	}
		break;
	}
}

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
	MX_DMA_Init();
	MX_FDCAN1_Init();
	MX_I2C2_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_USART4_UART_Init();
	MX_USB_Device_Init();
	MX_IWDG_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart4, &uart_rx_byte, 1);
	HDC2021_Init(&hi2c2, HDC2021_RESOLUTION_14BIT, HDC2021_RESOLUTION_14BIT, HDC2021_RATE_OFF);

	cli.println = cli_println;	//define function used for cli.println
	cli.cmd_tbl = cmd_tbl;			//define name of array used for cmd_tbl
	cli.cmd_cnt = sizeof(cmd_tbl) / sizeof(cmd_t);	//define number of commands
	cli_init(&cli);

	HAL_TIM_PWM_Start(&htim4, GRN_LED);

	ble_init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg);
		cli_process(&cli);	//periodically call to process incoming characters

		if (ble_initialized == 1)
		{
			hci_user_evt_proc();
		}

		if (cdc_connection_open_flag == 1)
		{
			target_cli_port = CLI_PORT_USB;
		}
		else if (ble_connection_flag != 0)
		{
			target_cli_port = CLI_PORT_BLE;
		}
		else
		{
			target_cli_port = CLI_PORT_UART;
		}

		if (current_cli_port != target_cli_port)
		{

			if (target_cli_port == CLI_PORT_USB)
			{
				cli.println("\r\n[CLI Disconnected - USB Connection Overriding]\r\n");
			}
			else if (target_cli_port == CLI_PORT_BLE)
			{
				cli.println("\r\n[CLI Disconnected - BLE Connection Overriding]\r\n");
			}
			else
			{
				cli.println("\r\n[CLI Disconnected - Reverting to UART]\r\n");
			}

			current_cli_port = target_cli_port;

			cli_init(&cli);

			if (target_cli_port == CLI_PORT_USB)
			{
				HAL_TIM_PWM_Start(&htim4, RED_LED);
				HAL_TIM_PWM_Stop(&htim4, GRN_LED);
				HAL_TIM_PWM_Stop(&htim4, BLU_LED);
			}
			else if (target_cli_port == CLI_PORT_BLE)
			{
				HAL_TIM_PWM_Stop(&htim4, RED_LED);
				HAL_TIM_PWM_Stop(&htim4, GRN_LED);
				HAL_TIM_PWM_Start(&htim4, BLU_LED);
			}
			else
			{
				HAL_TIM_PWM_Stop(&htim4, RED_LED);
				HAL_TIM_PWM_Start(&htim4, GRN_LED);
				HAL_TIM_PWM_Stop(&htim4, BLU_LED);
			}
		}

		if (button_pushed == 1)
		{
			read_temp();
			button_pushed = 0;
		}

		if (can_rx_flag)
		{
			FDCAN_RxHeaderTypeDef RxHeader;
			uint8_t RxData[8];
			char msg[128];

			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			{
				int len = sprintf(msg, "RX ID: 0x%03lX [%ld] Data: ", RxHeader.Identifier, RxHeader.DataLength >> 16);

				for (int i = 0; i < 8; i++)
				{
					len += sprintf(msg + len, "%02X ", RxData[i]);
				}
				strcat(msg, "\r\n");
				cli.println(msg);
			}
			else
			{
				cli.println("RX Message Error\r\n");
			}

			can_rx_flag = 0;
		}
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{

	/* USER CODE BEGIN FDCAN1_Init 0 */

	/* USER CODE END FDCAN1_Init 0 */

	/* USER CODE BEGIN FDCAN1_Init 1 */

	/* USER CODE END FDCAN1_Init 1 */
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_BUS_MONITORING;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 2;
	hfdcan1.Init.NominalSyncJumpWidth = 1;
	hfdcan1.Init.NominalTimeSeg1 = 13;
	hfdcan1.Init.NominalTimeSeg2 = 1;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 1;
	hfdcan1.Init.DataTimeSeg1 = 1;
	hfdcan1.Init.DataTimeSeg2 = 1;
	hfdcan1.Init.StdFiltersNbr = 0;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN1_Init 2 */

	/* USER CODE END FDCAN1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00503D58;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 3;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 9;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 5;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 3;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 7999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART4_UART_Init(void)
{

	/* USER CODE BEGIN USART4_Init 0 */

	/* USER CODE END USART4_Init 0 */

	/* USER CODE BEGIN USART4_Init 1 */

	/* USER CODE END USART4_Init 1 */
	huart4.Instance = USART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART4_Init 2 */

	/* USER CODE END USART4_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, BLE_RESET_Pin | BLE_SPI_CS_Pin | CAN_PWR_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : BLE_RESET_Pin BLE_SPI_CS_Pin CAN_PWR_EN_Pin */
	GPIO_InitStruct.Pin = BLE_RESET_Pin | BLE_SPI_CS_Pin | CAN_PWR_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BLE_SPI_IRQ_Pin */
	GPIO_InitStruct.Pin = BLE_SPI_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BLE_SPI_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USR_BTN_Pin */
	GPIO_InitStruct.Pin = USR_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USR_BTN_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
