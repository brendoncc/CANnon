/**
 ******************************************************************************
 * @file           : slcan.c
 * @brief          : Implementation of the Lawicel/SLCAN protocol parser
 ******************************************************************************
 */

#include "slcan.h"
#include "usbd_cdc_if.h"
#include "stm32g0xx_hal.h"
#include <stdio.h>
#include <string.h>

/* --- External Dependencies from main.c --- */
extern HAL_StatusTypeDef can_send(uint32_t id, uint8_t *data, uint32_t len);
extern HAL_StatusTypeDef can_set_mode(uint32_t mode);

/* --- Private Defines --- */
#define SLCAN_BUF_SIZE 32
#define SLCAN_ACK      "\r"
#define SLCAN_NACK     "\a"

/* --- Private Variables --- */
static char rx_buf[SLCAN_BUF_SIZE];
static uint8_t rx_idx = 0;

/* --- Private Helper Functions --- */

/* Transmits a string back over the USB CDC interface */
static void slcan_transmit(const char *str)
{
	CDC_Transmit_FS((uint8_t*) str, strlen(str));
}

/* Converts an ASCII hex character to an integer (e.g., 'A' -> 10) */
static uint8_t hex_to_int(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return 0;
}

/* Parses the buffered SLCAN command and executes the corresponding action */
static void slcan_process_command(const char *cmd)
{
	char type = cmd[0];

	switch (type)
	{
	/* ------------------------------------------------ */
	/* 'O': Open CAN channel                            */
	/* ------------------------------------------------ */
	case 'O':
		if (can_set_mode(FDCAN_MODE_NORMAL) == HAL_OK)
		{
			slcan_transmit(SLCAN_ACK);
		}
		else
		{
			slcan_transmit(SLCAN_NACK);
		}
		break;

		/* ------------------------------------------------ */
		/* 'C': Close CAN channel                           */
		/* ------------------------------------------------ */
	case 'C':
		/* For now, we will just acknowledge. In a full implementation,
		 you might transition back to a low-power or standby mode. */
		slcan_transmit(SLCAN_ACK);
		break;

		/* ------------------------------------------------ */
		/* 'S': Set Baudrate (S0 to S8)                     */
		/* ------------------------------------------------ */
	case 'S':
		/* STM32CubeMX FDCAN clock generation is statically compiled.
		 We simply acknowledge the PC software's baudrate request
		 so it doesn't crash, but rely on our hardcoded FDCAN timings. */
		slcan_transmit(SLCAN_ACK);
		break;

		/* ------------------------------------------------ */
		/* 't': Transmit Standard Frame (tIIILDD...)        */
		/* ------------------------------------------------ */
	case 't':
	{
		if (strlen(cmd) < 5)
		{
			slcan_transmit(SLCAN_NACK);
			break;
		}

		/* 1. Parse 11-bit ID (3 hex chars) */
		uint32_t id = (hex_to_int(cmd[1]) << 8) | (hex_to_int(cmd[2]) << 4) | hex_to_int(cmd[3]);

		/* 2. Parse DLC (Data Length Code, 1 char) */
		uint8_t dlc = hex_to_int(cmd[4]);
		if (dlc > 8)
			dlc = 8; /* Max 8 bytes for classic CAN */

		/* 3. Parse Data Bytes */
		uint8_t data[8] =
		{ 0 };
		for (uint8_t i = 0; i < dlc; i++)
		{
			/* Data bytes start at index 5. Two hex chars per byte. */
			data[i] = (hex_to_int(cmd[5 + (i * 2)]) << 4) | hex_to_int(cmd[6 + (i * 2)]);
		}

		/* 4. Send to physical bus */
		if (can_send(id, data, dlc) == HAL_OK)
		{
			slcan_transmit(SLCAN_ACK); /* Standard Lawicel success */
		}
		else
		{
			slcan_transmit(SLCAN_NACK);
		}
		break;
	}

		/* ------------------------------------------------ */
		/* 'V': Hardware / Firmware Version                 */
		/* ------------------------------------------------ */
	case 'V':
		/* Software expects "V[HW_VER][SW_VER]\r" */
		slcan_transmit("V0101\r");
		break;

	case 'v':
		/* Software expects "v[SW_VER]\r" */
		slcan_transmit("v0101\r");
		break;

		/* ------------------------------------------------ */
		/* 'N': Serial Number                               */
		/* ------------------------------------------------ */
	case 'N':
		/* Software expects "N[4_ALPHANUMERIC]\r" */
		slcan_transmit("N0001\r");
		break;

		/* Unknown or Unsupported Command */
	default:
		slcan_transmit(SLCAN_NACK);
		break;
	}
}

/* --- Public Functions --- */

void slcan_rx(char c)
{
	/* Lawicel commands are terminated by a Carriage Return (\r) */
	if (c == '\r')
	{
		rx_buf[rx_idx] = '\0'; // Null-terminate the string
		if (rx_idx > 0)
		{
			slcan_process_command(rx_buf);
		}
		rx_idx = 0; // Reset buffer for next command
	}
	/* Ignore Line Feeds (\n) or prevent buffer overflows */
	else if (c != '\n' && rx_idx < (SLCAN_BUF_SIZE - 1))
	{
		rx_buf[rx_idx++] = c;
	}
}

void slcan_format_rx_frame(uint32_t id, uint8_t *data, uint8_t len)
{
	char buf[32];

	/* Format header: 't' + 3-char ID + 1-char length */
	int pos = sprintf(buf, "t%03X%d", (unsigned int) id, len);

	/* Append hex data bytes */
	for (int i = 0; i < len; i++)
	{
		pos += sprintf(buf + pos, "%02X", data[i]);
	}

	/* Terminate with CR */
	buf[pos++] = '\r';

	/* Send directly over USB CDC */
	CDC_Transmit_FS((uint8_t*) buf, pos);
}
