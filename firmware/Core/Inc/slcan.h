/**
 ******************************************************************************
 * @file           : slcan.h
 * @brief          : Header for Lawicel/SLCAN protocol parser
 ******************************************************************************
 */
#ifndef __SLCAN_H
#define __SLCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* --- Public Functions --- */

/**
 * @brief Feeds a single character from the USB CDC interface into the SLCAN state machine.
 * @param c The character received.
 */
void slcan_rx(char c);

/**
 * @brief Formats an incoming CAN frame into an SLCAN ASCII string and transmits it over USB.
 * @param id The standard 11-bit CAN ID.
 * @param data Pointer to the payload.
 * @param len The length of the payload in bytes (0-8).
 */
void slcan_format_rx_frame(uint32_t id, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SLCAN_H */