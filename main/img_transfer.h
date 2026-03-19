#ifndef IMG_TRANSFER_H
#define IMG_TRANSFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize image transfer subsystem
 */
void img_transfer_init(void);

/**
 * @brief Process image transfer related commands (ID 0x50)
 * 
 * @param data Received packet data
 * @param len Packet length
 */
void process_img_command(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // IMG_TRANSFER_H
