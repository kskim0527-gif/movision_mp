#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#include <stdint.h>
#include <stddef.h>

void fw_update_init(void);
void process_fw_update_command(const uint8_t *data, size_t len);

#endif // FW_UPDATE_H
