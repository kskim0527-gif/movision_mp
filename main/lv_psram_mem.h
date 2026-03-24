#ifndef LV_PSRAM_MEM_H
#define LV_PSRAM_MEM_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *lvgl_psram_malloc(size_t size);
void lvgl_psram_free(void *ptr);
void *lvgl_psram_realloc(void *ptr, size_t new_size);

#ifdef __cplusplus
}
#endif

#endif // LV_PSRAM_MEM_H
