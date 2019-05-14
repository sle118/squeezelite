#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#define CODEC_INCLUDE "flc,pcm,mp3"
#define VS1053_CS 5   // D1 // 5
#define VS1053_DCS 16 // D0 // 16
#define VS1053_DREQ 4 // D3 // 4



#include <driver/spi_common.h>
#include <driver/spi_master.h>
//#include <POSIX_VS1053.h>

void init_VS1053();
void LoadPlugin(const uint16_t *d, uint16_t len);
