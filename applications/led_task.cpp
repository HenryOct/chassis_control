#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

constexpr uint16_t RGB_FLOW_COLOR_CHANGE_TIME = 10;
constexpr uint8_t RGB_FLOW_COLOR_LENGTH = 6;

uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGTH + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

// LED任务，实现RGB流水灯效果
extern "C" void led_task()
{
	led.start();

	while (1) {
		for (uint16_t i = 0; i < RGB_FLOW_COLOR_LENGTH; i++) {
			uint32_t start_color = RGB_flow_color[i];
			uint32_t end_color = RGB_flow_color[i + 1];

			int32_t start_alpha = (start_color & 0xFF000000) >> 24;
			int32_t start_red = (start_color & 0x00FF0000) >> 16;
			int32_t start_green = (start_color & 0x0000FF00) >> 8;
			int32_t start_blue = start_color & 0x000000FF;

			int32_t delta_alpha = ((end_color & 0xFF000000) >> 24) - start_alpha;
			int32_t delta_red = ((end_color & 0x00FF0000) >> 16) - start_red;
			int32_t delta_green = ((end_color & 0x0000FF00) >> 8) - start_green;
			int32_t delta_blue = (end_color & 0x000000FF) - start_blue;

			for (uint16_t j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++) {
				uint32_t alpha = start_alpha + (delta_alpha * j / RGB_FLOW_COLOR_CHANGE_TIME);
				uint32_t red = alpha * (start_red + (delta_red * j / RGB_FLOW_COLOR_CHANGE_TIME));
				uint32_t green = alpha * (start_green + (delta_green * j / RGB_FLOW_COLOR_CHANGE_TIME));
				uint32_t blue = alpha * (start_blue + (delta_blue * j / RGB_FLOW_COLOR_CHANGE_TIME));

				led.set(red, green, blue);
				osDelay(100);
			}
		}
	}
}
