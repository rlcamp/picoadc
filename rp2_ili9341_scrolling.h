#include <stdint.h>

void ili9341_scrolling_init(void);
void ili9341_write_row_and_scroll(const uint8_t bins[restrict static 240]);
