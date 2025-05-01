#include "rp2_ili9341_scrolling.h"

#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#define ILI9341_SWRESET 0x01
#define ILI9341_SLPOUT 0x11

#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPON 0x29

#define ILI9341_CASET 0x2A
#define ILI9341_PASET 0x2B
#define ILI9341_RAMWR 0x2C

#define ILI9341_VSCRDEF 0x33
#define ILI9341_MADCTL 0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT 0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1 0xC0
#define ILI9341_PWCTR2 0xC1
#define ILI9341_VMCTR1 0xC5
#define ILI9341_VMCTR2 0xC7

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1

static void dc_pin(unsigned value) {
    gpio_put(16, value);
}

static void cs_pin(unsigned value) {
    gpio_put(17, value);
}

static void spi_write_bytes(const size_t len, const void * bytes) {
    spi_write_blocking(spi0, (const uint8_t *)bytes, len);
}

static void send_command_and_bytes_selected(uint8_t cmd, const size_t len, const void * bytes) {
    dc_pin(0);
    spi_write_bytes(1, &cmd);
    dc_pin(1);

    spi_write_bytes(len, bytes);
}

static void send_command_and_bytes(uint8_t cmd, const size_t len, const void * bytes) {
    cs_pin(0);
    send_command_and_bytes_selected(cmd, len, bytes);
    cs_pin(1);
}

/* turbo is copyright google llc, apache 2.0 license */
static const unsigned char colormap[256][3] = {
    {48, 18, 59}, {50, 21, 67}, {51, 24, 74}, {52, 27, 81}, {53, 30, 88}, {54, 33, 95},
    {55, 36, 102}, {56, 39, 109}, {57, 42, 115}, {58, 45, 121}, {59, 47, 128}, {60, 50, 134},
    {61, 53, 139}, {62, 56, 145}, {63, 59, 151}, {63, 62, 156}, {64, 64, 162}, {65, 67, 167},
    {65, 70, 172}, {66, 73, 177}, {66, 75, 181}, {67, 78, 186}, {68, 81, 191}, {68, 84, 195},
    {68, 86, 199}, {69, 89, 203}, {69, 92, 207}, {69, 94, 211}, {70, 97, 214}, {70, 100, 218},
    {70, 102, 221}, {70, 105, 224}, {70, 107, 227}, {71, 110, 230}, {71, 113, 233}, {71, 115, 235},
    {71, 118, 238}, {71, 120, 240}, {71, 123, 242}, {70, 125, 244}, {70, 128, 246}, {70, 130, 248},
    {70, 133, 250}, {70, 135, 251}, {69, 138, 252}, {69, 140, 253}, {68, 143, 254}, {67, 145, 254},
    {66, 148, 255}, {65, 150, 255}, {64, 153, 255}, {62, 155, 254}, {61, 158, 254}, {59, 160, 253},
    {58, 163, 252}, {56, 165, 251}, {55, 168, 250}, {53, 171, 248}, {51, 173, 247}, {49, 175, 245},
    {47, 178, 244}, {46, 180, 242}, {44, 183, 240}, {42, 185, 238}, {40, 188, 235}, {39, 190, 233},
    {37, 192, 231}, {35, 195, 228}, {34, 197, 226}, {32, 199, 223}, {31, 201, 221}, {30, 203, 218},
    {28, 205, 216}, {27, 208, 213}, {26, 210, 210}, {26, 212, 208}, {25, 213, 205}, {24, 215, 202},
    {24, 217, 200}, {24, 219, 197}, {24, 221, 194}, {24, 222, 192}, {24, 224, 189}, {25, 226, 187},
    {25, 227, 185}, {26, 228, 182}, {28, 230, 180}, {29, 231, 178}, {31, 233, 175}, {32, 234, 172},
    {34, 235, 170}, {37, 236, 167}, {39, 238, 164}, {42, 239, 161}, {44, 240, 158}, {47, 241, 155},
    {50, 242, 152}, {53, 243, 148}, {56, 244, 145}, {60, 245, 142}, {63, 246, 138}, {67, 247, 135},
    {70, 248, 132}, {74, 248, 128}, {78, 249, 125}, {82, 250, 122}, {85, 250, 118}, {89, 251, 115},
    {93, 252, 111}, {97, 252, 108}, {101, 253, 105}, {105, 253, 102}, {109, 254, 98}, {113, 254, 95},
    {117, 254, 92}, {121, 254, 89}, {125, 255, 86}, {128, 255, 83}, {132, 255, 81}, {136, 255, 78},
    {139, 255, 75}, {143, 255, 73}, {146, 255, 71}, {150, 254, 68}, {153, 254, 66}, {156, 254, 64},
    {159, 253, 63}, {161, 253, 61}, {164, 252, 60}, {167, 252, 58}, {169, 251, 57}, {172, 251, 56},
    {175, 250, 55}, {177, 249, 54}, {180, 248, 54}, {183, 247, 53}, {185, 246, 53}, {188, 245, 52},
    {190, 244, 52}, {193, 243, 52}, {195, 241, 52}, {198, 240, 52}, {200, 239, 52}, {203, 237, 52},
    {205, 236, 52}, {208, 234, 52}, {210, 233, 53}, {212, 231, 53}, {215, 229, 53}, {217, 228, 54},
    {219, 226, 54}, {221, 224, 55}, {223, 223, 55}, {225, 221, 55}, {227, 219, 56}, {229, 217, 56},
    {231, 215, 57}, {233, 213, 57}, {235, 211, 57}, {236, 209, 58}, {238, 207, 58}, {239, 205, 58},
    {241, 203, 58}, {242, 201, 58}, {244, 199, 58}, {245, 197, 58}, {246, 195, 58}, {247, 193, 58},
    {248, 190, 57}, {249, 188, 57}, {250, 186, 57}, {251, 184, 56}, {251, 182, 55}, {252, 179, 54},
    {252, 177, 54}, {253, 174, 53}, {253, 172, 52}, {254, 169, 51}, {254, 167, 50}, {254, 164, 49},
    {254, 161, 48}, {254, 158, 47}, {254, 155, 45}, {254, 153, 44}, {254, 150, 43}, {254, 147, 42},
    {254, 144, 41}, {253, 141, 39}, {253, 138, 38}, {252, 135, 37}, {252, 132, 35}, {251, 129, 34},
    {251, 126, 33}, {250, 123, 31}, {249, 120, 30}, {249, 117, 29}, {248, 114, 28}, {247, 111, 26},
    {246, 108, 25}, {245, 105, 24}, {244, 102, 23}, {243, 99, 21}, {242, 96, 20}, {241, 93, 19},
    {240, 91, 18}, {239, 88, 17}, {237, 85, 16}, {236, 83, 15}, {235, 80, 14}, {234, 78, 13},
    {232, 75, 12}, {231, 73, 12}, {229, 71, 11}, {228, 69, 10}, {226, 67, 10}, {225, 65, 9},
    {223, 63, 8}, {221, 61, 8}, {220, 59, 7}, {218, 57, 7}, {216, 55, 6}, {214, 53, 6},
    {212, 51, 5}, {210, 49, 5}, {208, 47, 5}, {206, 45, 4}, {204, 43, 4}, {202, 42, 4},
    {200, 40, 3}, {197, 38, 3}, {195, 37, 3}, {193, 35, 2}, {190, 33, 2}, {188, 32, 2},
    {185, 30, 2}, {183, 29, 2}, {180, 27, 1}, {178, 26, 1}, {175, 24, 1}, {172, 23, 1},
    {169, 22, 1}, {167, 20, 1}, {164, 19, 1}, {161, 18, 1}, {158, 16, 1}, {155, 15, 1},
    {152, 14, 1}, {149, 13, 1}, {146, 11, 1}, {142, 10, 1}, {139, 9, 2}, {136, 8, 2},
    {133, 7, 2}, {129, 6, 2}, {126, 5, 2}, {122, 4, 3} };

static uint16_t colormap_rgb565_swapped[256];

static uint16_t rgb565(const uint8_t r, const uint8_t g, const uint8_t b) {
    return ((r & 0b11111000) << 8U) | ((g & 0b11111100) << 3U) | (b >> 3U);
}

#define IDMA_SPI_WRITE 1

static uint16_t mapped[240];

void __scratch_y("") spi_dma_write_finish_handler(void) {
    /* acknowledge the interrupt so that it doesn't re-fire */
    dma_hw->ints1 = 1U << IDMA_SPI_WRITE;

    cs_pin(1);
}

void ili9341_scrolling_init(void) {
    for (size_t ic = 0; ic < 256; ic++)
        colormap_rgb565_swapped[ic] = __builtin_bswap16(rgb565(colormap[ic][0], colormap[ic][1], colormap[ic][2]));

    /* wait until power has been on for a bit */
    sleep_ms(150);

    spi_init(spi0, 8000000);
    /* use pins 16, 17, 18, 19 for DC, CS, SCK, MOSI espectively */
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);

    gpio_init(16);
    gpio_set_dir(16, GPIO_OUT);
    gpio_init(17);
    gpio_set_dir(17, GPIO_OUT);
    dc_pin(1);
    cs_pin(1);

    send_command_and_bytes(ILI9341_SWRESET, 0, NULL);

    /* TODO: factor out blocking delays? */
    sleep_ms(150);

    /* "We got that from some ILITEK example code. We don't know what it means." - adafruit */
    send_command_and_bytes(0xCF, 3, (const uint8_t[]) { 0x00, 0xC1, 0x30 }); /* power control B */
    send_command_and_bytes(0xED, 4, (const uint8_t[]) { 0x64, 0x03, 0x12, 0x81 }); /* power on sequence control */
    send_command_and_bytes(0xE8, 3, (const uint8_t[]) { 0x85, 0x00, 0x78 }); /* driver timing control A */
    send_command_and_bytes(0xCB, 5, (const uint8_t[]) { 0x39, 0x2C, 0x00, 0x34, 0x0 }); /* power control A */
    send_command_and_bytes(0xF7, 1, (const uint8_t[]) { 0x20 }); /* pump ratio control */
    send_command_and_bytes(0xEA, 2, (const uint8_t[]) { 0x00, 0x00 }); /* driver timing control B */
    send_command_and_bytes(ILI9341_PWCTR1, 1, (const uint8_t[]) { 1, 0x23 });
    send_command_and_bytes(ILI9341_PWCTR2, 1, (const uint8_t[]) { 1, 0x10 });
    send_command_and_bytes(ILI9341_VMCTR1, 2, (const uint8_t[]) { 0x3e, 0x28 });
    send_command_and_bytes(ILI9341_VMCTR2, 1, (const uint8_t[]) { 0x86 });
    send_command_and_bytes(ILI9341_MADCTL, 1, (const uint8_t[]) { 0x48 });
    send_command_and_bytes(ILI9341_VSCRSADD, 1, (const uint8_t[]) { 0x00 });
    send_command_and_bytes(ILI9341_PIXFMT, 1, (const uint8_t[]) { 0x55 });
    send_command_and_bytes(ILI9341_FRMCTR1, 2, (const uint8_t[]) { 0x00, 0x18 });
    send_command_and_bytes(ILI9341_DFUNCTR, 3, (const uint8_t[]) { 0x08, 0x82, 0x27 });
    send_command_and_bytes(0xF2, 1, (const uint8_t[]) { 0x00 }); /* disable 3g */
    send_command_and_bytes(ILI9341_GAMMASET, 1, (const uint8_t[]) { 0x01 });
    send_command_and_bytes(ILI9341_GMCTRP1, 15, (const uint8_t[]) {
        0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00
    });
    send_command_and_bytes(ILI9341_GMCTRN1, 15, (const uint8_t[]) {
        0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F
    });
    send_command_and_bytes(ILI9341_SLPOUT, 0, NULL); /* exit sleep */
    sleep_ms(150);
    send_command_and_bytes(ILI9341_DISPON, 0, NULL); /* display on */
    sleep_ms(150);

    /* blank display */
    cs_pin(0);
    send_command_and_bytes_selected(ILI9341_CASET, 4, (const uint16_t[]) { __builtin_bswap16(0), __builtin_bswap16(239) });
    send_command_and_bytes_selected(ILI9341_PASET, 4, (const uint16_t[]) { __builtin_bswap16(0), __builtin_bswap16(319) });
    send_command_and_bytes_selected(ILI9341_RAMWR, 0, NULL);

    const uint16_t black = __builtin_bswap16(rgb565(0, 0, 0));
    for (size_t iy = 0; iy < 320; iy++)
        for (size_t ix = 0; ix < 240; ix++)
            spi_write_bytes(2, &black);
    cs_pin(1);

    cs_pin(0);
    send_command_and_bytes_selected(ILI9341_VSCRDEF, 6, (const uint16_t[]) {
        __builtin_bswap16(0), __builtin_bswap16(320), __builtin_bswap16(0) });
    cs_pin(1);

    dma_channel_claim(IDMA_SPI_WRITE);

    dma_channel_acknowledge_irq1(IDMA_SPI_WRITE);
    dma_channel_set_irq1_enabled(IDMA_SPI_WRITE, true);

    irq_set_exclusive_handler(DMA_IRQ_1, spi_dma_write_finish_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

extern void yield(void);

void ili9341_write_row_and_scroll(const uint8_t bins[restrict static 240]) {
    static size_t iy_scroll = 0;

    /* if previous write is still sending, wait before disturbing the buffer */
    while (dma_channel_is_busy(IDMA_SPI_WRITE)) yield();

    /* map 0-256 input values to colormap pixel values */
    for (size_t ix = 0; ix < 240; ix++)
        mapped[ix] = colormap_rgb565_swapped[bins[ix]];

    cs_pin(0);
    send_command_and_bytes_selected(ILI9341_VSCRSADD, 2, (const uint16_t[]) { __builtin_bswap16(iy_scroll % 320) });
    send_command_and_bytes_selected(ILI9341_PASET, 4, (const uint16_t[]) { __builtin_bswap16((iy_scroll + 319) % 320), __builtin_bswap16((iy_scroll + 319) % 320) });
    send_command_and_bytes_selected(ILI9341_RAMWR, 0, NULL);

    dma_channel_config cfg = dma_channel_get_default_config(IDMA_SPI_WRITE);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_dreq(&cfg, spi_get_dreq(spi0, true));
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);

    dma_channel_configure(IDMA_SPI_WRITE, &cfg,
                          &spi_get_hw(spi0)->dr, mapped,
                          sizeof(mapped), true);

    iy_scroll = (iy_scroll + 1) % 320;
}
