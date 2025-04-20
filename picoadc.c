#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "RP2350.h"

#include <tusb.h>

/* as a diagnostic we will output a 2 kHz full scale square wave on this pin */
#define PWM_PIN 22

/* adc input pin number is 26 plus this value */
#define ADC_CHANNEL 1

/* we will use dma channels 0 and 1 */
#define IDMA_ADC_BASE 0

/* total number of bytes of sram used for ring buffer */
#define RING_BUFFER_SIZE 8192

/* number of chunks to use, must be a power of two, must be at least 2 for minimum
 functionality, must be at least 3 for 50% overlapped fft functionality */
#define CHUNKS 4
#define SAMPLES_PER_CHUNK (RING_BUFFER_SIZE / (CHUNKS * sizeof(int16_t)))

#define SAMPLE_RATE_NUMERATOR 48000000ULL
unsigned long sample_rate_denominator = 1500;

void yield(void) {
    /* we could do context switching here for cooperative multitasking if we wanted */
    __dsb();
    __wfe();
}

void usb_out_chars(const unsigned char * buf, size_t length) {
    while (length) {
        const size_t available = tud_cdc_write_available();
        const size_t write_now = length < available ? length : available;
        if (write_now) {
            const size_t written_now = tud_cdc_write(buf, write_now);
            length -= written_now;
            buf += written_now;
        }
        tud_task();
        tud_cdc_write_flush();
    }
}

size_t ichunk_written = 0;
static int16_t adc_chunks[4][SAMPLES_PER_CHUNK];

/* this function gets called once per chunk, i.e. four times per full ring buffer */
void __scratch_y("") adc_dma_irq_handler(void) {
    const unsigned idma_channel = IDMA_ADC_BASE + (ichunk_written % 2);
    dma_channel_hw_t * dma_channel = &dma_hw->ch[idma_channel];
    dma_hw->intr = 1U << idma_channel;

    ichunk_written++;

    /* reconfigure the channel that just finished */
    dma_channel->write_addr = (uintptr_t)&adc_chunks[(ichunk_written + 2) % CHUNKS];
    dma_channel->transfer_count = SAMPLES_PER_CHUNK;
    __DSB();
}

static void init_test_signal(void) {
    /* this will emit a full scale 2 kHz square wave on a pin (numerator is 48 MHz) */
    const unsigned square_wave_frequency_denominator = 24000;

    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    const unsigned slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice_num, square_wave_frequency_denominator - 1);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, square_wave_frequency_denominator / 2);
    pwm_set_enabled(slice_num, true);
}

static void adc_dma_init(void) {
    adc_init();
    adc_select_input(ADC_CHANNEL);
    adc_gpio_init(ADC_CHANNEL + 26);

    adc_fifo_setup(true, /* write each conversion to fifo */
                   true, /* enable dma request when fifo contains data */
                   1, /* dreq when one sample present */
                   false, /* disable err bit */
                   false /* no byte shift */ );

    adc_set_clkdiv(sample_rate_denominator - 1);

    for (size_t iconfig = 0; iconfig < 2; iconfig++) {
        dma_channel_config cfg = dma_channel_get_default_config(IDMA_ADC_BASE + iconfig);
        channel_config_set_chain_to(&cfg, IDMA_ADC_BASE + !iconfig);
        channel_config_set_dreq(&cfg, DREQ_ADC);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);

        dma_channel_configure(IDMA_ADC_BASE + iconfig,
                              &cfg,
                              &adc_chunks[iconfig],
                              &adc_hw->fifo,
                              SAMPLES_PER_CHUNK,
                              false);
    }

    dma_hw->ints0 |= (1u << (IDMA_ADC_BASE + 0)) | (1u << (IDMA_ADC_BASE + 1));
    dma_hw->inte0 |= (1u << (IDMA_ADC_BASE + 0)) | (1u << (IDMA_ADC_BASE + 1));
    __DSB();
    irq_set_exclusive_handler(DMA_IRQ_0, adc_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(IDMA_ADC_BASE + 0);
    adc_run(true);
}

int main(void) {
    set_sys_clock_48mhz();

    init_test_signal();

    tusb_init();

    /* this will block until the usb device is actually opened on the host end */
    while (!tud_cdc_connected()) tud_task();

    /* wtf */
    for (uint32_t t0 = time_us_32(); time_us_32() - t0 < 50000; ) tud_task();

    adc_dma_init();

    size_t ichunk_read = 0;

    /* inner loop over chunks of data */
    while (tud_cdc_connected()) {
        /* wait until there is a new chunk of data */
        while (ichunk_read == *(volatile size_t *)&ichunk_written) yield();

        const int16_t * restrict chunk = adc_chunks[ichunk_read % CHUNKS];
        ichunk_read++;

        usb_out_chars((void *)chunk, sizeof(int16_t) * SAMPLES_PER_CHUNK);
        tud_task();
    }

    /* this should be sufficient but it isn't */
    while (tud_cdc_write_flush()) tud_task();

    /* annoyingly we need to wait here before resetting */
    for (uint32_t t0 = time_us_32(); time_us_32() - t0 < 50000; ) tud_task();

    NVIC_SystemReset();
}
