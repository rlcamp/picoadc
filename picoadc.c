#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "RP2350.h"

#include "fft_anywhere.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

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
    asm volatile("dsb; wfe");
}

/* note this is NOT volatile because we want to access it as a regular variable from within
 ISR context where it is safe to do so, and do explicitly volatile reads of it otherwise */
size_t ichunk_written = 0;

/* the actual big ring buffer, divided into 4 chunks */
static int16_t adc_chunks[CHUNKS][SAMPLES_PER_CHUNK];

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

static float cmagsquaredf(const float complex a) {
    return crealf(a) * crealf(a) + cimagf(a) * cimagf(a);
}

static unsigned char hex_byte(unsigned char c) {
    return (c >= 10) ? ('A' + (c - 10)) : ('0' + c);
}

int main(void) {
    set_sys_clock_48mhz();

    init_test_signal();

    stdio_usb_init();
    while (!stdio_usb_connected()) yield();

    adc_dma_init();

    /* fft length */
    const size_t T = 2 * SAMPLES_PER_CHUNK;

    /* number of kept frequency bins */
    const size_t F = T / 2 + 1;

    const float fs = SAMPLE_RATE_NUMERATOR / (float)sample_rate_denominator;

    /* number of fft frames to average for each line of output pixels */
    const float dt_desired = 0.0f; /* seconds */
    const size_t fft_frames_per_average = fmaxf(1.0f, dt_desired * fs / (0.5f * T) + 0.5f);

    const float df = fs / T, dt = 0.5f * fft_frames_per_average * T / fs;

    /* plan an r2c fft */
    struct planned_real_fft * const plan = plan_real_fft_of_length(T);

    /* allocate a bunch of buffers */
    float * restrict const scratch_in = malloc(sizeof(float) * T);
    float complex * restrict const scratch_out = malloc(sizeof(float complex) * T / 2);
    float * restrict const window = malloc(sizeof(float) * (T / 2 + 1));
    float * restrict const spectrum_power = malloc(sizeof(float) * F);

    /* hex-encoded spectrum output, plus \r\n */
    char * restrict const out_line = malloc(2 * F + 2);
    out_line[2 * F + 0] = '\r';
    out_line[2 * F + 1] = '\n';

    /* hann window, exploiting symmetry, accounting for averaging over time, normalized
     such that a full scale real-valued sine wave will have a -3.0103 dB response */
    for (size_t it = 0; it < T / 2 + 1; it++)
        window[it] = (1.0f - cosf(2.0f * (float)M_PI * (float)it / T)) * (float)M_SQRT2 / (T * 32767.0f * sqrtf(fft_frames_per_average));

    size_t ichunk_read = 0;
    size_t iframe_averaged = 0;

    /* output will be on [-192, 0) dB relative to full scale, in 0.75 dB increments */
    const float out_scale = 0.75f;
    const float out_offset = -256.0f * out_scale;
    const float one_over_out_scale = 1.0f / out_scale;

    /* inner loop over chunks of data */
    while (stdio_usb_connected()) {
        /* wait until there is a new chunk of data. note the forced volatile read */
        while (ichunk_read == *(volatile size_t *)&ichunk_written) yield();

        /* get pointers to last two framehalves and advance the read cursor */
        const int16_t * restrict chunk_prev = adc_chunks[(ichunk_read - 1) % CHUNKS];
        const int16_t * restrict chunk_this = adc_chunks[(ichunk_read + 0) % CHUNKS];
        ichunk_read++;

        /* copy samples from ring buffer, promote to floating point, multiply by window */
        for (size_t it_frame = 0, it_framehalf = 0; it_frame < T / 2; it_frame++, it_framehalf++)
            scratch_in[it_frame] = chunk_prev[it_framehalf] * window[it_frame];
        for (size_t it_frame = T / 2, it_framehalf = 0; it_frame < T; it_frame++, it_framehalf++)
            scratch_in[it_frame] = chunk_this[it_framehalf] * window[T / 2 - it_framehalf];

        /* do the r2c fft of the most recent two framehalves of samples */
        fft_evaluate_real(scratch_out, scratch_in, plan);

        /* unpack dc and nyquist bins */
        const float complex nyquist_bin = cimagf(scratch_out[0]);
        scratch_out[0] = crealf(scratch_out[0]);

        /* possibly reset power accumulator */
        if (0 == iframe_averaged)
            memset(spectrum_power, 0, sizeof(float) * F);

        /* accumulate power */
        for (size_t iw = 0; iw < F - 1; iw++)
            spectrum_power[iw] += cmagsquaredf(scratch_out[iw]);

        if (T / 2 < F) spectrum_power[T / 2] += cmagsquaredf(nyquist_bin);

        iframe_averaged++;
        if (fft_frames_per_average == iframe_averaged) {
            iframe_averaged = 0;

            /* dc and nyquist bins need to be weighted down to account for r2c fft */
            spectrum_power[0] *= 0.5f;
            if (T / 2 < F) spectrum_power[T / 2] *= 0.5f;

            /* emit beginning of line to stdout */
            dprintf(1, "%.5f,%.5f,", df, dt);

            /* map power to eight bit log scale values */
            for (size_t iw = 0; iw < F; iw++) {
                const float dB = 10.0f * log10f(spectrum_power[iw]);
                const uint8_t quantized = fminf(255.0f, fmaxf(0.0f, (dB - out_offset) * one_over_out_scale + 0.5f));

                out_line[2 * iw + 0] = hex_byte(quantized / 16U);
                out_line[2 * iw + 1] = hex_byte(quantized % 16U);
            }

            /* emit rest of line to stdout (via usb in this case) */
            write(1, (void *)out_line, 2 * F + 2);
        }
    }

    NVIC_SystemReset();
}
