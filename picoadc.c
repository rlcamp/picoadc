#include "pico/stdio_uart.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "RP2350.h"

#include "bsp/board_api.h"
#include <tusb.h>

#include "fft_anywhere.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>

/* as a diagnostic we will output a 2 kHz full scale square wave on this pin */
#define PWM_PIN 22

/* adc input pin number is 26 plus this value */
#define ADC_CHANNEL 1

#define IDMA_ADC 0

/* 2^this is the total number of bytes of sram used for the big ring buffer */
#define RING_BUFFER_WRAP_BITS 12

/* total number of bytes of sram used for ring buffer */
#define RING_BUFFER_SIZE (1U << RING_BUFFER_WRAP_BITS)

/* when we finish collecting chunk N, we use chunk N-1 and N to affect 50% overlapped FFT
 inputs, while chunk N+1 is collected. therefore we need at least three chunks, and we also
 have a power of two alignment requirement, therefore the minimum number of chunks is 4.
 higher numbers of chunks allow the main thread to momentarily fall farther behind real
 time as long as it can catch back up on average */
#define CHUNKS 4
#define SAMPLES_PER_CHUNK (RING_BUFFER_SIZE / (CHUNKS * sizeof(int16_t)))
static_assert(CHUNKS >= 4 && !(CHUNKS & (CHUNKS - 1)),
              "number of chunks must be a power of two, at least 4");

#define SAMPLE_RATE_NUMERATOR 48000000ULL
unsigned long sample_rate_denominator = 1500;

void yield(void) {
    /* we could do context switching here for cooperative multitasking if we wanted */
    asm volatile("dsb; wfe");
}

/* note this is NOT volatile because we want to access it as a regular variable from within
 ISR context where it is safe to do so, and do explicitly volatile reads of it otherwise */
size_t ichunk_written = 0;

/* the actual big ring buffer, divided into 4 chunks, aligned to its own total size */
__attribute((aligned(RING_BUFFER_SIZE)))
static int16_t adc_chunks[CHUNKS][SAMPLES_PER_CHUNK];

/* this function gets called once per chunk, i.e. four times per full ring buffer */
void __scratch_y("") adc_dma_irq_handler_single(void) {
    dma_hw->ints0 = 1U << IDMA_ADC;

    ichunk_written++;
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

    dma_channel_claim(IDMA_ADC);
    dma_channel_config cfg = dma_channel_get_default_config(IDMA_ADC);
    channel_config_set_dreq(&cfg, DREQ_ADC);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_ring(&cfg, true, RING_BUFFER_WRAP_BITS);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);

    dma_channel_configure(IDMA_ADC,
                          &cfg,
                          &adc_chunks[0],
                          &adc_hw->fifo,
                          SAMPLES_PER_CHUNK | (1U << 28), /* enable self retrigger */
                          false);

    dma_channel_acknowledge_irq0(IDMA_ADC);
    dma_channel_set_irq0_enabled(IDMA_ADC, true);
    __DSB();
    irq_set_exclusive_handler(DMA_IRQ_0, adc_dma_irq_handler_single);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(IDMA_ADC);
    adc_run(true);
}

static float cmagsquaredf(const float complex a) {
    return crealf(a) * crealf(a) + cimagf(a) * cimagf(a);
}

extern void write_to_usb_cdc(const char * buf, size_t length);

static size_t base64_encoded_size_including_null_termination(const size_t plain_size) {
    return ((plain_size + 2) / 3) * 4 + 1;
}

static char * base64_encode(char * dest, const void * plainv, const size_t plain_size) {
    static const char symbols[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const unsigned char * plain = plainv;
    const size_t encoded_size = base64_encoded_size_including_null_termination(plain_size) - 1;
    char * encoded_start = dest ? dest : malloc(encoded_size + 1);
    char * encoded = encoded_start;

    /* loop over all complete groups of four encoded, three decoded bytes */
    for (const unsigned char * const stop = plain + plain_size - 2; plain < stop; encoded += 4, plain += 3) {
        const unsigned int merged = plain[0] << 16 | plain[1] << 8 | plain[2];
        encoded[0] = symbols[(merged >> 18) & 0x3F];
        encoded[1] = symbols[(merged >> 12) & 0x3F];
        encoded[2] = symbols[(merged >>  6) & 0x3F];
        encoded[3] = symbols[(merged      ) & 0x3F];
    }

    /* handle partial group at end */
    if (plain_size % 3) {
        const unsigned int merged = plain[0] << 16 | (2 == plain_size % 3 ? plain[1] << 8 : 0);
        encoded[0] = symbols[(merged >> 18) & 0x3F];
        encoded[1] = symbols[(merged >> 12) & 0x3F];
        encoded[2] = (2 == plain_size % 3) ? symbols[(merged >> 6) & 0x3F] : '=';
        encoded[3] = '=';
        encoded[4] = '\0';
    }
    else
        encoded[0] = '\0';

    return encoded_start;
}


int main(void) {
    set_sys_clock_48mhz();

    adc_dma_init();

    board_init();
    tusb_init();

    if (board_init_after_tusb)
        board_init_after_tusb();

    /* init stdout/stderr on uart tx, do not enable uart rx */
    stdio_uart_init_full(uart_default, 115200, PICO_DEFAULT_UART_TX_PIN, -1);

    init_test_signal();

    /* fft length */
    const size_t T = 2 * SAMPLES_PER_CHUNK;

    /* number of kept frequency bins */
    const size_t F = T / 2 + 1;

    const float fs = SAMPLE_RATE_NUMERATOR / (float)sample_rate_denominator;

    /* number of fft frames to average for each line of output pixels */
    /* WARNING: at small enough values of this, we won't be able to send out the rows of
     spectrum data fast enough on the physical uart */
    const float dt_desired = 0.2f; /* seconds */
    const size_t fft_frames_per_average = fmaxf(1.0f, dt_desired * fs / (0.5f * T) + 0.5f);

    const float df = fs / T, dt = 0.5f * fft_frames_per_average * T / fs;

    /* plan an r2c fft */
    struct planned_real_fft * const plan = plan_real_fft_of_length(T);

    /* allocate a bunch of buffers */
    float * restrict const scratch_in = malloc(sizeof(float) * T);
    float complex * restrict const scratch_out = malloc(sizeof(float complex) * T / 2);
    float * restrict const window = malloc(sizeof(float) * (T / 2 + 1));
    float * restrict const spectrum_power = malloc(sizeof(float) * F);

    uint8_t * restrict const spectrum_quantized = malloc(sizeof(uint8_t) * F);

    const size_t strlen_spectrum_encoded = base64_encoded_size_including_null_termination(F) - 1;

    const size_t outlen = sizeof("000.00000,000.00000,\r\n") + strlen_spectrum_encoded;
    char * restrict const line_out = malloc(outlen);

    /* hann window, exploiting symmetry, accounting for averaging over time, normalized
     such that a full scale real-valued sine wave will have a -3.0103 dB response */
    for (size_t it = 0; it < T / 2 + 1; it++)
        window[it] = (1.0f - cosf(2.0f * (float)M_PI * (float)it / T)) * (float)M_SQRT2 / (T * 2047.0f * sqrtf(fft_frames_per_average));

    /* initially we will pretend we are "caught up" with wherever the stream is now */
    size_t ichunk_read = *(volatile size_t *)&ichunk_written;
    size_t iframe_averaged = 0;

    /* output will be on [-192, 0) dB relative to full scale, in 0.75 dB increments */
    const float out_scale = 0.75f;
    const float out_offset = -256.0f * out_scale;
    const float one_over_out_scale = 1.0f / out_scale;

    /* inner loop over chunks of data */
    while (1) {
        /* need to unconditionally do this regularly to make sure usb stuff happens */
        tud_task();

        /* wait until there is a new chunk of data. note the forced volatile read */
        while (ichunk_read == *(volatile size_t *)&ichunk_written) {
            tud_task();
            yield();
        }

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

        /* accumulate power */
        if (0 == iframe_averaged) {
            for (size_t iw = 0; iw < F - 1; iw++)
                spectrum_power[iw] = cmagsquaredf(scratch_out[iw]);

            if (T / 2 < F) spectrum_power[T / 2] = cmagsquaredf(nyquist_bin);
        } else {
            for (size_t iw = 0; iw < F - 1; iw++)
                spectrum_power[iw] += cmagsquaredf(scratch_out[iw]);

            if (T / 2 < F) spectrum_power[T / 2] += cmagsquaredf(nyquist_bin);
        }

        iframe_averaged++;
        if (fft_frames_per_average == iframe_averaged) {
            iframe_averaged = 0;

            /* dc and nyquist bins need to be weighted down to account for r2c fft */
            spectrum_power[0] *= 0.5f;
            if (T / 2 < F) spectrum_power[T / 2] *= 0.5f;

            /* write into first part of output line */
            size_t off = snprintf(line_out, outlen, "%.5f,%.5f,", df, dt);

            /* map power to eight bit log scale values */
            for (size_t iw = 0; iw < F; iw++) {
                const float dB = 10.0f * log10f(spectrum_power[iw]);
                spectrum_quantized[iw] = fminf(255.0f, fmaxf(0.0f, (dB - out_offset) * one_over_out_scale + 0.5f));
            }

            base64_encode(line_out + off, spectrum_quantized, F);
            off += strlen_spectrum_encoded;

            /* finish line */
            off += snprintf(line_out + off, outlen - off, "\r\n");

            /* emit line to usb cdc serial */
            if (tud_cdc_connected())
                write_to_usb_cdc(line_out, off);

            /* emit line to uart */
            write(1, line_out, off);
        }
    }

    /* not reached */
}
