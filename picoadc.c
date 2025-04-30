/* campbell, 2025, isc license to the extent applicable */

/* we will be directly accessing these RP2350 subsystems using these api functions */
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

/* don't remember why we need this, maybe just for dsb and wfe, but including it by name
 also prevents us from accidentally attempting to compile for RP2040 */
#include "RP2350.h"

#include "fft_anywhere.h"
#include "rp2_ili9341_scrolling.h"

/* c standard includes */
#include <math.h>
#include <stdlib.h>
#include <assert.h>

/* adc input pin number is 26 plus this value */
#define ADC_CHANNEL 1

/* we will hardcode this DMA channel for use by the ADC, since we have to hardcode which
 interrupt fires whenever it completes a transaction */
#define IDMA_ADC 0

/* 2^this is the total number of bytes of sram used for the big ring buffer. this value,
 and the number of chunks, indirectly determine the FFT resolution, since each chunk is
 used as half of an FFT frame */
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
    __DSB();

    /* depending on whether an interrupt (or other processor-waking event) has already
     become pending, this will EITHER put the processor to sleep until an interrupt becomes
     pending, OR it will clear the event-is-pending condition and NOT put the processor to
     sleep, and the main loop will re-check the condition and call this again */
    __WFE();
}

/* ring buffer writer cursor, increments by one each time a chunk is done, i.e. 4 times per
 ring buffer wraparound interval, if there are 4 chunks. incrementing happens inside an
 interrupt handler that fires when the DMA subsystem has finished writing that chunk worth
 of samples into the ring buffer from the ADC subsystem. assuming the processor is asleep
 waiting for the next chunk to finish, it will wake up and service the interrupt handler,
 which will increment the write cursor, and then resume running the main code which will
 see that the write cursor is ahead of the read cursor and react accordingly.

 note this is NOT volatile because we want to access it as a regular variable from within
 ISR context where it is safe to do so, and do explicitly volatile reads of it otherwise */
size_t ichunk_written = 0;

/* the actual ring buffer, divided into chunks, and aligned to its own total size */
__attribute((aligned(RING_BUFFER_SIZE)))
static int16_t adc_chunks[CHUNKS][SAMPLES_PER_CHUNK];

/* interrupt handler, gets called once per chunk, i.e. four times per full ring buffer */
void __scratch_y("") adc_dma_irq_handler_single(void) {
    /* acknowledge the interrupt so that it doesn't re-fire until the next chunk */
    dma_hw->ints0 = 1U << IDMA_ADC;

    /* increment the write cursor. as an unsigned integer type, this will eventually wrap
     around on some large power of 2 boundary, and as long as the read cursor(s) are the
     same type and we only compare read cursors to the write cursor using strict equality
     or by subtracting the read cursor from the write cursor, we can always see how far
     behind the reader is, even if the writer has wrapped around and the reader has not */
    ichunk_written++;

    /* enforce that the incremented value has propagated fully to memory before the main
     thread is allowed to look at it to see if it has changed, so that we don't erroneously
     put the processor back to sleep without handling the new chunk */
    __DSB();
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

    /* tell the ADC subsystem that the sample period should be this many main clock ticks */
    adc_set_clkdiv(sample_rate_denominator - 1);

    /* set up a DMA channel in the rp2350-specific ring buffer mode. each time the ADC
     tells the DMA that a sample can be read from the ADC's fifo, the DMA will copy it to
     the current address in the ring buffer and increment the dest address. after one chunk
     worth of samples, the transaction will finish and the interrupt handler will fire, but
     the channel will immediately retrigger itself and continue to increment the dest
     address. "ring" mode causes the dest address to wrap around at the given number of
     least significant bits of the address, i.e. as long as the whole buffer is aligned in
     absolute memory such that it starts with those least significant bits cleared, it will
     wrap around seamlessly */
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

    /* set up the interrupt handler to fire every time a transaction (one chunk) finishes */
    dma_channel_acknowledge_irq0(IDMA_ADC);
    dma_channel_set_irq0_enabled(IDMA_ADC, true);
    __DSB();
    irq_set_exclusive_handler(DMA_IRQ_0, adc_dma_irq_handler_single);
    irq_set_enabled(DMA_IRQ_0, true);

    /* start the DMA channel, and then start the ADC itself which will feed it */
    dma_channel_start(IDMA_ADC);
    adc_run(true);
}

static float cmagsquaredf(const float complex a) {
    /* this is equivalent to |a|^2, or a * conj(a), but computed more efficiently */
    return crealf(a) * crealf(a) + cimagf(a) * cimagf(a);
}

int main(void) {
    /* this is not a terribly cpu intensive program, so leave the main clock at 48 MHz
     note that this requires the USB PLL to be left enabled even if not used otherwise */
    set_sys_clock_48mhz();

    adc_dma_init();

    ili9341_scrolling_init();

    /* turn off clocks to a bunch of stuff we aren't using, saves about 4 mW */
    clocks_hw->wake_en1 = (CLOCKS_WAKE_EN1_BITS &
                           ~(CLOCKS_WAKE_EN1_CLK_SYS_USBCTRL_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_UART1_BITS |
                             CLOCKS_WAKE_EN1_CLK_PERI_UART1_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_UART0_BITS |
                             CLOCKS_WAKE_EN1_CLK_PERI_UART0_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_TRNG_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_TIMER1_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_TIMER0_BITS |
                             CLOCKS_WAKE_EN1_CLK_SYS_SPI1_BITS |
                             CLOCKS_WAKE_EN1_CLK_PERI_SPI1_BITS));

    clocks_hw->wake_en0 = (CLOCKS_WAKE_EN0_BITS &
                           ~(CLOCKS_WAKE_EN0_CLK_SYS_SHA256_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PWM_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO2_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO1_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_PIO0_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_JTAG_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_I2C1_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_I2C0_BITS |
                             CLOCKS_WAKE_EN0_CLK_SYS_HSTX_BITS |
                             CLOCKS_WAKE_EN0_CLK_HSTX_BITS));

    /* make sure we don't clock anything in sleep that wasn't clocked in wake */
    clocks_hw->sleep_en1 = clocks_hw->wake_en1;
    clocks_hw->sleep_en0 = clocks_hw->wake_en0;

    /* fft length. we use each chunk twice, first as the final 50% of an fft frame, then
     as the initial 50% of the following fft frame */
    const size_t T = 2 * SAMPLES_PER_CHUNK;

    /* number of kept frequency bins. the dc and nyquist bins are purely real, and the r2c
     fft computes them by packing the nyquist bin into the imaginary component of the dc
     bin, thereby only needing T/2 storage, but we will unpack them for subsequent logic */
    const size_t F = T / 2 + 1;

    /* plan an r2c fft. note that this internally calls malloc, which can and should
     raise eyebrows in embedded microcontroller code, along with the fact that it consumes
     memory at runtime with values that could be known at compile time. it IS possible to
     bake this fft plan into the binary, however this can also increase power consumption
     and latency if the fft plan does not fit in the rp2350's XIP cache (16 kB) */
    struct planned_real_fft * const plan = plan_real_fft_of_length(T);

    /* allocate a bunch of buffers. these can easily be static allocations if desired and
     if the above usage of malloc has also been factored out */
    float * restrict const scratch_in = malloc(sizeof(float) * T);
    float complex * restrict const scratch_out = malloc(sizeof(float complex) * T / 2);
    float * restrict const window = malloc(sizeof(float) * (T / 2 + 1));

    uint8_t * restrict const spectrum_quantized = malloc(sizeof(uint8_t) * F);

    /* hann window, exploiting symmetry, normalized such that a full scale real-valued sine
     wave will have a -3.0103 dB response */
    for (size_t it = 0; it < T / 2 + 1; it++)
        window[it] = (1.0f - cosf(2.0f * (float)M_PI * (float)it / T)) * (float)M_SQRT2 / (T * 2047.0f);

    /* initially we will pretend we are "caught up" with wherever the stream is now */
    size_t ichunk_read = *(volatile size_t *)&ichunk_written;

    /* output will be on [-96, 0) dB relative to full scale, in 0.75 dB increments */
    const float out_scale = 0.375f;
    const float out_offset = -256.0f * out_scale;
    const float one_over_out_scale = 1.0f / out_scale;

    /* inner loop over chunks of data */
    while (1) {
        /* wait until there is a new chunk of data. note the forced volatile read of a not
         otherwise volatile value, which tells the compiler it is not allowed to assume
         it already knows what this value is because it just read it */
        while (ichunk_read == *(volatile size_t *)&ichunk_written) yield();

        /* get pointers to most recent two chunks, and advance the read cursor */
        const int16_t * restrict chunk_prev = adc_chunks[(ichunk_read - 1) % CHUNKS];
        const int16_t * restrict chunk_this = adc_chunks[(ichunk_read + 0) % CHUNKS];
        ichunk_read++;

        /* copy samples from ring buffer into real-valued fft input scratch memory, in the
         process promoting the samples from 16-bit integers to 32-bit floating point, and
         multiplying by the hann window */
        for (size_t it_frame = 0, it_framehalf = 0; it_frame < T / 2; it_frame++, it_framehalf++)
            scratch_in[it_frame] = chunk_prev[it_framehalf] * window[it_frame];
        for (size_t it_frame = T / 2, it_framehalf = 0; it_frame < T; it_frame++, it_framehalf++)
            scratch_in[it_frame] = chunk_this[it_framehalf] * window[T / 2 - it_framehalf];

        /* do the real-to-complex fft of the most recent two framehalves of samples,
         putting the result in the length-T/2 scratch memory */
        fft_evaluate_real(scratch_out, scratch_in, plan);

        /* unpack dc and nyquist bins, which are real-valued and both shoved in the dc bin */
        const float complex nyquist_bin = cimagf(scratch_out[0]);
        scratch_out[0] = crealf(scratch_out[0]);

        /* loop over frequency bins (may not be full range) */
        for (size_t iw = 0; iw < F; iw++) {
            /* dc and nyquist bins need to be weighted down to account for r2c fft */
            const float power = (T / 2 == iw ? cmagsquaredf(nyquist_bin) * 0.5f :
                                 0 == iw ? cmagsquaredf(scratch_out[iw]) * 0.5f :
                                 cmagsquaredf(scratch_out[iw]));
            const float dB = 10.0f * log10f(power);

            spectrum_quantized[iw] = fminf(255.0f, fmaxf(0.0f, (dB - out_offset) * one_over_out_scale + 0.5f));
        }

        ili9341_write_row_and_scroll(spectrum_quantized);
    }

    /* not reached */
}
