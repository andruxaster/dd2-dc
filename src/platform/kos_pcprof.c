/* A sampling profiler for the Dreamcast build (DD2_PROFILE): timer channel 1 notes where the CPU was 1000
 * times a second; every 20 seconds the busiest 64-byte stretches of code go to the console as
 * "pcprof <address> <per mille>", to be matched with functions in dd2.elf (sh-elf-nm -n dd2.elf).
 * KallistiOS only needs channel 1 for nothing but its default handler; the watchdog timer it suggests
 * instead does not interrupt in emulators such as Flycast. */
#include <arch/timer.h>
#include <kos.h>
#include <stdio.h>
#include <string.h>

#define CODE_START 0x8c010000u
#define BUCKET_SHIFT 6
#define BUCKETS 32768 /* 2 MB of code */
#define SHOWN 60
#define PERIOD_SAMPLES 20000

static volatile uint32_t counts[BUCKETS];
static volatile uint32_t samples, outside;

static void sample(irq_t source, irq_context_t *context, void *data)
{
    (void)source, (void)data;
    timer_clear(TMU1);
    uint32_t pc = context ? context->pc : 0;
    uint32_t bucket = (pc - CODE_START) >> BUCKET_SHIFT;
    if (bucket < BUCKETS)
        counts[bucket]++;
    else
        outside++;
    samples++;
}

void kos_pcprof_start(void)
{
    irq_set_handler(EXC_TMU1_TUNI1, sample, NULL);
    timer_prime(TMU1, 1000, 1);
    timer_start(TMU1);
}

uint32_t kos_pcprof_samples(void)
{
    return samples;
}

/* Prints and clears the counts once enough samples are in. */
void kos_pcprof_poll(void)
{
    if (samples < PERIOD_SAMPLES)
        return;
    static uint32_t copy[BUCKETS];
    int old = irq_disable();
    memcpy(copy, (const void *)counts, sizeof(copy));
    memset((void *)counts, 0, sizeof(counts));
    uint32_t total = samples, out = outside;
    samples = outside = 0;
    irq_restore(old);
    printf("pcprof: %lu samples, %lu outside the code\n", (unsigned long)total, (unsigned long)out);
    for (int n = 0; n < SHOWN; n++) {
        int best = -1;
        for (int i = 0; i < BUCKETS; i++)
            if (copy[i] && (best < 0 || copy[i] > copy[best]))
                best = i;
        if (best < 0)
            break;
        printf("pcprof %08lx %lu\n", (unsigned long)(CODE_START + ((uint32_t)best << BUCKET_SHIFT)),
               (unsigned long)(copy[best] * 1000u / total));
        copy[best] = 0;
    }
}
