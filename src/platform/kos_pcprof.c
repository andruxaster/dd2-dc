/* A sampling profiler for the Dreamcast build (DD2_PROFILE): the watchdog timer, as an interval timer,
 * notes where the CPU was 1000 times a second; every 20 seconds the busiest 256-byte stretches of code go
 * to the console as "pcprof <address> <per mille>", to be matched with functions in dd2.elf
 * (sh-elf-nm -n dd2.elf). The TMU channels all belong to KallistiOS. */
#include <dc/wdt.h>
#include <kos.h>
#include <stdio.h>
#include <string.h>

#define CODE_START 0x8c010000u
#define BUCKET_SHIFT 8
#define BUCKETS 8192 /* 2 MB of code */
#define SHOWN 40
#define PERIOD_SAMPLES 20000

static volatile uint32_t counts[BUCKETS];
static volatile uint32_t samples, outside;

static void sample(void *data)
{
    (void)data;
    irq_context_t *context = irq_get_context();
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
    wdt_enable_timer(0, 1000, 15, sample, NULL);
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
