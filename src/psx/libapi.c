#include "psx/libapi.h"

#include <stddef.h>
#include <stdlib.h>

/* There are no interrupts to mask: callbacks run from VSync/etc_service on the game's own thread. */
int EnterCriticalSection(void)
{
    return 1;
}

void ExitCriticalSection(void)
{
}

/* The BIOS heap. The game resets it with InitHeap at each change of mode (front end, race, results),
 * dropping every block at once; it never frees most of them itself. Blocks come from the C library here,
 * so they are kept on a list and InitHeap frees them all, or each race would leak its buffers (on the
 * Dreamcast, out of memory after a few races). */
typedef union HeapBlock {
    struct {
        union HeapBlock *prev, *next;
        size_t size;
    } link;
    max_align_t align;
} HeapBlock;

static HeapBlock heap_list = {{&heap_list, &heap_list, 0}};

void *heap_alloc(size_t size)
{
    HeapBlock *b = malloc(sizeof(HeapBlock) + size);
    if (!b)
        return NULL;
    b->link.size = size;
    b->link.prev = &heap_list;
    b->link.next = heap_list.link.next;
    heap_list.link.next->link.prev = b;
    heap_list.link.next = b;
    return b + 1;
}

void heap_free(void *p)
{
    if (!p)
        return;
    HeapBlock *b = (HeapBlock *)p - 1;
    b->link.prev->link.next = b->link.next;
    b->link.next->link.prev = b->link.prev;
    free(b);
}

/* Shrinking keeps the block where it is, as the BIOS heap does: the game keeps pointers into a file's
 * buffer after cutting it to what it needs (the texture table). */
void *heap_realloc(void *p, size_t size)
{
    if (!p)
        return heap_alloc(size);
    HeapBlock *b = (HeapBlock *)p - 1;
    if (size <= b->link.size)
        return p;
    HeapBlock *prev = b->link.prev, *next = b->link.next;
    HeapBlock *n = realloc(b, sizeof(HeapBlock) + size);
    if (!n)
        return NULL;
    n->link.size = size;
    prev->link.next = n;
    next->link.prev = n;
    return n + 1;
}

void InitHeap(u_long *head, u_long size)
{
    (void)head;
    (void)size;
    while (heap_list.link.next != &heap_list)
        heap_free(heap_list.link.next + 1);
}

/* Events are only delivered by the memory card emulation (psx/libcard.c); callbacks and interrupt modes
 * are not needed by the game. */
#define MAX_EVENTS 32
#define EVENT_BASE 0xf1000000

static struct {
    u_long desc;
    long spec;
    int used, enabled, ready;
} events[MAX_EVENTS];

long OpenEvent(u_long desc, long spec, long mode, long (*func)(void))
{
    (void)mode;
    (void)func;
    /* PORT: the game opens the memory card's events again after each movie; the BIOS would keep
     * the old ones too until its table is full. The same class and kind gets the same event here. */
    for (int i = 0; i < MAX_EVENTS; i++) {
        if (events[i].used && events[i].desc == desc && events[i].spec == spec) {
            events[i].enabled = 0;
            events[i].ready = 0;
            return EVENT_BASE + i;
        }
    }
    for (int i = 0; i < MAX_EVENTS; i++) {
        if (!events[i].used) {
            events[i].desc = desc;
            events[i].spec = spec;
            events[i].used = 1;
            events[i].enabled = 0;
            events[i].ready = 0;
            return EVENT_BASE + i;
        }
    }
    return -1;
}

static int event_index(long event)
{
    long i = event - EVENT_BASE;
    return i >= 0 && i < MAX_EVENTS && events[i].used ? (int)i : -1;
}

long EnableEvent(long event)
{
    int i = event_index(event);
    if (i < 0)
        return 0;
    events[i].enabled = 1;
    return 1;
}

long TestEvent(long event)
{
    int i = event_index(event);
    if (i < 0 || !events[i].ready)
        return 0;
    events[i].ready = 0;
    return 1;
}

void DeliverEvent(u_long desc, long spec)
{
    for (int i = 0; i < MAX_EVENTS; i++)
        if (events[i].used && events[i].enabled && events[i].desc == desc && events[i].spec == spec)
            events[i].ready = 1;
}

/* Root counters only drive Profile_Init's timing, which the port does not use. */
long SetRCnt(u_long spec, u_short target, long mode)
{
    (void)spec;
    (void)target;
    (void)mode;
    return 1;
}

long SetSp(long sp)
{
    (void)sp;
    return 0;
}
