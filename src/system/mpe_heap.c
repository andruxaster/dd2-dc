#include "system/mpe_heap.h"

#include "psx/libapi.h"

static uint8_t heap_area[HEAP_SIZE];
uint8_t *heap_base = heap_area;

void *MPE_malloc(size_t size)
{
    return heap_alloc(size);
}

void MPE_free(void *p)
{
    heap_free(p);
}

void *MPE_realloc(void *p, size_t size)
{
    return heap_realloc(p, size);
}
