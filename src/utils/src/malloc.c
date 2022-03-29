#include "FreeRTOS.h"

void* malloc(size_t size)
{
    void* ptr = NULL;

    if (size > 0)
    {
        ptr = pvPortMalloc(size);
    }

    return ptr;
}

void free(void* ptr)
{
    if (ptr)
    {
        vPortFree(ptr);
    }
}

