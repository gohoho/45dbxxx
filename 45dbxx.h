#ifndef _45DBXX_H
#define _45DBXX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint8_t FlashSize_MBit;
    uint32_t FlashSize;
    uint16_t PageSize;
    uint16_t Pages;
    uint8_t Shift;
} AT45dbxx_t;

extern AT45dbxx_t AT45dbxx;

bool AT45dbxx_Init(void);
int AT45dbxx_EraseChip(void);
bool AT45dbxx_ErasePage(uint16_t page);
bool AT45dbxx_WritePage(uint16_t page, uint16_t offset, const void *data, uint16_t len);
int AT45dbxx_ReadPage(uint16_t page, void *data, uint16_t len);

int AT45dbxx_Write(uint32_t addr, const void *buf, size_t len);
int AT45dbxx_Read(uint32_t addr, void *buf, size_t len);

int AT45dbxx_Resume(void);
int AT45dbxx_PowerDown(void);

#endif
