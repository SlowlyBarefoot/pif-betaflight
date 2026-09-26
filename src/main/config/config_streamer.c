/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/flash.h"

#include "config/config_streamer.h"

#if !defined(CONFIG_IN_FLASH)
// Aligned to a program unit, since it is written a unit at a time.
#if defined(CONFIG_IN_RAM) && defined(PERSISTENT)
PERSISTENT uint8_t eepromData[EEPROM_SIZE] __attribute__((aligned(CONFIG_STREAMER_BUFFER_SIZE)));
#else
uint8_t eepromData[EEPROM_SIZE] __attribute__((aligned(CONFIG_STREAMER_BUFFER_SIZE)));
#endif
#endif


#if (defined(STM32H750xx) || defined(STM32H730xx)) && !(defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD))
#error "The configured MCU only has one flash page which contains the bootloader, no spare flash pages available, use external storage for persistent config or ram for target testing"
#endif
// @todo this is not strictly correct for F4/F7, where sector sizes are variable
#if !defined(FLASH_PAGE_SIZE)
// F1
# if defined(STM32F10X_MD)
#  define FLASH_PAGE_SIZE                 (0x400)
# elif defined(STM32F10X_HD)
#  define FLASH_PAGE_SIZE                 (0x800)
// F3
# elif defined(STM32F303xC)
#  define FLASH_PAGE_SIZE                 (0x800)
// F4
# elif defined(STM32F40_41xxx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors
# elif defined (STM32F411xE)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
# elif defined(STM32F427_437xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
# elif defined (STM32F446xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
// F7
#elif defined(STM32F722xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors
# elif defined(STM32F745xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000) // 32K sectors
# elif defined(STM32F746xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000)
# elif defined(STM32F765xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x8000)
# elif defined(UNIT_TEST)
#  define FLASH_PAGE_SIZE                 (0x400)
// H7
# elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x20000) // 128K sectors
# elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x2000) // 8K sectors
// G4
# elif defined(STM32G4)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x800) // 2K page
// SIMULATOR
# elif defined(SIMULATOR_BUILD)
#  define FLASH_PAGE_SIZE                 (0x400)
# else
#  error "Flash page size not defined for target."
# endif
#endif

#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_SDCARD)
// No flash sector method required.
#elif defined(CONFIG_IN_FLASH)
#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
/*
Sector 0    0x08000000 - 0x08007FFF 32 Kbytes
Sector 1    0x08008000 - 0x0800FFFF 32 Kbytes
Sector 2    0x08010000 - 0x08017FFF 32 Kbytes
Sector 3    0x08018000 - 0x0801FFFF 32 Kbytes
Sector 4    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 5    0x08040000 - 0x0807FFFF 256 Kbytes
Sector 6    0x08080000 - 0x080BFFFF 256 Kbytes
Sector 7    0x080C0000 - 0x080FFFFF 256 Kbytes

F7X5XI device with 2M flash
Sector 8    0x08100000 - 0x0813FFFF 256 Kbytes
Sector 9    0x08140000 - 0x0817FFFF 256 Kbytes
Sector 10   0x08180000 - 0x081BFFFF 256 Kbytes
Sector 11   0x081C0000 - 0x081FFFFF 256 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&__config_start <= 0x08017FFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&__config_start <= 0x080BFFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&__config_start <= 0x080FFFFF)
        return FLASH_SECTOR_7;
#if defined(STM32F765xx)
    if ((uint32_t)&__config_start <= 0x0813FFFF)
        return FLASH_SECTOR_8;
    if ((uint32_t)&__config_start <= 0x0817FFFF)
        return FLASH_SECTOR_9;
    if ((uint32_t)&__config_start <= 0x081BFFFF)
        return FLASH_SECTOR_10;
    if ((uint32_t)&__config_start <= 0x081FFFFF)
        return FLASH_SECTOR_11;
#endif

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32F722xx)
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08003FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&__config_start <= 0x0800BFFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&__config_start <= 0x0805FFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_SECTOR_7;

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32F4)
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes
*/

static uint32_t getFLASHSectorForEEPROM(void)
{
    if ((uint32_t)&__config_start <= 0x08003FFF)
        return FLASH_Sector_0;
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_Sector_1;
    if ((uint32_t)&__config_start <= 0x0800BFFF)
        return FLASH_Sector_2;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_Sector_3;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_Sector_4;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_Sector_5;
    if ((uint32_t)&__config_start <= 0x0805FFFF)
        return FLASH_Sector_6;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_Sector_7;
    if ((uint32_t)&__config_start <= 0x0809FFFF)
        return FLASH_Sector_8;
    if ((uint32_t)&__config_start <= 0x080DFFFF)
        return FLASH_Sector_9;
    if ((uint32_t)&__config_start <= 0x080BFFFF)
        return FLASH_Sector_10;
    if ((uint32_t)&__config_start <= 0x080FFFFF)
        return FLASH_Sector_11;

    // Not good
    while (1) {
        failureMode(FAILURE_CONFIG_STORE_FAILURE);
    }
}

#elif defined(STM32H743xx) || defined(STM32G4) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ) || defined(STM32H723xx) || defined(STM32H725xx)
/*
MCUs with uniform array of equal size sectors, handled in two banks having contiguous address.
(Devices with non-contiguous flash layout is not currently useful anyways.)

H743
2 bank * 8 sector/bank * 128K/sector (2MB)
Bank 1 0x08000000 - 0x080FFFFF 128KB * 8
Bank 2 0x08100000 - 0x081FFFFF 128KB * 8

H743
1 bank * 8 sector/bank * 128K/sector (1MB)
Bank 1 0x08000000 - 0x080FFFFF 128KB * 8

H7A3
2 bank * 128 sector/bank * 8KB/sector (2MB)
Bank 1 0x08000000 - 0x080FFFFF 8KB * 128
Bank 2 0x08100000 - 0x081FFFFF 8KB * 128

G473/474 in dual bank mode
2 bank * 128 sector/bank * 2KB/sector (512KB)
Bank 1 0x08000000 - 0x0803FFFF 2KB * 128
Bank 2 0x08040000 - 0x0807FFFF 2KB * 128

Note that FLASH_BANK_SIZE constant used in the following code changes depending on
bank operation mode. The code assumes dual bank operation, in which case the
FLASH_BANK_SIZE constant is set to one half of the available flash size in HAL.
*/

#if defined(STM32H743xx) || defined(STM32H723xx) || defined(STM32H725xx)
#define FLASH_PAGE_PER_BANK 8
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define FLASH_PAGE_PER_BANK 128
#elif defined(STM32G4)
#define FLASH_PAGE_PER_BANK 128
// These are not defined in CMSIS like H7
#define FLASH_BANK1_BASE FLASH_BASE
#define FLASH_BANK2_BASE (FLASH_BANK1_BASE + FLASH_BANK_SIZE)
#endif

static void getFLASHSectorForEEPROM(uint32_t address, uint32_t *bank, uint32_t *sector)
{
#if defined(FLASH_BANK2_BASE)
    if (address >= FLASH_BANK1_BASE && address < FLASH_BANK2_BASE) {
        *bank = FLASH_BANK_1;
    } else if (address >= FLASH_BANK2_BASE && address < FLASH_BANK2_BASE + FLASH_BANK_SIZE) {
        *bank = FLASH_BANK_2;
        address -= FLASH_BANK_SIZE;
    }
#else
    if (address >= FLASH_BANK1_BASE && address < FLASH_BANK1_BASE + FLASH_BANK_SIZE) {
        *bank = FLASH_BANK_1;
    }
#endif
    else {
        // Not good
        while (1) {
            failureMode(FAILURE_CONFIG_STORE_FAILURE);
        }
    }

    address -= FLASH_BANK1_BASE;
    *sector = address / FLASH_PAGE_SIZE;
}
#elif defined(STM32H750xx)
/*
The memory map supports 2 banks of 8 128k sectors like the H743xx, but there is only one 128K sector so we save some code
space by using a smaller function.

Bank 1
Sector 0    0x08000000 - 0x0801FFFF 128 Kbytes

*/

static void getFLASHSectorForEEPROM(uint32_t *bank, uint32_t *sector)
{

    uint32_t start = (uint32_t)&__config_start;

    if (start == FLASH_BANK1_BASE) {
        *sector = FLASH_SECTOR_0;
        *bank = FLASH_BANK_1;
    } else {
        // Not good
        while (1) {
            failureMode(FAILURE_CONFIG_STORE_FAILURE);
        }
    }
}
#endif
#endif // CONFIG_IN_FLASH

// The config is written through PIF's pif_flash, which gathers the stream
// into program units of CONFIG_STREAMER_BUFFER_SIZE bytes, erases each erase
// unit as the stream reaches it and pads the last unit. What is left here is
// the storage itself: unlocking and locking it, erasing a unit and programming
// one, for each place the config can live.

STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE <= PIF_FLASH_MAX_PROGRAM_SIZE, configStreamerProgramSize);

#if defined(CONFIG_IN_EXTERNAL_FLASH)
// Whether flashPageProgramBegin() has been called for a page that
// flashPageProgramFinish() has not ended yet.
static bool configPageOpen;
#endif

#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD) || defined(CONFIG_IN_FILE)
// PifFlash addresses are 32 bits, which a pointer of the simulator is not, so
// a region in memory is addressed from 0 and placed here.
static uintptr_t configMemoryBase;
#define CONFIG_MEMORY(address) (configMemoryBase + (address))
#else
#define CONFIG_MEMORY(address) ((uintptr_t)(address))
#endif

static void configStreamerLock(PifFlash *flash, BOOL lock)
{
    UNUSED(flash);

    if (!lock) {
#if defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD)
        memset(eepromData, 0, sizeof(eepromData));
#elif defined(CONFIG_IN_EXTERNAL_FLASH)
        configPageOpen = false;
#elif defined(CONFIG_IN_FLASH) || defined(CONFIG_IN_FILE)
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        HAL_FLASH_Unlock();
#else
        FLASH_Unlock();
#endif
#endif

#if defined(CONFIG_IN_FLASH)
#if defined(STM32F10X)
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#elif defined(STM32F303)
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F4)
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(UNIT_TEST) || defined(SIMULATOR_BUILD)
        // NOP
#else
# error "Unsupported CPU"
#endif
#endif
        return;
    }

#if defined(CONFIG_IN_SDCARD)
    bool saveEEPROMToSDCard(void); // XXX forward declaration to avoid circular dependency between config_streamer / config_eeprom
    saveEEPROMToSDCard();
    // TODO overwrite the data in the file on the SD card.
#elif defined(CONFIG_IN_EXTERNAL_FLASH)
    flashFlush();
    configPageOpen = false;
#elif defined(CONFIG_IN_RAM)
    // NOP
#elif defined(CONFIG_IN_FILE)
    FLASH_Lock();
#elif defined(CONFIG_IN_FLASH)
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    HAL_FLASH_Lock();
#else
    FLASH_Lock();
#endif
#endif
}

#if defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_FLASH)
static BOOL configStreamerErase(PifFlash *flash, uint32_t address)
{
    UNUSED(flash);

#if defined(CONFIG_IN_EXTERNAL_FLASH)
    // A page being programmed is ended before the chip is asked for anything else.
    if (configPageOpen) {
        flashPageProgramFinish();
        configPageOpen = false;
    }
    flashEraseSector(address);
    return TRUE;
#elif defined(STM32H7)
    FLASH_EraseInitTypeDef EraseInitStruct = {
        .TypeErase     = FLASH_TYPEERASE_SECTORS,
#if !(defined(STM32H7A3xx) || defined(STM32H7A3xxQ))
        .VoltageRange  = FLASH_VOLTAGE_RANGE_3, // 2.7-3.6V
#endif
        .NbSectors     = 1
    };
    getFLASHSectorForEEPROM(address, &EraseInitStruct.Banks, &EraseInitStruct.Sector);
    uint32_t SECTORError;
    return HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) == HAL_OK;
#elif defined(STM32F7)
    UNUSED(address);
    FLASH_EraseInitTypeDef EraseInitStruct = {
        .TypeErase     = FLASH_TYPEERASE_SECTORS,
        .VoltageRange  = FLASH_VOLTAGE_RANGE_3, // 2.7-3.6V
        .NbSectors     = 1
    };
    EraseInitStruct.Sector = getFLASHSectorForEEPROM();
    uint32_t SECTORError;
    return HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) == HAL_OK;
#elif defined(STM32G4)
    FLASH_EraseInitTypeDef EraseInitStruct = {
        .TypeErase     = FLASH_TYPEERASE_PAGES,
        .NbPages       = 1
    };
    getFLASHSectorForEEPROM(address, &EraseInitStruct.Banks, &EraseInitStruct.Page);
    uint32_t SECTORError;
    return HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) == HAL_OK;
#elif defined(STM32F4)
    UNUSED(address);
    return FLASH_EraseSector(getFLASHSectorForEEPROM(), VoltageRange_3) == FLASH_COMPLETE; //0x08080000 to 0x080A0000
#else // STM32F3, STM32F1
    return FLASH_ErasePage(address) == FLASH_COMPLETE;
#endif
}
#endif

static BOOL configStreamerProgram(PifFlash *flash, uint32_t address, const uint8_t *data, uint16_t size)
{
    UNUSED(flash);

#if defined(CONFIG_IN_EXTERNAL_FLASH)
    const flashGeometry_t *flashGeometry = flashGetGeometry();

    if (address % flashGeometry->pageSize == 0) {
        if (configPageOpen) {
            flashPageProgramFinish();
        }
        flashPageProgramBegin(address, NULL);
        configPageOpen = true;
    }

    const uint8_t *buffers[1] = { data };
    uint32_t bufferSizes[1] = { size };
    flashPageProgramContinue(buffers, bufferSizes, 1);
    return TRUE;
#elif defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD)
    memcpy((void *)CONFIG_MEMORY(address), data, size);
    return TRUE;
#elif defined(CONFIG_IN_FILE)
    UNUSED(size);
    return FLASH_ProgramWord(CONFIG_MEMORY(address), *(const uint32_t *)data) == FLASH_COMPLETE;
#elif defined(CONFIG_IN_FLASH)
    UNUSED(size);
#if defined(STM32H7)
    // For H7
    // HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t DataAddress);
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint64_t)(uint32_t)data) == HAL_OK;
#elif defined(STM32F7)
    // For F7
    // HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint64_t)*(const uint32_t *)data) == HAL_OK;
#elif defined(STM32G4)
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *(const uint64_t *)data) == HAL_OK;
#else // !STM32H7 && !STM32F7 && !STM32G4
    return FLASH_ProgramWord(address, *(const uint32_t *)data) == FLASH_COMPLETE;
#endif
#endif
}

#if !defined(CONFIG_IN_EXTERNAL_FLASH)
static BOOL configStreamerRead(PifFlash *flash, uint32_t address, uint8_t *data, size_t size)
{
    UNUSED(flash);

    memcpy(data, (const void *)CONFIG_MEMORY(address), size);
    return TRUE;
}
#endif

void config_streamer_init(config_streamer_t *c)
{
    memset(c, 0, sizeof(*c));
}

void config_streamer_start(config_streamer_t *c, uintptr_t base, int size)
{
    bool ready;

#if defined(CONFIG_IN_EXTERNAL_FLASH)
    // The config goes to its partition of the flash chip, whatever base says.
    UNUSED(base);
    UNUSED(size);
    const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_CONFIG);
    const flashGeometry_t *flashGeometry = flashGetGeometry();
    ready = flashPartition && pifFlash_Init(&c->flash, PIF_ID_AUTO,
        flashPartition->startSector * flashGeometry->sectorSize,
        (flashPartition->endSector + 1 - flashPartition->startSector) * flashGeometry->sectorSize,
        flashGeometry->sectorSize, CONFIG_STREAMER_BUFFER_SIZE, configStreamerErase, configStreamerProgram, NULL);
#elif defined(CONFIG_IN_FLASH)
    // base must start at FLASH_PAGE_SIZE boundary when using embedded flash.
    ready = pifFlash_Init(&c->flash, PIF_ID_AUTO, base, size, FLASH_PAGE_SIZE, CONFIG_STREAMER_BUFFER_SIZE,
        configStreamerErase, configStreamerProgram, configStreamerRead);
#else
    // RAM, an SD card image in RAM, or the simulator's file, none of which
    // is erased before it is written.
    configMemoryBase = base;
    ready = pifFlash_Init(&c->flash, PIF_ID_AUTO, 0, size, 0, CONFIG_STREAMER_BUFFER_SIZE,
        NULL, configStreamerProgram, configStreamerRead);
#endif

    if (!ready) {
        c->err = -1;
        return;
    }

    pifFlash_AttachActLock(&c->flash, configStreamerLock);
    pifFlash_Begin(&c->flash);
    c->err = 0;
}

static int configStreamerError(void)
{
    // FIXME the return values are currently magic numbers
    switch (pif_error) {
    case E_OVERFLOW_BUFFER:
        return -3; // past end of the config region
    case E_ACCESS_FAILED:
        return -2; // erase or program failed
    default:
        return -1;
    }
}

int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size)
{
    if (c->err == 0 && !pifFlash_Write(&c->flash, p, size)) {
        c->err = configStreamerError();
    }
    return c->err;
}

int config_streamer_status(config_streamer_t *c)
{
    return c->err;
}

int config_streamer_flush(config_streamer_t *c)
{
    // pifFlash_End() programs what is left over, padded to a whole unit.
    return c->err;
}

int config_streamer_finish(config_streamer_t *c)
{
    if (c->flash._writing && !pifFlash_End(&c->flash) && c->err == 0) {
        c->err = configStreamerError();
    }
    return c->err;
}
