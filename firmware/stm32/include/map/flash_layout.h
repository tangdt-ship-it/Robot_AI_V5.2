#ifndef STM32_MAP_FLASH_LAYOUT_H
#define STM32_MAP_FLASH_LAYOUT_H

#include <stdint.h>

// Audited for the platformio genericSTM32F103VE target:
// STM32F103VET6, 512 KiB Flash, 64 KiB SRAM, 2 KiB erase pages.
namespace Stm32FlashLayout {
constexpr uint32_t kFlashOrigin = 0x08000000UL;
constexpr uint32_t kFlashLength = 0x00080000UL;
constexpr uint32_t kFlashPageSize = 0x00000800UL;
constexpr uint32_t kFlashEnd = kFlashOrigin + kFlashLength;

// Existing wheel calibration record. Do not move without a migration plan.
constexpr uint32_t kCalibrationPage = 0x0807F800UL;

// Four inactive/active pages provide A/B storage for MAP 1 and MAP 2.
constexpr uint32_t kMap1A = 0x0807D000UL;
constexpr uint32_t kMap1B = 0x0807D800UL;
constexpr uint32_t kMap2A = 0x0807E000UL;
constexpr uint32_t kMap2B = 0x0807E800UL;

constexpr uint32_t kReservedGapPage = 0x0807F000UL;

constexpr bool IsPageAligned(uint32_t address) {
  return (address % kFlashPageSize) == 0U;
}

constexpr bool InFlash(uint32_t address, uint32_t length) {
  return address >= kFlashOrigin && address < kFlashEnd &&
         length <= kFlashEnd - address;
}

static_assert(IsPageAligned(kMap1A) && IsPageAligned(kMap1B) &&
                  IsPageAligned(kMap2A) && IsPageAligned(kMap2B) &&
                  IsPageAligned(kReservedGapPage) &&
                  IsPageAligned(kCalibrationPage),
              "MAP/calibration storage must start on Flash page boundaries");
static_assert(InFlash(kMap1A, kFlashPageSize) &&
                  InFlash(kMap1B, kFlashPageSize) &&
                  InFlash(kMap2A, kFlashPageSize) &&
                  InFlash(kMap2B, kFlashPageSize) &&
                  InFlash(kReservedGapPage, kFlashPageSize) &&
                  InFlash(kCalibrationPage, kFlashPageSize),
              "MAP/calibration storage must remain inside STM32 Flash");
static_assert(kMap1A + kFlashPageSize <= kMap1B &&
                  kMap1B + kFlashPageSize <= kMap2A &&
                  kMap2A + kFlashPageSize <= kMap2B &&
                  kMap2B + kFlashPageSize <= kReservedGapPage &&
                  kReservedGapPage + kFlashPageSize <= kCalibrationPage,
              "MAP A/B pages must not overlap one another or calibration");
}

#endif  // STM32_MAP_FLASH_LAYOUT_H
