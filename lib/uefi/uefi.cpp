/*
 * Copyright (C) 2024 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "boot_service.h"
#include "boot_service_provider.h"
#include "defer.h"
#include "pe.h"

#include <lib/bio.h>
#include <lib/heap.h>
#include <lk/console_cmd.h>
#include <lk/debug.h>
#include <lk/err.h>
#include <lk/trace.h>
#include <platform.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include "configuration_table.h"
#include "protocols/simple_text_output_protocol.h"
#include "runtime_service.h"
#include "runtime_service_provider.h"
#include "switch_stack.h"
#include "system_table.h"
#include "text_protocol.h"

namespace {

constexpr auto EFI_SYSTEM_TABLE_SIGNATURE =
    static_cast<u64>(0x5453595320494249ULL);

// ASCII "PE\x0\x0"

using EfiEntry = int (*)(void *, struct EfiSystemTable *);

template <typename T> void fill(T *data, size_t skip, uint8_t begin = 0) {
  auto ptr = reinterpret_cast<char *>(data);
  for (size_t i = 0; i < sizeof(T); i++) {
    if (i < skip) {
      continue;
    }
    ptr[i] = begin++;
  }
}

constexpr size_t BIT26 = 1 << 26;
constexpr size_t BIT11 = 1 << 11;
constexpr size_t BIT10 = 1 << 10;

/**
  Pass in a pointer to an ARM MOVT or MOVW immediate instruciton and
  return the immediate data encoded in the instruction.

  @param  Instruction   Pointer to ARM MOVT or MOVW immediate instruction

  @return Immediate address encoded in the instruction

**/
uint16_t ThumbMovtImmediateAddress(const uint16_t *Instruction) {
  uint32_t Movt;
  uint16_t Address;

  // Thumb2 is two 16-bit instructions working together. Not a single 32-bit
  // instruction Example MOVT R0, #0 is 0x0000f2c0 or 0xf2c0 0x0000
  Movt = (*Instruction << 16) | (*(Instruction + 1));

  // imm16 = imm4:i:imm3:imm8
  //         imm4 -> Bit19:Bit16
  //         i    -> Bit26
  //         imm3 -> Bit14:Bit12
  //         imm8 -> Bit7:Bit0
  Address = (uint16_t)(Movt & 0x000000ff);         // imm8
  Address |= (uint16_t)((Movt >> 4) & 0x0000f700); // imm4 imm3
  Address |= (((Movt & BIT26) != 0) ? BIT11 : 0);  // i
  return Address;
}

/**
  Pass in a pointer to an ARM MOVW/MOVT instruciton pair and
  return the immediate data encoded in the two` instruction.

  @param  Instructions  Pointer to ARM MOVW/MOVT insturction pair

  @return Immediate address encoded in the instructions

**/
uint32_t ThumbMovwMovtImmediateAddress(uint16_t *Instructions) {
  uint16_t *Word;
  uint16_t *Top;

  Word = Instructions; // MOVW
  Top = Word + 2;      // MOVT

  return (ThumbMovtImmediateAddress(Top) << 16) +
         ThumbMovtImmediateAddress(Word);
}

/**
  Update an ARM MOVT or MOVW immediate instruction immediate data.

  @param  Instruction   Pointer to ARM MOVT or MOVW immediate instruction
  @param  Address       New addres to patch into the instruction
**/
void ThumbMovtImmediatePatch(uint16_t *Instruction, uint16_t Address) {
  uint16_t Patch;

  // First 16-bit chunk of instruciton
  Patch = ((Address >> 12) & 0x000f);              // imm4
  Patch |= (((Address & BIT11) != 0) ? BIT10 : 0); // i
  // Mask out instruction bits and or in address
  *(Instruction) = (*Instruction & ~0x040f) | Patch;

  // Second 16-bit chunk of instruction
  Patch = Address & 0x000000ff;           // imm8
  Patch |= ((Address << 4) & 0x00007000); // imm3
  // Mask out instruction bits and or in address
  Instruction++;
  *Instruction = (*Instruction & ~0x70ff) | Patch;
}

/**
  Update an ARM MOVW/MOVT immediate instruction instruction pair.

  @param  Instructions  Pointer to ARM MOVW/MOVT instruction pair
  @param  Address       New addres to patch into the instructions
**/
void ThumbMovwMovtImmediatePatch(uint16_t *Instructions, uint32_t Address) {
  uint16_t *Word;
  uint16_t *Top;

  Word = Instructions; // MOVW
  Top = Word + 2;      // MOVT

  ThumbMovtImmediatePatch(Word, (uint16_t)(Address & 0xffff));
  ThumbMovtImmediatePatch(Top, (uint16_t)(Address >> 16));
}

int relocate_image(char *image) {
  const auto dos_header = reinterpret_cast<IMAGE_DOS_HEADER *>(image);
  const auto pe_header = dos_header->GetPEHeader();
  const auto optional_header = &pe_header->OptionalHeader;
  const auto reloc_directory =
      optional_header->DataDirectory[IMAGE_DIRECTORY_ENTRY_BASERELOC];
  if (reloc_directory.Size == 0) {
    printf("Relocation section empty\n");
    return 0;
  }
  auto RelocBase = reinterpret_cast<EFI_IMAGE_BASE_RELOCATION *>(
      image + reloc_directory.VirtualAddress);
  const auto RelocBaseEnd = reinterpret_cast<EFI_IMAGE_BASE_RELOCATION *>(
      (char *)RelocBase + reloc_directory.Size);
  const auto Adjust =
      reinterpret_cast<size_t>(image - optional_header->ImageBase);
  //
  // Run this relocation record
  //
  while (RelocBase < RelocBaseEnd) {
    auto Reloc =
        (uint16_t *)((char *)RelocBase + sizeof(EFI_IMAGE_BASE_RELOCATION));
    auto RelocEnd = reinterpret_cast<uint16_t *>((char *)RelocBase +
                                                 RelocBase->SizeOfBlock);
    if (RelocBase->SizeOfBlock == 0) {
      printf("Found relocation block of size 0, this is wrong\n");
      return -1;
    }
    while (Reloc < RelocEnd) {
      auto Fixup = image + RelocBase->VirtualAddress + (*Reloc & 0xFFF);
      if (Fixup == nullptr) {
        return 0;
      }

      auto Fixup16 = reinterpret_cast<uint16_t *>(Fixup);
      auto Fixup32 = reinterpret_cast<uint32_t *>(Fixup);
      auto Fixup64 = reinterpret_cast<uint64_t *>(Fixup);
      uint32_t FixupVal = 0;
      switch ((*Reloc) >> 12) {
      case EFI_IMAGE_REL_BASED_ABSOLUTE:
        break;

      case EFI_IMAGE_REL_BASED_HIGH:
        *Fixup16 = (uint16_t)(*Fixup16 + ((uint16_t)((uint32_t)Adjust >> 16)));

        break;

      case EFI_IMAGE_REL_BASED_LOW:
        *Fixup16 = (uint16_t)(*Fixup16 + ((uint16_t)Adjust & 0xffff));

        break;

      case EFI_IMAGE_REL_BASED_HIGHLOW:
        *Fixup32 = *Fixup32 + (uint32_t)Adjust;
        break;

      case EFI_IMAGE_REL_BASED_DIR64:
        *Fixup64 = *Fixup64 + (uint64_t)Adjust;
        break;

      case EFI_IMAGE_REL_BASED_ARM_MOV32T:
        FixupVal = ThumbMovwMovtImmediateAddress(Fixup16) + (uint32_t)Adjust;
        ThumbMovwMovtImmediatePatch(Fixup16, FixupVal);

        break;

      case EFI_IMAGE_REL_BASED_ARM_MOV32A:
        printf("Unsupported relocation type: EFI_IMAGE_REL_BASED_ARM_MOV32A\n");
        // break omitted - ARM instruction encoding not implemented
        break;

      default:
        printf("Unsupported relocation type: %d\n", (*Reloc) >> 12);
        return -1;
      }

      //
      // Next relocation record
      //
      Reloc += 1;
    }
    RelocBase = reinterpret_cast<EFI_IMAGE_BASE_RELOCATION *>(RelocEnd);
  }
  optional_header->ImageBase = reinterpret_cast<size_t>(image);
  return 0;
}

int load_sections_and_execute(bdev_t *dev,
                              const IMAGE_NT_HEADERS64 *pe_header) {
  const auto file_header = &pe_header->FileHeader;
  const auto optional_header = &pe_header->OptionalHeader;
  const auto sections = file_header->NumberOfSections;
  const auto section_header = reinterpret_cast<const IMAGE_SECTION_HEADER *>(
      reinterpret_cast<const char *>(pe_header) + sizeof(IMAGE_FILE_HEADER) +
      file_header->SizeOfOptionalHeader);
  if (sections <= 0) {
    printf("This PE file does not have any sections, unsupported.\n");
    return -8;
  }
  for (size_t i = 0; i < sections; i++) {
    if (section_header[i].NumberOfRelocations != 0) {
      printf("Section %s requires relocation, which is not supported.\n",
             section_header[i].Name);
      return -6;
    }
  }
  const auto &last_section = section_header[sections - 1];
  const auto virtual_size =
      last_section.VirtualAddress + last_section.Misc.VirtualSize;
  const auto image_base = reinterpret_cast<char *>(
      alloc_page(reinterpret_cast<void *>(optional_header->ImageBase),
                 virtual_size, 21 /* Kernel requires 2MB alignment */));
  if (image_base == nullptr) {
    return -7;
  }
  memset(image_base, 0, virtual_size);
  bio_read(dev, image_base, 0, section_header[0].PointerToRawData);

  for (size_t i = 0; i < sections; i++) {
    const auto &section = section_header[i];
    bio_read(dev, image_base + section.VirtualAddress, section.PointerToRawData,
             section.SizeOfRawData);
  }
  printf("Relocating image from 0x%llx to %p\n", optional_header->ImageBase,
         image_base);
  relocate_image(image_base);
  auto entry = reinterpret_cast<int (*)(void *, void *)>(
      image_base + optional_header->AddressOfEntryPoint);
  printf("Entry function located at %p\n", entry);

  EfiSystemTable &table = *static_cast<EfiSystemTable *>(alloc_page(PAGE_SIZE));
  EfiBootService boot_service{};
  EfiRuntimeService runtime_service{};
  fill(&runtime_service, 0);
  fill(&boot_service, 0);
  setup_runtime_service_table(&runtime_service);
  setup_boot_service_table(&boot_service);
  table.runtime_service = &runtime_service;
  table.boot_services = &boot_service;
  table.header.signature = EFI_SYSTEM_TABLE_SIGNATURE;
  table.header.revision = 2 << 16;
  EfiSimpleTextOutputProtocol console_out = get_text_output_protocol();
  table.con_out = &console_out;
  table.configuration_table =
      reinterpret_cast<EfiConfigurationTable *>(alloc_page(PAGE_SIZE));
  setup_configuration_table(&table);

  constexpr size_t kStackSize = 8 * 1024ul * 1024;
  auto stack = reinterpret_cast<char *>(alloc_page(kStackSize, 23));
  memset(stack, 0, kStackSize);
  printf("Calling kernel with stack [%p, %p]\n", stack,
         stack + kStackSize - 1);
  return call_with_stack(stack + kStackSize, entry, image_base, &table);
}

int load_pe_file(const char *blkdev) {
  bdev_t *dev = bio_open(blkdev);
  if (!dev) {
    printf("error opening block device %s\n", blkdev);
    return -1;
  }
  DEFER {
    bio_close(dev);
    dev = nullptr;
  };
  constexpr size_t kBlocKSize = 4096;

  lk_time_t t = current_time();
  uint8_t *address = static_cast<uint8_t *>(malloc(kBlocKSize));
  ssize_t err = bio_read(dev, static_cast<void *>(address), 0, kBlocKSize);
  t = current_time() - t;
  dprintf(INFO, "bio_read returns %d, took %u msecs (%d bytes/sec)\n", (int)err,
          (uint)t, (uint32_t)((uint64_t)err * 1000 / t));

  const auto dos_header = reinterpret_cast<const IMAGE_DOS_HEADER *>(address);
  if (!dos_header->CheckMagic()) {
    printf("DOS Magic check failed %x\n", dos_header->e_magic);
    return -2;
  }
  if (dos_header->e_lfanew > kBlocKSize - sizeof(IMAGE_FILE_HEADER)) {
    printf(
        "Invalid PE header offset %d exceeds maximum read size of %zu - %zu\n",
        dos_header->e_lfanew, kBlocKSize, sizeof(IMAGE_FILE_HEADER));
    return -3;
  }
  const auto pe_header = dos_header->GetPEHeader();
  const auto file_header = &pe_header->FileHeader;
  if (LE32(file_header->Signature) != kPEHeader) {
    printf("COFF Magic check failed %x\n", LE32(file_header->Signature));
    return -4;
  }
  printf("PE header machine type: %x\n",
         static_cast<int>(file_header->Machine));
  if (file_header->SizeOfOptionalHeader > sizeof(IMAGE_OPTIONAL_HEADER64) ||
      file_header->SizeOfOptionalHeader <
          sizeof(IMAGE_OPTIONAL_HEADER64) -
              sizeof(IMAGE_OPTIONAL_HEADER64::DataDirectory)) {
    printf("Unexpected size of optional header %d, expected %zu\n",
           file_header->SizeOfOptionalHeader, sizeof(IMAGE_OPTIONAL_HEADER64));
    return -5;
  }
  const auto optional_header = &pe_header->OptionalHeader;
  if (optional_header->Subsystem != SubsystemType::EFIApplication) {
    printf("Unsupported Subsystem type: %d %s\n", optional_header->Subsystem,
           ToString(optional_header->Subsystem));
  }
  printf("Valid UEFI application found.\n");
  auto ret = load_sections_and_execute(dev, pe_header);
  printf("UEFI Application return code: %d\n", ret);
  return ret;
}

int cmd_uefi_load(int argc, const console_cmd_args *argv) {
  if (argc != 2) {
    printf("Usage: %s <name of block device to load from>\n", argv[0].str);
    return 1;
  }
  load_pe_file(argv[1].str);
  return 0;
}

STATIC_COMMAND_START
STATIC_COMMAND("uefi_load", "load UEFI application and run it", &cmd_uefi_load)
STATIC_COMMAND_END(uefi);

} // namespace
