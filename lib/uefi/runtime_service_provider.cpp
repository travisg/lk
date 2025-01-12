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

#include "runtime_service_provider.h"
#include "types.h"

#include <stdio.h>
#include <string.h>

namespace {

constexpr auto &&kSecureBoot = L"SecureBoot";

EFI_STATUS GetVariable(char16_t *VariableName, EfiGuid *VendorGuid,
                       uint32_t *Attributes, size_t *DataSize, void *Data) {
  char buffer[512];
  size_t i = 0;
  while (VariableName[i] && i < sizeof(buffer)) {
    size_t j = 0;
    for (j = 0; j < sizeof(buffer) - 1 && VariableName[i + j]; j++) {
      buffer[j] = VariableName[i + j];
    }
    i += j;
  }
  buffer[i] = 0;
  if (strcmp(buffer, "SecureBoot") == 0 || strcmp(buffer, "SetupMode") == 0) {
    if (DataSize) {
      *DataSize = 1;
    }
    if (Data) {
      memset(Data, 0, 1);
    }
    return SUCCESS;
  }

  printf("%s(%s) is unsupported\n", __FUNCTION__, buffer);
  return UNSUPPORTED;
}

EFI_STATUS SetVirtualAddressMap(size_t MemoryMapSize, size_t DescriptorSize,
                                uint32_t DescriptorVersion,
                                EfiMemoryDescriptor *VirtualMap) {
  printf("%s is unsupported\n", __FUNCTION__);
  return UNSUPPORTED;
}

EFI_STATUS SetVariable(char16_t *VariableName, EfiGuid *VendorGuid,
                       uint32_t Attributes, size_t DataSize, void *Data) {
  printf("%s is unsupported\n", __FUNCTION__);
  return UNSUPPORTED;
}

} // namespace

void setup_runtime_service_table(EfiRuntimeService *service) {
  service->GetVariable = GetVariable;
  service->SetVariable = SetVariable;
  service->SetVirtualAddressMap = SetVirtualAddressMap;
}
