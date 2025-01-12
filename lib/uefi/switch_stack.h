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

#include <lk/compiler.h>
#include <stddef.h>

__BEGIN_CDECLS

size_t call_with_stack_asm(void *stack, const void *function, void *param1,
                       void *param2, void *param3, void *param4);

__END_CDECLS

#ifdef __cplusplus
template <typename Function, typename P1, typename P2, typename P3, typename P4>
size_t call_with_stack(void *stack, Function &&fp, P1 &&param1, P2 &&param2,
                       P3 &&param3, P4 &&param4) {
  return call_with_stack_asm(
      stack, reinterpret_cast<const void *>(fp),
      reinterpret_cast<void *>(param1), reinterpret_cast<void *>(param2),
      reinterpret_cast<void *>(param3), reinterpret_cast<void *>(param4));
}

template <typename Function, typename P1, typename P2>
size_t call_with_stack(void *stack, Function &&fp, P1 &&param1, P2 &&param2) {
  return call_with_stack_asm(
      stack, reinterpret_cast<const void *>(fp),
      reinterpret_cast<void *>(param1), reinterpret_cast<void *>(param2), 0, 0);
}
#endif