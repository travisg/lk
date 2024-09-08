/*
 * Copyright (c) 2024 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#if M68K_MMU

void m68k_mmu_early_init(void);
void m68k_mmu_init(void);

#endif // M68K_MMU