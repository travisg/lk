#pragma once

void pl011_uart_init(int irq, int nr, uintptr_t base);
void pl011_uart_init_early(int nr, uintptr_t base);
