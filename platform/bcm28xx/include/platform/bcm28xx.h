/*
 * Copyright (c) 2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */
#pragma once

#define SDRAM_BASE 0
/* Note: BCM2836/BCM2837 use different peripheral base than BCM2835 */
#define BCM_PERIPH_BASE_PHYS    (0x3f000000U)
#define BCM_PERIPH_SIZE         (0x01100000U)

#if BCM2836
#define BCM_PERIPH_BASE_VIRT    (0xe0000000U)
#elif BCM2837
#define BCM_PERIPH_BASE_VIRT    (0xffffffffc0000000ULL)
#define MEMORY_APERTURE_SIZE    (1024 * 1024 * 1024)
#else
#error Unknown BCM28XX Variant
#endif

#define BCM_SDRAM_BUS_ADDR_BASE_NO_L2 0xc0000000
#define BCM_SDRAM_BUS_ADDR_BASE (BCM_SDRAM_BUS_ADDR_BASE_NO_L2)

/* pointer to 'local' peripherals at 0x40000000 */
#define BCM_LOCAL_PERIPH_BASE_VIRT (BCM_PERIPH_BASE_VIRT + 0x01000000)

#define IC0_BASE                (BCM_PERIPH_BASE_VIRT + 0x2000)
#define ST_BASE                 (BCM_PERIPH_BASE_VIRT + 0x3000)
#define MPHI_BASE               (BCM_PERIPH_BASE_VIRT + 0x6000)
#define DMA_BASE                (BCM_PERIPH_BASE_VIRT + 0x7000)
#define ARM_BASE                (BCM_PERIPH_BASE_VIRT + 0xB000)
#define PM_BASE                 (BCM_PERIPH_BASE_VIRT + 0x100000)
#define PCM_CLOCK_BASE          (BCM_PERIPH_BASE_VIRT + 0x101098)
#define RNG_BASE                (BCM_PERIPH_BASE_VIRT + 0x104000)
#define GPIO_BASE               (BCM_PERIPH_BASE_VIRT + 0x200000)

// all PL011 uarts, 2-5 only exist on BCM2828/pi4/VC6
// all share the same INTERRUPT_VC_UART
#define UART0_BASE              (BCM_PERIPH_BASE_VIRT + 0x201000)
#define UART2_BASE              (BCM_PERIPH_BASE_VIRT + 0x201400)
#define UART3_BASE              (BCM_PERIPH_BASE_VIRT + 0x201600)
#define UART4_BASE              (BCM_PERIPH_BASE_VIRT + 0x201800)
#define UART5_BASE              (BCM_PERIPH_BASE_VIRT + 0x201a00)

#define MMCI0_BASE              (BCM_PERIPH_BASE_VIRT + 0x202000)
#define I2S_BASE                (BCM_PERIPH_BASE_VIRT + 0x203000)
#define SPI0_BASE               (BCM_PERIPH_BASE_VIRT + 0x204000)
#define BSC0_BASE               (BCM_PERIPH_BASE_VIRT + 0x205000)
#define AUX_BASE                (BCM_PERIPH_BASE_VIRT + 0x215000)
#define MINIUART_BASE           (BCM_PERIPH_BASE_VIRT + 0x215040)
#define EMMC_BASE               (BCM_PERIPH_BASE_VIRT + 0x300000)
#define SMI_BASE                (BCM_PERIPH_BASE_VIRT + 0x600000)
#define BSC1_BASE               (BCM_PERIPH_BASE_VIRT + 0x804000)
#define USB_BASE                (BCM_PERIPH_BASE_VIRT + 0x980000)
#define MCORE_BASE              (BCM_PERIPH_BASE_VIRT + 0x0000)

#define ARMCTRL_BASE            (ARM_BASE + 0x000)
#define ARMCTRL_INTC_BASE       (ARM_BASE + 0x200)
#define ARMCTRL_TIMER0_1_BASE   (ARM_BASE + 0x400)
#define ARMCTRL_0_SBM_BASE      (ARM_BASE + 0x800)

/* Videocore (GPU) mailbox registers for core0 */
#define ARM0_MAILBOX_BASE       (ARM_BASE + 0x0880)
#define ARM0_MAILBOX_READ       (ARM0_MAILBOX_BASE + 0x00)
#define ARM0_MAILBOX_PEEK       (ARM0_MAILBOX_BASE + 0x10)
#define ARM0_MAILBOX_CONFIG     (ARM0_MAILBOX_BASE + 0x1C)
#define ARM0_MAILBOX_STATUS     (ARM0_MAILBOX_BASE + 0x18)
#define ARM0_MAILBOX_WRITE      (ARM0_MAILBOX_BASE + 0x20)

#define ARM_LOCAL_BASE          (BCM_LOCAL_PERIPH_BASE_VIRT)

/* interrupts */
#define ARM_IRQ1_BASE                  0
#define INTERRUPT_TIMER0               (ARM_IRQ1_BASE + 0)
#define INTERRUPT_TIMER1               (ARM_IRQ1_BASE + 1)
#define INTERRUPT_TIMER2               (ARM_IRQ1_BASE + 2)
#define INTERRUPT_TIMER3               (ARM_IRQ1_BASE + 3)
#define INTERRUPT_CODEC0               (ARM_IRQ1_BASE + 4)
#define INTERRUPT_CODEC1               (ARM_IRQ1_BASE + 5)
#define INTERRUPT_CODEC2               (ARM_IRQ1_BASE + 6)
#define INTERRUPT_VC_JPEG              (ARM_IRQ1_BASE + 7)
#define INTERRUPT_ISP                  (ARM_IRQ1_BASE + 8)
#define INTERRUPT_VC_USB               (ARM_IRQ1_BASE + 9)
#define INTERRUPT_VC_3D                (ARM_IRQ1_BASE + 10)
#define INTERRUPT_TRANSPOSER           (ARM_IRQ1_BASE + 11)
#define INTERRUPT_MULTICORESYNC0       (ARM_IRQ1_BASE + 12)
#define INTERRUPT_MULTICORESYNC1       (ARM_IRQ1_BASE + 13)
#define INTERRUPT_MULTICORESYNC2       (ARM_IRQ1_BASE + 14)
#define INTERRUPT_MULTICORESYNC3       (ARM_IRQ1_BASE + 15)
#define INTERRUPT_DMA0                 (ARM_IRQ1_BASE + 16)
#define INTERRUPT_DMA1                 (ARM_IRQ1_BASE + 17)
#define INTERRUPT_VC_DMA2              (ARM_IRQ1_BASE + 18)
#define INTERRUPT_VC_DMA3              (ARM_IRQ1_BASE + 19)
#define INTERRUPT_DMA4                 (ARM_IRQ1_BASE + 20)
#define INTERRUPT_DMA5                 (ARM_IRQ1_BASE + 21)
#define INTERRUPT_DMA6                 (ARM_IRQ1_BASE + 22)
#define INTERRUPT_DMA7                 (ARM_IRQ1_BASE + 23)
#define INTERRUPT_DMA8                 (ARM_IRQ1_BASE + 24)
#define INTERRUPT_DMA9                 (ARM_IRQ1_BASE + 25)
#define INTERRUPT_DMA10                (ARM_IRQ1_BASE + 26)
#define INTERRUPT_DMA11                (ARM_IRQ1_BASE + 27)
#define INTERRUPT_DMA12                (ARM_IRQ1_BASE + 28)
#define INTERRUPT_AUX                  (ARM_IRQ1_BASE + 29)
#define INTERRUPT_ARM                  (ARM_IRQ1_BASE + 30)
#define INTERRUPT_VPUDMA               (ARM_IRQ1_BASE + 31)

#define ARM_IRQ2_BASE                  32
#define INTERRUPT_HOSTPORT             (ARM_IRQ2_BASE + 0)
#define INTERRUPT_VIDEOSCALER          (ARM_IRQ2_BASE + 1)
#define INTERRUPT_CCP2TX               (ARM_IRQ2_BASE + 2)
#define INTERRUPT_SDC                  (ARM_IRQ2_BASE + 3)
#define INTERRUPT_DSI0                 (ARM_IRQ2_BASE + 4)
#define INTERRUPT_AVE                  (ARM_IRQ2_BASE + 5)
#define INTERRUPT_CAM0                 (ARM_IRQ2_BASE + 6)
#define INTERRUPT_CAM1                 (ARM_IRQ2_BASE + 7)
#define INTERRUPT_HDMI0                (ARM_IRQ2_BASE + 8)
#define INTERRUPT_HDMI1                (ARM_IRQ2_BASE + 9)
#define INTERRUPT_PIXELVALVE1          (ARM_IRQ2_BASE + 10)
#define INTERRUPT_I2CSPISLV            (ARM_IRQ2_BASE + 11)
#define INTERRUPT_DSI1                 (ARM_IRQ2_BASE + 12)
#define INTERRUPT_PWA0                 (ARM_IRQ2_BASE + 13)
#define INTERRUPT_PWA1                 (ARM_IRQ2_BASE + 14)
#define INTERRUPT_CPR                  (ARM_IRQ2_BASE + 15)
#define INTERRUPT_SMI                  (ARM_IRQ2_BASE + 16)
#define INTERRUPT_GPIO0                (ARM_IRQ2_BASE + 17)
#define INTERRUPT_GPIO1                (ARM_IRQ2_BASE + 18)
#define INTERRUPT_GPIO2                (ARM_IRQ2_BASE + 19)
#define INTERRUPT_GPIO3                (ARM_IRQ2_BASE + 20)
#define INTERRUPT_VC_I2C               (ARM_IRQ2_BASE + 21)
#define INTERRUPT_VC_SPI               (ARM_IRQ2_BASE + 22)
#define INTERRUPT_VC_I2SPCM            (ARM_IRQ2_BASE + 23)
#define INTERRUPT_VC_SDIO              (ARM_IRQ2_BASE + 24)
#define INTERRUPT_VC_UART              (ARM_IRQ2_BASE + 25)
#define INTERRUPT_SLIMBUS              (ARM_IRQ2_BASE + 26)
#define INTERRUPT_VEC                  (ARM_IRQ2_BASE + 27)
#define INTERRUPT_CPG                  (ARM_IRQ2_BASE + 28)
#define INTERRUPT_RNG                  (ARM_IRQ2_BASE + 29)
#define INTERRUPT_VC_ARASANSDIO        (ARM_IRQ2_BASE + 30)
#define INTERRUPT_AVSPMON              (ARM_IRQ2_BASE + 31)

/* ARM interrupts, which are mostly mirrored from bank 1 and 2 */
#define ARM_IRQ0_BASE                  64
#define INTERRUPT_ARM_TIMER            (ARM_IRQ0_BASE + 0)
#define INTERRUPT_ARM_MAILBOX          (ARM_IRQ0_BASE + 1)
#define INTERRUPT_ARM_DOORBELL_0       (ARM_IRQ0_BASE + 2)
#define INTERRUPT_ARM_DOORBELL_1       (ARM_IRQ0_BASE + 3)
#define INTERRUPT_VPU0_HALTED          (ARM_IRQ0_BASE + 4)
#define INTERRUPT_VPU1_HALTED          (ARM_IRQ0_BASE + 5)
#define INTERRUPT_ILLEGAL_TYPE0        (ARM_IRQ0_BASE + 6)
#define INTERRUPT_ILLEGAL_TYPE1        (ARM_IRQ0_BASE + 7)
#define INTERRUPT_PENDING1             (ARM_IRQ0_BASE + 8)
#define INTERRUPT_PENDING2             (ARM_IRQ0_BASE + 9)
#define INTERRUPT_JPEG                 (ARM_IRQ0_BASE + 10)
#define INTERRUPT_USB                  (ARM_IRQ0_BASE + 11)
#define INTERRUPT_3D                   (ARM_IRQ0_BASE + 12)
#define INTERRUPT_DMA2                 (ARM_IRQ0_BASE + 13)
#define INTERRUPT_DMA3                 (ARM_IRQ0_BASE + 14)
#define INTERRUPT_I2C                  (ARM_IRQ0_BASE + 15)
#define INTERRUPT_SPI                  (ARM_IRQ0_BASE + 16)
#define INTERRUPT_I2SPCM               (ARM_IRQ0_BASE + 17)
#define INTERRUPT_SDIO                 (ARM_IRQ0_BASE + 18)
#define INTERRUPT_UART                 (ARM_IRQ0_BASE + 19)
#define INTERRUPT_ARASANSDIO           (ARM_IRQ0_BASE + 20)

#define ARM_IRQ_LOCAL_BASE             96
#define INTERRUPT_ARM_LOCAL_CNTPSIRQ   (ARM_IRQ_LOCAL_BASE + 0)
#define INTERRUPT_ARM_LOCAL_CNTPNSIRQ  (ARM_IRQ_LOCAL_BASE + 1)
#define INTERRUPT_ARM_LOCAL_CNTHPIRQ   (ARM_IRQ_LOCAL_BASE + 2)
#define INTERRUPT_ARM_LOCAL_CNTVIRQ    (ARM_IRQ_LOCAL_BASE + 3)
#define INTERRUPT_ARM_LOCAL_MAILBOX0   (ARM_IRQ_LOCAL_BASE + 4)
#define INTERRUPT_ARM_LOCAL_MAILBOX1   (ARM_IRQ_LOCAL_BASE + 5)
#define INTERRUPT_ARM_LOCAL_MAILBOX2   (ARM_IRQ_LOCAL_BASE + 6)
#define INTERRUPT_ARM_LOCAL_MAILBOX3   (ARM_IRQ_LOCAL_BASE + 7)
#define INTERRUPT_ARM_LOCAL_GPU_FAST   (ARM_IRQ_LOCAL_BASE + 8)
#define INTERRUPT_ARM_LOCAL_PMU_FAST   (ARM_IRQ_LOCAL_BASE + 9)
#define INTERRUPT_ARM_LOCAL_ZERO       (ARM_IRQ_LOCAL_BASE + 10)
#define INTERRUPT_ARM_LOCAL_TIMER      (ARM_IRQ_LOCAL_BASE + 11)

#define MAX_INT INTERRUPT_ARM_LOCAL_TIMER

/* GPIO */

#define GPIO_IN 0
#define GPIO_OUT 1
#define GPIO_ALT5 2
#define GPIO_ALT4 3
#define GPIO_ALT0 4
#define GPIO_ALT1 5
#define GPIO_ALT2 6
#define GPIO_ALT3 7

#define GPIO_GPFSEL0   (GPIO_BASE + 0x00)
#define GPIO_GPFSEL1   (GPIO_BASE + 0x04)
#define GPIO_GPFSEL2   (GPIO_BASE + 0x08)
#define GPIO_GPFSEL3   (GPIO_BASE + 0x0C)
#define GPIO_GPFSEL4   (GPIO_BASE + 0x10)
#define GPIO_GPFSEL5   (GPIO_BASE + 0x14)
#define GPIO_GPSET0    (GPIO_BASE + 0x1C)
#define GPIO_GPSET1    (GPIO_BASE + 0x20)
#define GPIO_GPCLR0    (GPIO_BASE + 0x28)
#define GPIO_GPCLR1    (GPIO_BASE + 0x2C)
#define GPIO_GPLEV0    (GPIO_BASE + 0x34)
#define GPIO_GPLEV1    (GPIO_BASE + 0x38)
#define GPIO_GPEDS0    (GPIO_BASE + 0x40)
#define GPIO_GPEDS1    (GPIO_BASE + 0x44)
#define GPIO_GPREN0    (GPIO_BASE + 0x4C)
#define GPIO_GPREN1    (GPIO_BASE + 0x50)
#define GPIO_GPFEN0    (GPIO_BASE + 0x58)
#define GPIO_GPFEN1    (GPIO_BASE + 0x5C)
#define GPIO_GPHEN0    (GPIO_BASE + 0x64)
#define GPIO_GPHEN1    (GPIO_BASE + 0x68)
#define GPIO_GPLEN0    (GPIO_BASE + 0x70)
#define GPIO_GPLEN1    (GPIO_BASE + 0x74)
#define GPIO_GPAREN0   (GPIO_BASE + 0x7C)
#define GPIO_GPAREN1   (GPIO_BASE + 0x80)
#define GPIO_GPAFEN0   (GPIO_BASE + 0x88)
#define GPIO_GPAFEN1   (GPIO_BASE + 0x8C)
#define GPIO_GPPUD     (GPIO_BASE + 0x94)
#define GPIO_GPPUDCLK0 (GPIO_BASE + 0x98)
#define GPIO_GPPUDCLK1 (GPIO_BASE + 0x9C)
