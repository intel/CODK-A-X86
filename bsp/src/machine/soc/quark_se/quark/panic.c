/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file panic.c
 *
 * Generic services for panic management
 *
 */

#include "panic_qrk_se.h"
#include "machine/soc/quark_se/quark/reboot.h"
#include <string.h>
#include "os/os.h"
#include "infra/version.h"
#include "scss_registers.h"

#ifdef CONFIG_IPC
#include "infra/ipc.h"
#endif
#ifdef CONFIG_IPC_UART
#include "infra/ipc_uart.h"
#endif
#ifdef CONFIG_QUARK_SE_PANIC_DEBUG
#include <misc/printk.h>
#include "quark_se_mapping.h"
#define ARC_PANIC_DUMP_ADDR  (ARC_RAM_START_ADDR+ARC_RAM_SIZE) /*!< ARC panic dump location (for debug purposes) */
#endif

/**
 * \brief Generic panic error codes for quark_se platform
 */
#define PANIC_UNKNOWN    -1 /*!< Error code for unknown core panic */
#define PANIC_ARC_CORE    1 /*!< Error code for panic triggered by ARC core panic */
#define PANIC_QRK_CORE    2 /*!< Error code for panic triggered by QRK core panic */
#define PANIC_NORDIC_CORE 3 /*!< Error code for panic triggered by nordic core panic */

#define PANIC_NOTIFY_TIMEOUT 2000 /*!< panic notification timeout (in ms) */

#define PANIC_UART_IPC_CHANNEL 1 /*!< uart ipc channel used to handle panic propagation */

#define PANIC_QRK_STACK_SIZE 160 /*!< QRK max stack dump size (in bytes) */

/**
 * \brief Panic status of quark_se cores
 */
struct panic_core_status {
    unsigned is_panic : 1;
};

static volatile struct panic_core_status panic_core_status[QRK_SE_CORE_COUNT] = {{0}};

extern char __ram_phys_end[];
const uint32_t _x86_dump_location = ((uint32_t)__ram_phys_end)
                                    - sizeof(struct panic_data_footer);

#ifdef CONFIG_QUARK_SE_PANIC_DEBUG
/**
 * \brief Flush ARC and QRK panic dumps over UART for debug purposes
 *
 * This function flushes x86/arcv2 panic dump (registers and stack) over uart
 *
 * \param footer pointer to panic dump footer location in RAM
 */
static void panic_handler(struct panic_data_footer *footer)
{
    unsigned int i;
    // Check magic key
    if (footer->magic != PANIC_DATA_MAGIC) {
        printk("\nWrong magic key detected (0x%x) on 0x%x\n", footer->magic, ((uint32_t)&footer->magic)+4);
        return;
    }

    printk("\nPANIC for %d (dump located on 0x%x)\n", footer->arch, ((uint32_t)&footer->magic)+4);

    switch (footer->arch) {
    case QRK_CORE:
        {
        // Get ptr to arch specific dump data
        struct x86_panic_arch_data *qrk_info = (struct x86_panic_arch_data*)((uint8_t*)footer - sizeof(struct x86_panic_arch_data));

        printk("QRK BUILD CKSUM: %X \n", footer->build_cksum);
        printk("QRK BUILD VERSION: %X \n", footer->struct_version);
        printk("QRK PANIC (%d, 0x%x) - EIP=0x%X - ESP=0x%X\n",
                qrk_info->type, qrk_info->error, qrk_info->eip, qrk_info->esp);

        printk("\nRegisters: \n\n");
        printk("cr2:   %x\n", qrk_info->cr2);
        printk("edi:   %x\n", qrk_info->edi);
        printk("esi:   %x\n", qrk_info->esi);
        printk("ebx:   %x\n", qrk_info->ebx);
        printk("ebp:   %x\n", qrk_info->ebp);
        printk("edx:   %x\n", qrk_info->edx);
        printk("ecx:   %x\n", qrk_info->ecx);
        printk("eax:   %x\n", qrk_info->eax);
        printk("error: %x\n", qrk_info->error);
        printk("cs:    %x\n", qrk_info->cs);
        printk("flags: %x\n", qrk_info->flags);
        printk("ss:    %x\n", qrk_info->ss);

        printk("\nStack: \n");
        unsigned int stack_dump_size = (footer->struct_size -
                sizeof(struct panic_data_footer) -
                sizeof(*qrk_info)+3)>>2; // 4 byte aligned
        uint32_t *esp = (uint32_t*)qrk_info-stack_dump_size;

        for(i = 0; i < stack_dump_size; i++, esp++) {
            if (!(i & 3)) {
                printk("\n0x%x:", (uint32_t)(qrk_info->esp)+i*4);
            }
            printk(" %x", *esp);
        }
        }
        break;
    case ARC_CORE:
        {
        // Get ptr to arch specific dump data
        struct arcv2_panic_arch_data *arc_info = (struct arcv2_panic_arch_data*)((uint8_t*)footer - sizeof(struct arcv2_panic_arch_data));

        printk("ARC BUILD CKSUM: %X \n", footer->build_cksum);
        printk("ARC BUILD VERSION: %X \n", footer->struct_version);
        printk("ARC PANIC - eret=0x%X - sp=0x%X\n\n", arc_info->eret, arc_info->sp);

        printk("Registers: \n\n");
        for(i=0; i<27; i++) {
            printk(i < 10 ? " r%d:%x " : "r%d:%x ", i, arc_info->registers[i]);
            if((i+1) % 4 == 0) {
                printk("\n");
            }
        }
        printk(" fp:%x\n", arc_info->fp);
        printk(" sp:%x ", arc_info->sp);
        printk(" il:%x ", arc_info->il);
        printk("r30:%x ", arc_info->r30);
        printk(" bl:%x \n\n", arc_info->bl);

        printk("Status registers: \n\n");
        printk("      ecr:%x\n", arc_info->ecr);
        printk("      efa:%x\n", arc_info->efa);
        printk("     eret:%x\n", arc_info->eret);
        printk(" erstatus:%x\n", arc_info->erstatus);

        printk("\nStack: \n");
        unsigned int stack_dump_size = (footer->struct_size -
                sizeof(struct panic_data_footer) -
                sizeof(*arc_info)+3)>>2; // 4 byte aligned
        uint32_t *sp = (uint32_t*)arc_info-stack_dump_size;

        for(i = 0; i < stack_dump_size; i++, sp++) {
            if (!(i & 3)) {
                printk("\n0x%x:", (uint32_t)(arc_info->sp)+i*4);
            }
            printk(" %x", *sp);
        }
        }
        break;
    default:
        printk("PANIC from unknown arch\n");
    }
    // Flush last char in uart FIFO buffer
    printk("\n");
}
#endif

void panic_notify(int timeout)
{
#ifdef CONFIG_IPC
    if (!panic_core_status[ARC_CORE].is_panic) {
        // Send IPC notification to trigger panic on ARC
        ipc_request_sync_int(IPC_PANIC_NOTIFICATION, 0, 0, NULL);
    }
#endif
#ifdef CONFIG_IPC_UART
    if (!panic_core_status[NORDIC_CORE].is_panic) {
        // Send uart IPC notification to trigger panic on Nordic
        uart_ipc_send_sync_resp(PANIC_UART_IPC_CHANNEL, IPC_PANIC_NOTIFICATION, 0, 0, NULL);
    }
#endif

    // TODO: implement panic done signal handler
    panic_core_status[ARC_CORE].is_panic = 1;
    panic_core_status[NORDIC_CORE].is_panic = 1;

    // Wait a little for ARC to process panic notification
    unsigned int delay = (32000000/1000/(10+1+3))*timeout; // nop(10), dec(1), jne(3)
    __asm__("1:"
            "decl %0;"
            "nop;nop;nop;nop;nop;"
            "nop;nop;nop;nop;nop;"
            "jne 1b;"
            ::"r"(delay):"cc");
}

/**
 * \brief Exception trap function for QRK panic management
 *
 *  This function handles a panic triggered by an exception on QRK
 *
 * \param [in] QRK core panic dump pointer
 *
 */
void panic_trap(struct x86_panic_arch_data *dump)
{
    struct x86_panic_arch_data *qrk_info = &((struct x86_panic_data*)(__ram_phys_end)-1)->arch_data;
    struct panic_data_footer *footer = &((struct x86_panic_data*)(__ram_phys_end)-1)->footer;

    interrupt_lock();
    panic_core_status[QRK_CORE].is_panic = 1;


    footer->struct_size = sizeof(struct panic_data_footer)+
                          sizeof(struct x86_panic_arch_data);
    // Copy stack to panic RAM location if esp is valid
    if ((qrk_info->esp >= QUARK_RAM_START_ADDR) &&
            ((qrk_info->esp+PANIC_QRK_STACK_SIZE) <= (QUARK_RAM_START_ADDR + QUARK_RAM_SIZE))) {
        // Align esp pointer on 32bits to avoid alignement issues
        memcpy(((uint8_t*)qrk_info)-PANIC_QRK_STACK_SIZE,
                (uint32_t*)((qrk_info->esp+3) & (~3)), PANIC_QRK_STACK_SIZE);

        footer->struct_size += PANIC_QRK_STACK_SIZE;
    }

    // Set panic structure header
    footer->magic = PANIC_DATA_MAGIC;
    footer->arch = QRK_CORE;
    footer->flags = PANIC_DATA_FLAG_FRAME_VALID;
    footer->reserved = 0;
    footer->time = get_time_ms();
    memcpy(&footer->build_cksum, (const void *)version_header.hash, sizeof(version_header.hash));
    footer->struct_version = version_header.version;

    // Notify panic to the other cores with timeout as synchronisation
    panic_notify(PANIC_NOTIFY_TIMEOUT);

#ifdef CONFIG_QUARK_SE_PANIC_DEBUG
    // Flush QRK panic dump over UART (for debug purposes)
    panic_handler(footer);
    // Flush ARC panic dump over UART (for debug purposes)
    panic_handler((struct panic_data_footer*)
            (ARC_PANIC_DUMP_ADDR-sizeof(struct panic_data_footer)));
#endif
    // Reboot platform
    reboot();
    while(1);
}

void handle_panic_notification(int core_id)
{
    switch (core_id) {
    case ARC_CORE:
        panic_core_status[ARC_CORE].is_panic = 1;
        // Panic master core
        panic(PANIC_ARC_CORE);
        break;
    case NORDIC_CORE:
        panic_core_status[NORDIC_CORE].is_panic = 1;
        // Panic master core
        panic(PANIC_NORDIC_CORE);
        break;
    default:
        // Unknown panic reason
        panic(PANIC_UNKNOWN);
    }
}
