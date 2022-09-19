/****************************************************************************
 * arch/risc-v/src/k210/k210_plic.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_K210_K210_PLIC_H
#define __ARCH_RISCV_SRC_K210_K210_PLIC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*plic_irq_callback_t)(void *ctx);

typedef struct _plic_instance_t
{
    plic_irq_callback_t callback;
    void *ctx;
} plic_instance_t;

typedef struct _plic_callback_t
{
    plic_irq_callback_t callback;
    void *ctx;
    uint32_t priority;
} plic_interrupt_t;

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_K210_K210_PLIC_H */
