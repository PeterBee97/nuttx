/* Copyright 2018 Canaan Inc.
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
 */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/mutex.h>
#include <debug.h>
#include "k210_memorymap.h"
#include "k210_fpioa.h"
#include "k210_spi.h"
#include "k210_sysctl.h"
#include "chip.h"

#define FIX_CACHE 0

volatile spi_t *const spi[4] =
    {
        (volatile spi_t *)K210_SPI0_BASE,
        (volatile spi_t *)K210_SPI1_BASE,
        (volatile spi_t *)K210_SPI_SLAVE_BASE,
        (volatile spi_t *)K210_SPI3_BASE};

typedef struct _spi_dma_context
{
    uint8_t *buffer;
    size_t buf_len;
    uint32_t *malloc_buffer;
    spi_transfer_mode_t int_mode;
    dmac_channel_number_t dmac_channel;
    spi_device_num_t spi_num;
    plic_instance_t spi_int_instance;
} spi_dma_context_t;

spi_dma_context_t spi_dma_context[4];

typedef struct _spi_instance_t
{
    spi_device_num_t spi_num;
    spi_transfer_mode_t transfer_mode;
    dmac_channel_number_t dmac_channel;
    plic_instance_t spi_int_instance;
    mutex_t lock;
} spi_instance_t;

// static const struct spi_ops_s k210_spi2_ops =
// {
//   .lock              = k210_spi_lock,
//   .select            = k210_spi_select,
//   .setfrequency      = k210_spi_setfrequency,
//   .setmode           = k210_spi_setmode,
//   .setbits           = k210_spi_setbits,
// #ifdef CONFIG_SPI_HWFEATURES
//   .hwfeatures        = k210_spi_hwfeatures,
// #endif
//   .status            = k210_spi2_status,
// #ifdef CONFIG_SPI_CMDDATA
//   .cmddata           = k210_spi2_cmddata,
// #endif
//   .send              = k210_spi_send,
// #ifdef CONFIG_SPI_EXCHANGE
//   .exchange          = k210_spi_exchange,
// #else
//   .sndblock          = k210_spi_sndblock,
//   .recvblock         = k210_spi_recvblock,
// #endif
// #ifdef CONFIG_SPI_TRIGGER
//   .trigger           = k210_spi_trigger,
// #endif
//   .registercallback  = NULL,
// };

static spi_instance_t g_spi_instance[4];

static spi_frame_format_t spi_get_frame_format(spi_device_num_t spi_num)
{
    uint8_t frf_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            frf_offset = 21;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return -1;
        case 3:
        default:
            frf_offset = 22;
            break;
    }
    volatile spi_t *spi_adapter = spi[spi_num];
    return ((spi_adapter->ctrlr0 >> frf_offset) & 0x3);
}

static spi_transfer_width_t spi_get_frame_size(size_t data_bit_length)
{
    if(data_bit_length < 8)
        return SPI_TRANS_CHAR;
    else if(data_bit_length < 16)
        return SPI_TRANS_SHORT;
    return SPI_TRANS_INT;
}

static int spi_dma_irq(int irq, FAR void *ctx, FAR void *arg)
{
    spi_instance_t *v_instance = (spi_instance_t *)ctx;
    volatile spi_t *spi_handle = spi[v_instance->spi_num];
    dmac_irq_unregister(v_instance->dmac_channel);
    while((spi_handle->sr & 0x05) != 0x04)
        ;
    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;
    nxmutex_unlock(&v_instance->lock);
    if(v_instance->spi_int_instance.callback)
    {
        v_instance->spi_int_instance.callback(v_instance->spi_int_instance.ctx);
    }
    return 0;
}

static int spi_clk_init(uint8_t spi_num)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    if(spi_num == 3)
        sysctl_clock_set_clock_select(SYSCTL_CLOCK_SELECT_SPI3, 1);
    sysctl_clock_enable(SYSCTL_CLOCK_SPI0 + spi_num);
    sysctl_clock_set_threshold(SYSCTL_THRESHOLD_SPI0 + spi_num, 0);
    return 0;
}

static void spi_set_tmod(uint8_t spi_num, uint32_t tmod)
{
    assert(spi_num < SPI_DEVICE_MAX);
    volatile spi_t *spi_handle = spi[spi_num];
    uint8_t tmod_offset = 0;
    switch(spi_num)
    {
        case 0:
        case 1:
        case 2:
            tmod_offset = 8;
            break;
        case 3:
        default:
            tmod_offset = 10;
            break;
    }
    modifyreg32((uintptr_t)&spi_handle->ctrlr0, 3 << tmod_offset, (tmod & 3) << tmod_offset);
}

void spi_init(spi_device_num_t spi_num, spi_work_mode_t work_mode, spi_frame_format_t frame_format,
              size_t data_bit_length, uint32_t endian)
{
    assert(data_bit_length >= 4 && data_bit_length <= 32);
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    spi_clk_init(spi_num);

    uint8_t dfs_offset, frf_offset, work_mode_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            frf_offset = 21;
            work_mode_offset = 6;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            frf_offset = 22;
            work_mode_offset = 8;
            break;
    }

    switch(frame_format)
    {
        case SPI_FF_DUAL:
            assert(data_bit_length % 2 == 0);
            break;
        case SPI_FF_QUAD:
            assert(data_bit_length % 4 == 0);
            break;
        case SPI_FF_OCTAL:
            assert(data_bit_length % 8 == 0);
            break;
        default:
            break;
    }
    volatile spi_t *spi_adapter = spi[spi_num];
    if(spi_adapter->baudr == 0)
        spi_adapter->baudr = 0x14;
    spi_adapter->imr = 0x00;
    spi_adapter->dmacr = 0x00;
    spi_adapter->dmatdlr = 0x10;
    spi_adapter->dmardlr = 0x00;
    spi_adapter->ser = 0x00;
    spi_adapter->ssienr = 0x00;
    spi_adapter->ctrlr0 = (work_mode << work_mode_offset) | (frame_format << frf_offset) | ((data_bit_length - 1) << dfs_offset);
    spi_adapter->spi_ctrlr0 = 0;
    spi_adapter->endian = endian;
}

void spi_init_non_standard(spi_device_num_t spi_num, uint32_t instruction_length, uint32_t address_length,
                           uint32_t wait_cycles, spi_instruction_address_trans_mode_t instruction_address_trans_mode)
{
    assert(wait_cycles < (1 << 5));
    assert(instruction_address_trans_mode < 3);
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    volatile spi_t *spi_handle = spi[spi_num];
    uint32_t inst_l = 0;
    switch(instruction_length)
    {
        case 0:
            inst_l = 0;
            break;
        case 4:
            inst_l = 1;
            break;
        case 8:
            inst_l = 2;
            break;
        case 16:
            inst_l = 3;
            break;
        default:
            syslog(LOG_ERR, "Invalid instruction length");
            return;
    }

    assert(address_length % 4 == 0 && address_length <= 60);
    uint32_t addr_l = address_length / 4;

    spi_handle->spi_ctrlr0 = (wait_cycles << 11) | (inst_l << 8) | (addr_l << 2) | instruction_address_trans_mode;
}

uint32_t spi_set_clk_rate(spi_device_num_t spi_num, uint32_t spi_clk)
{
    uint32_t spi_baudr = sysctl_clock_get_freq(SYSCTL_CLOCK_SPI0 + spi_num) / spi_clk;
    if(spi_baudr < 2)
    {
        spi_baudr = 2;
    } else if(spi_baudr > 65534)
    {
        spi_baudr = 65534;
    }
    volatile spi_t *spi_adapter = spi[spi_num];
    spi_adapter->baudr = spi_baudr;
    return sysctl_clock_get_freq(SYSCTL_CLOCK_SPI0 + spi_num) / spi_baudr;
}

void spi_send_data_normal(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint8_t *tx_buff, size_t tx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);

    size_t index, fifo_len;
    spi_set_tmod(spi_num, SPI_TMOD_TRANS);

    volatile spi_t *spi_handle = spi[spi_num];

    uint8_t dfs_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            break;
    }
    uint32_t data_bit_length = (spi_handle->ctrlr0 >> dfs_offset) & 0x1F;
    spi_transfer_width_t frame_width = spi_get_frame_size(data_bit_length);

    uint8_t v_misalign_flag = 0;
    uint32_t v_send_data;
    if((uintptr_t)tx_buff % frame_width)
        v_misalign_flag = 1;

    spi_handle->ssienr = 0x01;
    spi_handle->ser = 1U << chip_select;
    uint32_t i = 0;
    while(tx_len)
    {
        fifo_len = 32 - spi_handle->txflr;
        fifo_len = fifo_len < tx_len ? fifo_len : tx_len;
        switch(frame_width)
        {
            case SPI_TRANS_INT:
                fifo_len = fifo_len / 4 * 4;
                if(v_misalign_flag)
                {
                    for(index = 0; index < fifo_len; index += 4)
                    {
                        memcpy(&v_send_data, tx_buff + i, 4);
                        spi_handle->dr[0] = v_send_data;
                        i += 4;
                    }
                } else
                {
                    for(index = 0; index < fifo_len / 4; index++)
                        spi_handle->dr[0] = ((uint32_t *)tx_buff)[i++];
                }
                break;
            case SPI_TRANS_SHORT:
                fifo_len = fifo_len / 2 * 2;
                if(v_misalign_flag)
                {
                    for(index = 0; index < fifo_len; index += 2)
                    {
                        memcpy(&v_send_data, tx_buff + i, 2);
                        spi_handle->dr[0] = v_send_data;
                        i += 2;
                    }
                } else
                {
                    for(index = 0; index < fifo_len / 2; index++)
                        spi_handle->dr[0] = ((uint16_t *)tx_buff)[i++];
                }
                break;
            default:
                for(index = 0; index < fifo_len; index++)
                    spi_handle->dr[0] = tx_buff[i++];
                break;
        }
        tx_len -= fifo_len;
    }
    while((spi_handle->sr & 0x05) != 0x04)
        ;
    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;
}

void spi_send_data_standard(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint8_t *cmd_buff,
                            size_t cmd_len, const uint8_t *tx_buff, size_t tx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    uint8_t *v_buf = malloc(cmd_len + tx_len);
    size_t i;
    for(i = 0; i < cmd_len; i++)
        v_buf[i] = cmd_buff[i];
    for(i = 0; i < tx_len; i++)
        v_buf[cmd_len + i] = tx_buff[i];

    spi_send_data_normal(spi_num, chip_select, v_buf, cmd_len + tx_len);
    free((void *)v_buf);
}

void spi_send_data_standard_dma(dmac_channel_number_t channel_num, spi_device_num_t spi_num,
                                spi_chip_select_t chip_select,
                                const uint8_t *cmd_buff, size_t cmd_len, const uint8_t *tx_buff, size_t tx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);

    volatile spi_t *spi_handle = spi[spi_num];

    uint8_t dfs_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            break;
    }
    uint32_t data_bit_length = (spi_handle->ctrlr0 >> dfs_offset) & 0x1F;
    spi_transfer_width_t frame_width = spi_get_frame_size(data_bit_length);

    uint32_t *buf;
    size_t v_send_len;
    int i;
    switch(frame_width)
    {
        case SPI_TRANS_INT:
            buf = malloc(cmd_len + tx_len);
            for(i = 0; i < cmd_len / 4; i++)
                buf[i] = ((uint32_t *)cmd_buff)[i];
            for(i = 0; i < tx_len / 4; i++)
                buf[cmd_len / 4 + i] = ((uint32_t *)tx_buff)[i];
            v_send_len = (cmd_len + tx_len) / 4;
            break;
        case SPI_TRANS_SHORT:
            buf = malloc((cmd_len + tx_len) / 2 * sizeof(uint32_t));
            for(i = 0; i < cmd_len / 2; i++)
                buf[i] = ((uint16_t *)cmd_buff)[i];
            for(i = 0; i < tx_len / 2; i++)
                buf[cmd_len / 2 + i] = ((uint16_t *)tx_buff)[i];
            v_send_len = (cmd_len + tx_len) / 2;
            break;
        default:
            buf = malloc((cmd_len + tx_len) * sizeof(uint32_t));
            for(i = 0; i < cmd_len; i++)
                buf[i] = cmd_buff[i];
            for(i = 0; i < tx_len; i++)
                buf[cmd_len + i] = tx_buff[i];
            v_send_len = cmd_len + tx_len;
            break;
    }

    spi_send_data_normal_dma(channel_num, spi_num, chip_select, buf, v_send_len, SPI_TRANS_INT);

    free((void *)buf);
}

void spi_send_data_normal_dma(dmac_channel_number_t channel_num, spi_device_num_t spi_num,
                              spi_chip_select_t chip_select,
                              const void *tx_buff, size_t tx_len, spi_transfer_width_t spi_transfer_width)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    spi_set_tmod(spi_num, SPI_TMOD_TRANS);
    volatile spi_t *spi_handle = spi[spi_num];
    uint32_t *buf;
    int i;
    switch(spi_transfer_width)
    {
        case SPI_TRANS_SHORT:
#if FIX_CACHE
            buf = (uint32_t *)iomem_malloc((tx_len) * sizeof(uint32_t));
#else
            buf = (uint32_t *)malloc((tx_len) * sizeof(uint32_t));
#endif
            for(i = 0; i < tx_len; i++)
                buf[i] = ((uint16_t *)tx_buff)[i];
            break;
        case SPI_TRANS_INT:
            buf = (uint32_t *)tx_buff;
            break;
        case SPI_TRANS_CHAR:
        default:
#if FIX_CACHE
            buf = (uint32_t *)iomem_malloc((tx_len) * sizeof(uint32_t));
#else
            buf = (uint32_t *)malloc((tx_len) * sizeof(uint32_t));
#endif

            for(i = 0; i < tx_len; i++)
                buf[i] = ((uint8_t *)tx_buff)[i];
            break;
    }
    spi_handle->dmacr = 0x2; /*enable dma transmit*/
    spi_handle->ssienr = 0x01;

    sysctl_dma_select((sysctl_dma_channel_t)channel_num, SYSCTL_DMA_SELECT_SSI0_TX_REQ + spi_num * 2);

    dmac_set_single_mode(channel_num, buf, (void *)(&spi_handle->dr[0]), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                         DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, tx_len);
    spi_handle->ser = 1U << chip_select;
    dmac_wait_done(channel_num);
    if(spi_transfer_width != SPI_TRANS_INT)
    {
#if FIX_CACHE
        iomem_free((void *)buf);
#else
        free((void *)buf);
#endif
    }
    while((spi_handle->sr & 0x05) != 0x04)
        ;
    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;
}

void spi_dup_send_receive_data_dma(dmac_channel_number_t dma_send_channel_num,
                                   dmac_channel_number_t dma_receive_channel_num,
                                   spi_device_num_t spi_num, spi_chip_select_t chip_select,
                                   const uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len)
{
    spi_set_tmod(spi_num, SPI_TMOD_TRANS_RECV);
    volatile spi_t *spi_handle = spi[spi_num];

    uint8_t dfs_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            break;
    }
    uint32_t data_bit_length = (spi_handle->ctrlr0 >> dfs_offset) & 0x1F;
    spi_transfer_width_t frame_width = spi_get_frame_size(data_bit_length);
    size_t v_tx_len = tx_len / frame_width;
    size_t v_rx_len = rx_len / frame_width;

    size_t v_max_len = v_tx_len > v_rx_len ? v_tx_len : v_rx_len;

#if FIX_CACHE
    uint32_t *v_tx_buf = iomem_malloc(v_max_len * 4);
    uint32_t *v_rx_buf = iomem_malloc(v_max_len * 4);
#else
    uint32_t *v_tx_buf = malloc(v_max_len * 4);
    uint32_t *v_rx_buf = malloc(v_max_len * 4);
#endif
    uint32_t i = 0;
    switch(frame_width)
    {
        case SPI_TRANS_INT:
            for(i = 0; i < v_tx_len; i++)
            {
                v_tx_buf[i] = ((uint32_t *)tx_buf)[i];
            }
            if(v_max_len > v_tx_len)
            {
                while(i < v_max_len)
                {
                    v_tx_buf[i++] = 0xFFFFFFFF;
                }
            }
            break;
        case SPI_TRANS_SHORT:
            for(i = 0; i < v_tx_len; i++)
            {
                v_tx_buf[i] = ((uint16_t *)tx_buf)[i];
            }
            if(v_max_len > v_tx_len)
            {
                while(i < v_max_len)
                {
                    v_tx_buf[i++] = 0xFFFFFFFF;
                }
            }
            break;
        default:
            for(i = 0; i < v_tx_len; i++)
            {
                v_tx_buf[i] = tx_buf[i];
            }
            if(v_max_len > v_tx_len)
            {
                while(i < v_max_len)
                {
                    v_tx_buf[i++] = 0xFFFFFFFF;
                }
            }
            break;
    }

    spi_handle->dmacr = 0x3;
    spi_handle->ssienr = 0x01;

    sysctl_dma_select((sysctl_dma_channel_t)dma_send_channel_num, SYSCTL_DMA_SELECT_SSI0_TX_REQ + spi_num * 2);
    sysctl_dma_select((sysctl_dma_channel_t)dma_receive_channel_num, SYSCTL_DMA_SELECT_SSI0_RX_REQ + spi_num * 2);

    dmac_set_single_mode(dma_receive_channel_num, (void *)(&spi_handle->dr[0]), v_rx_buf, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                         DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, v_max_len);

    dmac_set_single_mode(dma_send_channel_num, v_tx_buf, (void *)(&spi_handle->dr[0]), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                         DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, v_max_len);

    spi_handle->ser = 1U << chip_select;
    dmac_wait_done(dma_send_channel_num);
    dmac_wait_done(dma_receive_channel_num);

    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;

    switch(frame_width)
    {
        case SPI_TRANS_INT:
            for(i = 0; i < v_rx_len; i++)
                ((uint32_t *)rx_buf)[i] = v_rx_buf[i];
            break;
        case SPI_TRANS_SHORT:
            for(i = 0; i < v_rx_len; i++)
                ((uint16_t *)rx_buf)[i] = v_rx_buf[i];
            break;
        default:
            for(i = 0; i < v_rx_len; i++)
                rx_buf[i] = v_rx_buf[i];
            break;
    }
#if FIX_CACHE
    iomem_free(v_tx_buf);
    iomem_free(v_rx_buf);
#else
    free(v_tx_buf);
    free(v_rx_buf);
#endif
}

void spi_receive_data_standard(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint8_t *cmd_buff,
                               size_t cmd_len, uint8_t *rx_buff, size_t rx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    size_t index, fifo_len;
    if(cmd_len == 0)
        spi_set_tmod(spi_num, SPI_TMOD_RECV);
    else
        spi_set_tmod(spi_num, SPI_TMOD_EEROM);
    volatile spi_t *spi_handle = spi[spi_num];

    uint8_t dfs_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            break;
    }
    uint32_t data_bit_length = (spi_handle->ctrlr0 >> dfs_offset) & 0x1F;
    spi_transfer_width_t frame_width = spi_get_frame_size(data_bit_length);

    uint32_t i = 0;
    size_t v_cmd_len = cmd_len / frame_width;
    uint32_t v_rx_len = rx_len / frame_width;

    spi_handle->ctrlr1 = (uint32_t)(v_rx_len - 1);
    spi_handle->ssienr = 0x01;

    while(v_cmd_len)
    {
        fifo_len = 32 - spi_handle->txflr;
        fifo_len = fifo_len < v_cmd_len ? fifo_len : v_cmd_len;
        switch(frame_width)
        {
            case SPI_TRANS_INT:
                for(index = 0; index < fifo_len; index++)
                    spi_handle->dr[0] = ((uint32_t *)cmd_buff)[i++];
                break;
            case SPI_TRANS_SHORT:
                for(index = 0; index < fifo_len; index++)
                    spi_handle->dr[0] = ((uint16_t *)cmd_buff)[i++];
                break;
            default:
                for(index = 0; index < fifo_len; index++)
                    spi_handle->dr[0] = cmd_buff[i++];
                break;
        }
        spi_handle->ser = 1U << chip_select;
        v_cmd_len -= fifo_len;
    }

    if(cmd_len == 0)
    {
        spi_handle->dr[0] = 0xffffffff;
        spi_handle->ser = 1U << chip_select;
    }

    i = 0;
    while(v_rx_len)
    {
        fifo_len = spi_handle->rxflr;
        fifo_len = fifo_len < v_rx_len ? fifo_len : v_rx_len;
        switch(frame_width)
        {
            case SPI_TRANS_INT:
                for(index = 0; index < fifo_len; index++)
                    ((uint32_t *)rx_buff)[i++] = spi_handle->dr[0];
                break;
            case SPI_TRANS_SHORT:
                for(index = 0; index < fifo_len; index++)
                    ((uint16_t *)rx_buff)[i++] = (uint16_t)spi_handle->dr[0];
                break;
            default:
                for(index = 0; index < fifo_len; index++)
                    rx_buff[i++] = (uint8_t)spi_handle->dr[0];
                break;
        }

        v_rx_len -= fifo_len;
    }

    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;
}

void spi_receive_data_normal_dma(dmac_channel_number_t dma_send_channel_num,
                                 dmac_channel_number_t dma_receive_channel_num,
                                 spi_device_num_t spi_num, spi_chip_select_t chip_select, const void *cmd_buff,
                                 size_t cmd_len, void *rx_buff, size_t rx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);

    if(cmd_len == 0)
        spi_set_tmod(spi_num, SPI_TMOD_RECV);
    else
        spi_set_tmod(spi_num, SPI_TMOD_EEROM);

    volatile spi_t *spi_handle = spi[spi_num];

    spi_handle->ctrlr1 = (uint32_t)(rx_len - 1);
    spi_handle->dmacr = 0x3;
    spi_handle->ssienr = 0x01;
    if(cmd_len)
        sysctl_dma_select((sysctl_dma_channel_t)dma_send_channel_num, SYSCTL_DMA_SELECT_SSI0_TX_REQ + spi_num * 2);
    sysctl_dma_select((sysctl_dma_channel_t)dma_receive_channel_num, SYSCTL_DMA_SELECT_SSI0_RX_REQ + spi_num * 2);

    dmac_set_single_mode(dma_receive_channel_num, (void *)(&spi_handle->dr[0]), rx_buff, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                         DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, rx_len);
    if(cmd_len)
        dmac_set_single_mode(dma_send_channel_num, cmd_buff, (void *)(&spi_handle->dr[0]), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                             DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, cmd_len);
    if(cmd_len == 0 && spi_get_frame_format(spi_num) == SPI_FF_STANDARD)
        spi[spi_num]->dr[0] = 0xffffffff;
    spi_handle->ser = 1U << chip_select;
    if(cmd_len)
        dmac_wait_done(dma_send_channel_num);
    dmac_wait_done(dma_receive_channel_num);

    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;
}

void spi_receive_data_standard_dma(dmac_channel_number_t dma_send_channel_num,
                                   dmac_channel_number_t dma_receive_channel_num,
                                   spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint8_t *cmd_buff,
                                   size_t cmd_len, uint8_t *rx_buff, size_t rx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    volatile spi_t *spi_handle = spi[spi_num];

    uint8_t dfs_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            break;
    }
    uint32_t data_bit_length = (spi_handle->ctrlr0 >> dfs_offset) & 0x1F;
    spi_transfer_width_t frame_width = spi_get_frame_size(data_bit_length);

    size_t i;

    uint32_t *write_cmd;
    uint32_t *read_buf;
    size_t v_recv_len;
    size_t v_cmd_len;
    switch(frame_width)
    {
        case SPI_TRANS_INT:
#if FIX_CACHE
            write_cmd = iomem_malloc(cmd_len + rx_len);
#else
            write_cmd = malloc(cmd_len + rx_len);
#endif
            for(i = 0; i < cmd_len / 4; i++)
                write_cmd[i] = ((uint32_t *)cmd_buff)[i];
            read_buf = &write_cmd[i];
            v_recv_len = rx_len / 4;
            v_cmd_len = cmd_len / 4;
            break;
        case SPI_TRANS_SHORT:
#if FIX_CACHE
            write_cmd = iomem_malloc((cmd_len + rx_len) / 2 * sizeof(uint32_t));
#else
            write_cmd = malloc((cmd_len + rx_len) / 2 * sizeof(uint32_t));
#endif
            for(i = 0; i < cmd_len / 2; i++)
                write_cmd[i] = ((uint16_t *)cmd_buff)[i];
            read_buf = &write_cmd[i];
            v_recv_len = rx_len / 2;
            v_cmd_len = cmd_len / 2;
            break;
        default:
#if FIX_CACHE
            write_cmd = iomem_malloc((cmd_len + rx_len) * sizeof(uint32_t));
#else
            write_cmd = malloc((cmd_len + rx_len) * sizeof(uint32_t));
#endif
            for(i = 0; i < cmd_len; i++)
                write_cmd[i] = cmd_buff[i];
            read_buf = &write_cmd[i];
            v_recv_len = rx_len;
            v_cmd_len = cmd_len;
            break;
    }

    spi_receive_data_normal_dma(dma_send_channel_num, dma_receive_channel_num, spi_num, chip_select, write_cmd, v_cmd_len, read_buf, v_recv_len);

    switch(frame_width)
    {
        case SPI_TRANS_INT:
            for(i = 0; i < v_recv_len; i++)
                ((uint32_t *)rx_buff)[i] = read_buf[i];
            break;
        case SPI_TRANS_SHORT:
            for(i = 0; i < v_recv_len; i++)
                ((uint16_t *)rx_buff)[i] = read_buf[i];
            break;
        default:
            for(i = 0; i < v_recv_len; i++)
                rx_buff[i] = read_buf[i];
            break;
    }
#if FIX_CACHE
    iomem_free(write_cmd);
#else
    free(write_cmd);
#endif
}

void spi_receive_data_multiple(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint32_t *cmd_buff,
                               size_t cmd_len, uint8_t *rx_buff, size_t rx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);

    size_t index, fifo_len;
    if(cmd_len == 0)
        spi_set_tmod(spi_num, SPI_TMOD_RECV);
    else
        spi_set_tmod(spi_num, SPI_TMOD_EEROM);
    volatile spi_t *spi_handle = spi[spi_num];

    uint8_t dfs_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            break;
    }
    uint32_t data_bit_length = (spi_handle->ctrlr0 >> dfs_offset) & 0x1F;
    spi_transfer_width_t frame_width = spi_get_frame_size(data_bit_length);

    uint32_t v_cmd_len = cmd_len;
    uint32_t i = 0;

    uint32_t v_rx_len = rx_len / frame_width;

    spi_handle->ctrlr1 = (uint32_t)(v_rx_len - 1);
    spi_handle->ssienr = 0x01;

    while(v_cmd_len)
    {
        fifo_len = 32 - spi_handle->txflr;
        fifo_len = fifo_len < v_cmd_len ? fifo_len : v_cmd_len;

        for(index = 0; index < fifo_len; index++)
            spi_handle->dr[0] = *cmd_buff++;

        spi_handle->ser = 1U << chip_select;
        v_cmd_len -= fifo_len;
    }

    if(cmd_len == 0)
    {
        spi_handle->ser = 1U << chip_select;
    }

    while(v_rx_len)
    {
        fifo_len = spi_handle->rxflr;
        fifo_len = fifo_len < v_rx_len ? fifo_len : v_rx_len;
        switch(frame_width)
        {
            case SPI_TRANS_INT:
                for(index = 0; index < fifo_len; index++)
                    ((uint32_t *)rx_buff)[i++] = spi_handle->dr[0];
                break;
            case SPI_TRANS_SHORT:
                for(index = 0; index < fifo_len; index++)
                    ((uint16_t *)rx_buff)[i++] = (uint16_t)spi_handle->dr[0];
                break;
            default:
                for(index = 0; index < fifo_len; index++)
                    rx_buff[i++] = (uint8_t)spi_handle->dr[0];
                break;
        }

        v_rx_len -= fifo_len;
    }

    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;
}

void spi_receive_data_multiple_dma(dmac_channel_number_t dma_send_channel_num,
                                   dmac_channel_number_t dma_receive_channel_num,
                                   spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint32_t *cmd_buff,
                                   size_t cmd_len, uint8_t *rx_buff, size_t rx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);

    volatile spi_t *spi_handle = spi[spi_num];

    uint8_t dfs_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            break;
    }
    uint32_t data_bit_length = (spi_handle->ctrlr0 >> dfs_offset) & 0x1F;
    spi_transfer_width_t frame_width = spi_get_frame_size(data_bit_length);

    size_t i;

    uint32_t *write_cmd = NULL;
    uint32_t *read_buf;
    size_t v_recv_len;
    switch(frame_width)
    {
        case SPI_TRANS_INT:
            v_recv_len = rx_len / 4;
            break;
        case SPI_TRANS_SHORT:
#if FIX_CACHE
            write_cmd = iomem_malloc(cmd_len + rx_len / 2 * sizeof(uint32_t));
#else
            write_cmd = malloc(cmd_len + rx_len / 2 * sizeof(uint32_t));
#endif
            for(i = 0; i < cmd_len; i++)
                write_cmd[i] = cmd_buff[i];
            read_buf = &write_cmd[i];
            v_recv_len = rx_len / 2;
            break;
        default:
#if FIX_CACHE
            write_cmd = iomem_malloc(cmd_len + rx_len * sizeof(uint32_t));
#else
            write_cmd = malloc(cmd_len + rx_len * sizeof(uint32_t));
#endif
            for(i = 0; i < cmd_len; i++)
                write_cmd[i] = cmd_buff[i];
            read_buf = &write_cmd[i];
            v_recv_len = rx_len;
            break;
    }
    if(frame_width == SPI_TRANS_INT)
        spi_receive_data_normal_dma(dma_send_channel_num, dma_receive_channel_num, spi_num, chip_select, cmd_buff, cmd_len, rx_buff, v_recv_len);
    else
        spi_receive_data_normal_dma(dma_send_channel_num, dma_receive_channel_num, spi_num, chip_select, write_cmd, cmd_len, read_buf, v_recv_len);

    switch(frame_width)
    {
        case SPI_TRANS_INT:
            break;
        case SPI_TRANS_SHORT:
            for(i = 0; i < v_recv_len; i++)
                ((uint16_t *)rx_buff)[i] = read_buf[i];
            break;
        default:
            for(i = 0; i < v_recv_len; i++)
                rx_buff[i] = read_buf[i];
            break;
    }

    if(frame_width != SPI_TRANS_INT)
    {
#if FIX_CACHE
        iomem_free(write_cmd);
#else
        free(write_cmd);
#endif
    }
}

void spi_send_data_multiple(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint32_t *cmd_buff,
                            size_t cmd_len, const uint8_t *tx_buff, size_t tx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);

    size_t index, fifo_len;
    spi_set_tmod(spi_num, SPI_TMOD_TRANS);
    volatile spi_t *spi_handle = spi[spi_num];
    spi_handle->ssienr = 0x01;
    spi_handle->ser = 1U << chip_select;

    size_t v_cmd_len = cmd_len * 4;
    while(v_cmd_len)
    {
        fifo_len = 32 - spi_handle->txflr;
        fifo_len = fifo_len < v_cmd_len ? fifo_len : v_cmd_len;
        fifo_len = fifo_len / 4 * 4;
        for(index = 0; index < fifo_len / 4; index++)
            spi_handle->dr[0] = *cmd_buff++;
        v_cmd_len -= fifo_len;
    }
    spi_send_data_normal(spi_num, chip_select, tx_buff, tx_len);
}

void spi_send_data_multiple_dma(dmac_channel_number_t channel_num, spi_device_num_t spi_num,
                                spi_chip_select_t chip_select,
                                const uint32_t *cmd_buff, size_t cmd_len, const uint8_t *tx_buff, size_t tx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    volatile spi_t *spi_handle = spi[spi_num];

    uint8_t dfs_offset;
    switch(spi_num)
    {
        case 0:
        case 1:
            dfs_offset = 16;
            break;
        case 2:
            syslog(LOG_ERR, "Spi Bus 2 Not Support!");
            return;
        case 3:
        default:
            dfs_offset = 0;
            break;
    }
    uint32_t data_bit_length = (spi_handle->ctrlr0 >> dfs_offset) & 0x1F;
    spi_transfer_width_t frame_width = spi_get_frame_size(data_bit_length);

    uint32_t *buf;
    size_t v_send_len;
    int i;
    switch(frame_width)
    {
        case SPI_TRANS_INT:
#if FIX_CACHE
            buf = iomem_malloc(cmd_len * sizeof(uint32_t) + tx_len);
#else
            buf = malloc(cmd_len * sizeof(uint32_t) + tx_len);
#endif
            for(i = 0; i < cmd_len; i++)
                buf[i] = cmd_buff[i];
            for(i = 0; i < tx_len / 4; i++)
                buf[cmd_len + i] = ((uint32_t *)tx_buff)[i];
            v_send_len = cmd_len + tx_len / 4;
            break;
        case SPI_TRANS_SHORT:
#if FIX_CACHE
            buf = iomem_malloc(cmd_len * sizeof(uint32_t) + tx_len / 2 * sizeof(uint32_t));
#else
            buf = malloc(cmd_len * sizeof(uint32_t) + tx_len / 2 * sizeof(uint32_t));
#endif
            for(i = 0; i < cmd_len; i++)
                buf[i] = cmd_buff[i];
            for(i = 0; i < tx_len / 2; i++)
                buf[cmd_len + i] = ((uint16_t *)tx_buff)[i];
            v_send_len = cmd_len + tx_len / 2;
            break;
        default:
#if FIX_CACHE
            buf = iomem_malloc((cmd_len + tx_len) * sizeof(uint32_t));
#else
            buf = malloc((cmd_len + tx_len) * sizeof(uint32_t));
#endif
            for(i = 0; i < cmd_len; i++)
                buf[i] = cmd_buff[i];
            for(i = 0; i < tx_len; i++)
                buf[cmd_len + i] = tx_buff[i];
            v_send_len = cmd_len + tx_len;
            break;
    }

    spi_send_data_normal_dma(channel_num, spi_num, chip_select, buf, v_send_len, SPI_TRANS_INT);

#if FIX_CACHE
    iomem_free((void *)buf);
#else
    free((void *)buf);
#endif
}

void spi_fill_data_dma(dmac_channel_number_t channel_num, spi_device_num_t spi_num, spi_chip_select_t chip_select,
                       const uint32_t *tx_buff, size_t tx_len)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);

    spi_set_tmod(spi_num, SPI_TMOD_TRANS);
    volatile spi_t *spi_handle = spi[spi_num];
    spi_handle->dmacr = 0x2; /*enable dma transmit*/
    spi_handle->ssienr = 0x01;

    sysctl_dma_select((sysctl_dma_channel_t)channel_num, SYSCTL_DMA_SELECT_SSI0_TX_REQ + spi_num * 2);
    dmac_set_single_mode(channel_num, tx_buff, (void *)(&spi_handle->dr[0]), DMAC_ADDR_NOCHANGE, DMAC_ADDR_NOCHANGE,
                         DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, tx_len);
    spi_handle->ser = 1U << chip_select;
    dmac_wait_done(channel_num);

    while((spi_handle->sr & 0x05) != 0x04)
        ;
    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;
}

void spi_handle_data_dma(spi_device_num_t spi_num, spi_chip_select_t chip_select, spi_data_t data, plic_interrupt_t *cb)
{
    assert(spi_num < SPI_DEVICE_MAX && spi_num != 2);
    assert(chip_select < SPI_CHIP_SELECT_MAX);
    switch(data.transfer_mode)
    {
        case SPI_TMOD_TRANS_RECV:
        case SPI_TMOD_EEROM:
            assert(data.tx_buf && data.tx_len && data.rx_buf && data.rx_len);
            break;
        case SPI_TMOD_TRANS:
            assert(data.tx_buf && data.tx_len);
            break;
        case SPI_TMOD_RECV:
            assert(data.rx_buf && data.rx_len);
            break;
        default:
            syslog(LOG_ERR, "Transfer Mode ERR");
            return;
    }
    assert(data.tx_channel < DMAC_CHANNEL_MAX && data.rx_channel < DMAC_CHANNEL_MAX);
    volatile spi_t *spi_handle = spi[spi_num];

    nxmutex_lock(&g_spi_instance[spi_num].lock);
    if(cb)
    {
        g_spi_instance[spi_num].spi_int_instance.callback = cb->callback;
        g_spi_instance[spi_num].spi_int_instance.ctx = cb->ctx;
    }
    switch(data.transfer_mode)
    {
        case SPI_TMOD_RECV:
            spi_set_tmod(spi_num, SPI_TMOD_RECV);
            if(data.rx_len > 65536)
                data.rx_len = 65536;
            spi_handle->ctrlr1 = (uint32_t)(data.rx_len - 1);
            spi_handle->dmacr = 0x03;
            spi_handle->ssienr = 0x01;
            if(spi_get_frame_format(spi_num) == SPI_FF_STANDARD)
                spi_handle->dr[0] = 0xffffffff;
            if(cb)
            {
                dmac_irq_register(data.rx_channel, spi_dma_irq, &g_spi_instance[spi_num], cb->priority);
                g_spi_instance[spi_num].dmac_channel = data.rx_channel;
            }
            sysctl_dma_select((sysctl_dma_channel_t)data.rx_channel, SYSCTL_DMA_SELECT_SSI0_RX_REQ + spi_num * 2);
            dmac_set_single_mode(data.rx_channel, (void *)(&spi_handle->dr[0]), (void *)data.rx_buf, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                                 DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, data.rx_len);
            spi_handle->ser = 1U << chip_select;
            if(!cb)
                dmac_wait_done(data.rx_channel);
            break;
        case SPI_TMOD_TRANS:
            spi_set_tmod(spi_num, SPI_TMOD_TRANS);
            spi_handle->dmacr = 0x2; /*enable dma transmit*/
            spi_handle->ssienr = 0x01;

            if(cb)
            {
                dmac_irq_register(data.tx_channel, spi_dma_irq, &g_spi_instance[spi_num], cb->priority);
                g_spi_instance[spi_num].dmac_channel = data.tx_channel;
            }
            sysctl_dma_select(data.tx_channel, SYSCTL_DMA_SELECT_SSI0_TX_REQ + spi_num * 2);
            if(data.fill_mode)
                dmac_set_single_mode(data.tx_channel, data.tx_buf, (void *)(&spi_handle->dr[0]), DMAC_ADDR_NOCHANGE, DMAC_ADDR_NOCHANGE,
                                     DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, data.tx_len);
            else
                dmac_set_single_mode(data.tx_channel, data.tx_buf, (void *)(&spi_handle->dr[0]), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                                     DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, data.tx_len);
            spi_handle->ser = 1U << chip_select;
            if(!cb)
            {
                dmac_wait_done(data.tx_channel);
                while((spi_handle->sr & 0x05) != 0x04)
                    ;
            }
            break;
        case SPI_TMOD_EEROM:
            spi_set_tmod(spi_num, SPI_TMOD_EEROM);
            if(data.rx_len > 65536)
                data.rx_len = 65536;
            spi_handle->ctrlr1 = (uint32_t)(data.rx_len - 1);
            spi_handle->dmacr = 0x3;
            spi_handle->ssienr = 0x01;
            sysctl_dma_select(data.tx_channel, SYSCTL_DMA_SELECT_SSI0_TX_REQ + spi_num * 2);
            if(data.fill_mode)
                dmac_set_single_mode(data.tx_channel, data.tx_buf, (void *)(&spi_handle->dr[0]), DMAC_ADDR_NOCHANGE, DMAC_ADDR_NOCHANGE,
                                     DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, data.tx_len);
            else
                dmac_set_single_mode(data.tx_channel, data.tx_buf, (void *)(&spi_handle->dr[0]), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                                     DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, data.tx_len);
            if(cb)
            {
                dmac_irq_register(data.rx_channel, spi_dma_irq, &g_spi_instance[spi_num], cb->priority);
                g_spi_instance[spi_num].dmac_channel = data.rx_channel;
            }
            sysctl_dma_select(data.rx_channel, SYSCTL_DMA_SELECT_SSI0_RX_REQ + spi_num * 2);
            dmac_set_single_mode(data.rx_channel, (void *)(&spi_handle->dr[0]), (void *)data.rx_buf, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                                 DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, data.rx_len);
            spi_handle->ser = 1U << chip_select;
            if(!cb)
            {
                dmac_wait_done(data.tx_channel);
                dmac_wait_done(data.rx_channel);
            }

            break;
        case SPI_TMOD_TRANS_RECV:
            spi_set_tmod(spi_num, SPI_TMOD_TRANS_RECV);
            if(data.rx_len > 65536)
                data.rx_len = 65536;

            if(cb)
            {
                if(data.tx_len > data.rx_len)
                {
                    dmac_irq_register(data.tx_channel, spi_dma_irq, &g_spi_instance[spi_num], cb->priority);
                    g_spi_instance[spi_num].dmac_channel = data.tx_channel;
                } else
                {
                    dmac_irq_register(data.rx_channel, spi_dma_irq, &g_spi_instance[spi_num], cb->priority);
                    g_spi_instance[spi_num].dmac_channel = data.rx_channel;
                }
            }
            spi_handle->ctrlr1 = (uint32_t)(data.rx_len - 1);
            spi_handle->dmacr = 0x3;
            spi_handle->ssienr = 0x01;
            sysctl_dma_select(data.tx_channel, SYSCTL_DMA_SELECT_SSI0_TX_REQ + spi_num * 2);
            if(data.fill_mode)
                dmac_set_single_mode(data.tx_channel, data.tx_buf, (void *)(&spi_handle->dr[0]), DMAC_ADDR_NOCHANGE, DMAC_ADDR_NOCHANGE,
                                     DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, data.tx_len);
            else
                dmac_set_single_mode(data.tx_channel, data.tx_buf, (void *)(&spi_handle->dr[0]), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                                     DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, data.tx_len);
            sysctl_dma_select(data.rx_channel, SYSCTL_DMA_SELECT_SSI0_RX_REQ + spi_num * 2);
            dmac_set_single_mode(data.rx_channel, (void *)(&spi_handle->dr[0]), (void *)data.rx_buf, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                                 DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, data.rx_len);
            spi_handle->ser = 1U << chip_select;
            if(!cb)
            {
                dmac_wait_done(data.tx_channel);
                dmac_wait_done(data.rx_channel);
            }
            break;
    }
    if(!cb)
    {
        nxmutex_unlock(&g_spi_instance[spi_num].lock);
        spi_handle->ser = 0x00;
        spi_handle->ssienr = 0x00;
    }
}
