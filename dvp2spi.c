/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "dvp2spi.pio.h"

#define kHz 1000
#define MHz 1000000

// Data will be copied from gpio to buffer
uint16_t frame_buff[97 * 2 * 72 * 4];

void dvp_dma_init(PIO pio, uint sm, uint dma_chan, size_t dma_length)
{
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
                          NULL,          // Destination pointer, set later
                          &pio->rxf[sm], // Source pointer
                          dma_length,    // Number of transfers
                          false          // Start immediately
    );
}

void dvp_get_frame(PIO pio, uint sm, uint dma_chan)
{
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_set_enabled(pio, sm, true);

    dma_channel_set_write_addr(dma_chan, frame_buff, true);
    dma_channel_wait_for_finish_blocking(dma_chan);
}

void clk_set_active(PIO pio, uint sm, bool active)
{
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_set_enabled(pio, sm, active);
}

int main()
{
    stdio_init_all();
    printf("\nI/O initialized.\n");

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
    printf("Bus priority set.\n");

    const PIO pio_dvp = pio0;
    const uint sm_dvp = 0;
    const uint dma_dvp = 0;
    const PIO pio_clk = pio1;
    const uint sm_clk = 0;

    dvp_program_init(pio_dvp, sm_dvp, 1.f);
    dvp_dma_init(pio_dvp, sm_dvp, dma_dvp, count_of(frame_buff) / 2);
    printf("DVP input initialized.\n");
    // dvp_get_frame(pio_dvp, sm_dvp, dma_chan);

    clk_program_init(pio_clk, sm_clk, 1.f);
    printf("CLK output initialized.\n");
    clk_set_active(pio_clk, sm_clk, true);

    while (true)
    {
        sleep_ms(1); //
    }
}
