/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdbool.h>
#include "boards/custom_lidar.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "dvp2spi.pio.h"

#define kHz 1000
#define MHz 1000000

// Data will be copied from gpio to this buffer via DMA
#define res_X 104
#define res_Y 80
#define N_data 2
uint16_t frame_buff[N_data][res_Y][2 * res_X + 2] = {};

/**
 * Associate DVP-PIO to a DMA channel
 */
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

/**
 * Start DVP capturing, non-blocking
 * Query its state with dma_channel_is_busy()
 */
void dvp_arm_frame(PIO pio, uint sm, uint dma_chan)
{
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_set_write_addr(dma_chan, frame_buff, true);
    pio_sm_exec(pio, sm, pio_encode_wait_gpio(1, dvp_PIN_FV));
    pio_sm_set_enabled(pio, sm, true);
}

/**
 * Stop DVP capturing, non-blocking
 */
void dvp_stop_frame(PIO pio, uint sm)
{
    pio_sm_set_enabled(pio, sm, false);
}

/**
 * Main
 */
int main()
{
    stdio_init_all();
    printf("\nI/O initialized.\n");
    printf("Buffer size: %d bytes or %d pixels.\n", sizeof(frame_buff), sizeof(frame_buff)/sizeof(uint16_t));

    // init GPIO
    const uint FV_PIN = dvp_PIN_FV;
    gpio_init(FV_PIN);
    gpio_set_dir(FV_PIN, GPIO_IN);

    // set bus priority to DMA
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
    printf("Bus priority set.\n");

    // allocate PIOs
    const PIO pio_dvp = pio0;
    const uint sm_dvp = 0;
    const uint dma_dvp = 0;
    // init DVP input
    uint dma_trans_count = sizeof(frame_buff) / sizeof(uint16_t) / 2;
    dvp_program_init(pio_dvp, sm_dvp, 1.f);
    dvp_dma_init(pio_dvp, sm_dvp, dma_dvp, dma_trans_count);
    printf("DVP input initialized.\n");

    while (true)
    {
        // discard on-going frame
        while (gpio_get(FV_PIN));
        // frame capture - arm
        dvp_arm_frame(pio_dvp, sm_dvp, dma_dvp);
        // wait for new frame start
        while (!gpio_get(FV_PIN));
        // wait for new frame end
        while (gpio_get(FV_PIN) || dma_channel_is_busy(dma_dvp));
        // frame capture - terminate
        dvp_stop_frame(pio_dvp, sm_dvp);
        // check frame size by reading DMA transfer count
        if (dma_channel_hw_addr(dma_dvp)->transfer_count == 0)
            printf("Frame data acquired. Size is %d bytes.\n", sizeof(frame_buff));
        else
            printf("Frame incomplete! Remaining %d words.\n", dma_channel_hw_addr(dma_dvp)->transfer_count);

        // print key datas (data headers + pixel samples)
        printf("0x%04x 0x%04x 0x%04x 0x%04x.\n", frame_buff[0][0][0], frame_buff[0][0][1], frame_buff[0][0][2], frame_buff[0][0][3]);
        printf("0x%04x 0x%04x 0x%04x 0x%04x.\n", frame_buff[0][1][0], frame_buff[0][1][1], frame_buff[0][1][2], frame_buff[0][1][3]);
        printf("0x%04x 0x%04x 0x%04x 0x%04x.\n", frame_buff[1][0][0], frame_buff[1][0][1], frame_buff[1][0][2], frame_buff[1][0][3]);
        printf("0x%04x 0x%04x 0x%04x 0x%04x.\n", frame_buff[1][1][0], frame_buff[1][1][1], frame_buff[1][1][2], frame_buff[1][1][3]);
        // zzz
        sleep_ms(1000);
        
    }
}
