/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdbool.h>
#include "boards/custom_lidar.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
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
 *
 */
uint32_t dvp_get_frame(PIO pio_dvp, uint sm_dvp, uint dma_dvp, uint FV_PIN)
{
    // discard on-going frame
    while (gpio_get(FV_PIN))
        ;
    // frame capture - arm
    dvp_arm_frame(pio_dvp, sm_dvp, dma_dvp);
    // wait for new frame start
    while (!gpio_get(FV_PIN))
        ;
    // wait for new frame end
    while (gpio_get(FV_PIN) || dma_channel_is_busy(dma_dvp))
        ;
    // frame capture - terminate
    dvp_stop_frame(pio_dvp, sm_dvp);
    // check frame size by reading DMA transfer count
    return dma_channel_hw_addr(dma_dvp)->transfer_count;
}

/**
 * Main
 */
int main()
{
    stdio_init_all();
    printf("\nI/O initialized.\n");
    printf("Buffer size: %d bytes or %d pixels.\n", sizeof(frame_buff), sizeof(frame_buff) / sizeof(uint16_t));

    // init GPIO
    const uint FV_PIN = dvp_PIN_FV;
    gpio_init(FV_PIN);
    gpio_set_dir(FV_PIN, GPIO_IN);

    // init SPI slave
    spi_inst_t *spi_slave = spi0;
    int spi_speed;
    int spi_len;
    char command, ack;
    spi_speed = spi_init(spi_slave, 1 * MHz);
    spi_set_slave(spi_slave, true);
    spi_speed = spi_set_baudrate(spi_slave, 1 * MHz);
    spi_set_format(spi_slave, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    printf("SPI initialized to slave at %dHz.\n", spi_speed);
    gpio_set_function(4, GPIO_FUNC_SPI);
    gpio_set_function(7, GPIO_FUNC_SPI);
    gpio_set_function(6, GPIO_FUNC_SPI);
    gpio_set_function(5, GPIO_FUNC_SPI);
    printf("SPI allocated to pin 4(MOSI), 7(MISO), 6(CLK), 5(SS).\n");

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
        // check if there is SPI communication
        if (!spi_is_readable(spi_slave))
        {
            // nothing to do, skip
            continue;
        }
        else
        {
            // get SPI command
            spi_read_blocking(spi_slave, 0x10, &command, 1);
            if (command == 0x00)
                continue;
        }

        // check which subframe the command wants
        char subframe = command & 0x0f;
        printf("Subframe F%1d required.\n", subframe);

        // get designated subframe
        do
        {
            dvp_get_frame(pio_dvp, sm_dvp, dma_dvp, FV_PIN);
        } while (subframe != (((frame_buff[0][0][0] & 0x0300) >> 8) + 1));
        printf("Subframe F%1d fetched.\n", subframe);

        // alert master about ready subframe
        ack = 0x10 | subframe;
        spi_write_blocking(spi_slave, &ack, 1);
        printf("Subframe F%1d ready.\n", subframe);

        spi_len = 0;
        for (size_t integr = 0; integr < N_data; integr++)
        {
            for (size_t row = 0; row < res_Y; row++)
            {
                spi_len += spi_write_blocking(spi_slave, (uint8_t *)frame_buff[integr][row], 2 * (2 * res_X + 2));
            }
        }
        printf("Subframe F%1d sent. Length: %d bytes.\n", subframe, spi_len);

        // zzz
        //sleep_ms(1000);
    }
}
