/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdbool.h>
#include "boards/custom_lidar.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "dvp_in.pio.h"

#define kHz 1000
#define MHz 1000000

// Data will be copied from gpio to this buffer via DMA
#define res_X 104
#define res_Y 80
#define N_data 2
uint16_t subframe_buff[N_data][res_Y][2 * res_X + 2] = {};
uint16_t frame_buff[4][res_Y][2 * res_X + 2] = {};

/**
 * extract subframe data into frame buff
 */
int data_process()
{
    int subframe = (subframe_buff[0][0][0] & 0x0300) >> 8;
    for (int y = 0; y < res_Y; y++)
    {
        frame_buff[subframe][y][0] = subframe_buff[0][y][0];
        frame_buff[subframe][y][1] = subframe_buff[0][y][1];
        for (int x = 2; x < 2 * res_X + 2; x++)
        {
            frame_buff[subframe][y][x] = subframe_buff[N_data-1][y][x] - subframe_buff[0][y][x];
        }
    }
    return subframe;
}

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

    dma_channel_set_write_addr(dma_chan, subframe_buff, true);
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
 * core 1 main code
 */
void core1_main(void)
{
    // init GPIO
    const uint FV_PIN = dvp_PIN_FV;
    gpio_init(FV_PIN);
    gpio_set_dir(FV_PIN, GPIO_IN);
    // allocate PIOs
    const PIO pio_dvp = pio0;
    const uint sm_dvp = 0;
    const uint dma_dvp = 0;
    // init DVP input
    uint dma_trans_count = sizeof(subframe_buff) / sizeof(uint16_t) / 2;
    dvp_program_init(pio_dvp, sm_dvp, 1.f);
    dvp_dma_init(pio_dvp, sm_dvp, dma_dvp, dma_trans_count);
    printf("DVP input initialized.\n");

    while (1)
    {
        multicore_fifo_pop_blocking();
        // frame capture - arm
        dvp_arm_frame(pio_dvp, sm_dvp, dma_dvp);
        // process last subframe
        int last_subframe = data_process();
        multicore_fifo_push_timeout_us(last_subframe, 10);
        // wait for new frame start
        while (!gpio_get(FV_PIN))
            ;
        // wait for new frame end
        while (gpio_get(FV_PIN) || dma_channel_is_busy(dma_dvp))
            ;
        // frame capture - terminate
        dvp_stop_frame(pio_dvp, sm_dvp);
    }
}

/**
 * Main
 */
int main()
{
    stdio_init_all();
    printf("\nBuild version 20230303-1730\n");
    printf("I/O initialized.\n");
    printf("Subframe size: %d bytes or %d pixels.\n", sizeof(subframe_buff), sizeof(subframe_buff) / sizeof(uint16_t));
    printf("Fullframe size: %d bytes or %d pixels.\n", sizeof(frame_buff), sizeof(frame_buff) / sizeof(uint16_t));

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

    // start core 1
    multicore_reset_core1();
    multicore_launch_core1(core1_main);
    sleep_ms(10);
    multicore_fifo_push_blocking(0); // first run
    printf("Core 1 online?\n");

    while (true)
    {
        int subframe = multicore_fifo_pop_blocking();
        multicore_fifo_push_blocking(0); // continue
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
            printf("Frame required with cmd 0x%02X.\n", command);
            if (command == 0x00)
                continue;
        }

        // get designated subframe
        for (int i = 0; i < 3; i++)
        {
            multicore_fifo_pop_blocking();
            multicore_fifo_push_blocking(0); // continue
        }
        multicore_fifo_pop_blocking();
        printf("Frame fetched.\n");

        // alert master about ready subframe
        ack = 0x11;
        spi_write_blocking(spi_slave, &ack, 1);
        printf("Frame ready.\n");

        spi_len = 0;
        for (size_t subframe = 0; subframe < 4; subframe++)
        {
            for (size_t row = 0; row < res_Y; row++)
            {
                spi_len += spi_write_blocking(spi_slave, (uint8_t *)frame_buff[subframe][row], 2 * (2 * res_X + 2));
            }
        }
        printf("Frame sent. Length: %d bytes.\n", spi_len);
        multicore_fifo_push_blocking(0); // continue
    }
}
