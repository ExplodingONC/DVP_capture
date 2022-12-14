/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
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
uint16_t frame_buff[4][72][194] = {};

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
void dvp_get_frame(PIO pio, uint sm, uint dma_chan)
{
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_set_enabled(pio, sm, true);

    dma_channel_set_write_addr(dma_chan, frame_buff, true);
}

/**
 * Stop DVP capturing, non-blocking
 */
void dvp_stop_frame(PIO pio, uint sm)
{
    pio_sm_set_enabled(pio, sm, false);
}

/**
 * Start/stop timing generator
 */
void clk_set_active(PIO pio, uint sm, bool active)
{
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_set_enabled(pio, sm, active);
}

/**
 * Main
 */
int main()
{
    stdio_init_all();
    printf("\nI/O initialized.\n");

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
    const PIO pio_clk = pio1;
    const uint sm_clk = 0;
    // init DVP input
    dvp_program_init(pio_dvp, sm_dvp, 1.f);
    dvp_dma_init(pio_dvp, sm_dvp, dma_dvp, count_of(frame_buff) / 2);
    printf("DVP input initialized.\n");
    // init CLK output
    clk_program_init(pio_clk, sm_clk, 1.f);
    printf("CLK output initialized.\n");
    clk_set_active(pio_clk, sm_clk, true);

    // init SPI slave
    spi_inst_t *spi_slave = spi0;
    int spi_speed;
    int spi_len;
    char command;
    spi_speed = spi_init(spi_slave, 1 * MHz);
    printf("SPI initialized at %dHz.\n", spi_speed);
    spi_set_slave(spi_slave, true);
    spi_speed = spi_set_baudrate(spi_slave, 1 * MHz);
    spi_set_format(spi_slave, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    printf("SPI change to slave at %dHz.\n", spi_speed);
    gpio_set_function(4, GPIO_FUNC_SPI);
    gpio_set_function(7, GPIO_FUNC_SPI);
    gpio_set_function(6, GPIO_FUNC_SPI);
    gpio_set_function(5, GPIO_FUNC_SPI);
    printf("SPI allocated to pin 4(MOSI), 7(MISO), 6(CLK), 5(SS).\n");

    while (true)
    {
        if (spi_is_readable(spi_slave))
        {
            // read SPI instruction
            spi_len = spi_read_blocking(spi_slave, 0, &command, 1);
            printf("SPI command received: 0x%02x. Length: %d.\n", command, spi_len);
            // get a frame
            while (gpio_get(FV_PIN))
            {
                // FV remains high though the whole frame (F1-F4)
                // discard on-going frame and wait for a fresh one
                // meanwhile fail the query
                if (spi_is_readable(spi_slave))
                {
                    spi_read_blocking(spi_slave, 0, &command, 1);
                    printf("SPI query refused: 0x%02x.\n", command);
                }
            }
            dvp_get_frame(pio_dvp, sm_dvp, dma_dvp);
            while (dma_channel_is_busy(dma_dvp))
            {
                // fail the query
                if (spi_is_readable(spi_slave))
                {
                    spi_read_blocking(spi_slave, 0, &command, 1);
                    printf("SPI query refused: 0x%02x.\n", command);
                }
            }
            dvp_stop_frame(pio_dvp, sm_dvp);
            printf("Frame data acquired. Size is %d.\n", sizeof(frame_buff));

            // temp test
            uint16_t* temp_p = &frame_buff[0][0][0];
            printf("temp: %d.\n", temp_p[0]);
            for (size_t i = 0; i < 255; i++)
            {
                temp_p[i] = (uint16_t)i & 0x00FF;
            }

            printf("0x%04x 0x%04x 0x%04x 0x%04x.\n", frame_buff[0][0][0], frame_buff[0][0][1], frame_buff[0][0][2], frame_buff[0][0][3]);
            printf("0x%04x 0x%04x 0x%04x 0x%04x.\n", frame_buff[1][0][0], frame_buff[1][0][1], frame_buff[1][0][2], frame_buff[1][0][3]);
            printf("0x%04x 0x%04x 0x%04x 0x%04x.\n", frame_buff[2][0][0], frame_buff[2][0][1], frame_buff[2][0][2], frame_buff[2][0][3]);
            printf("0x%04x 0x%04x 0x%04x 0x%04x.\n", frame_buff[3][0][0], frame_buff[3][0][1], frame_buff[3][0][2], frame_buff[3][0][3]);
            // accept the query
            spi_len = spi_read_blocking(spi_slave, 1, &command, 1);
            printf("Transfer confirmed.\n");
            // send result back
            spi_len = spi_write_blocking(spi_slave, (uint8_t *)frame_buff, 2 * sizeof(frame_buff));
            printf("SPI data sent. Length: %d.\n", spi_len);
        }
        else
        {
            // nothing to do
            sleep_ms(1);
        }
    }
}
