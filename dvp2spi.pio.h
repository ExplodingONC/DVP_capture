// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --- //
// dvp //
// --- //

#define dvp_wrap_target 0
#define dvp_wrap 4

#define dvp_PIN_BASE 8
#define dvp_PIN_COUNT 16
#define dvp_PIN_FV 21
#define dvp_PIN_LV 20
#define dvp_PIN_CLK 22

static const uint16_t dvp_program_instructions[] = {
            //     .wrap_target
    0x2016, //  0: wait   0 gpio, 22                 
    0x00c3, //  1: jmp    pin, 3                     
    0x0004, //  2: jmp    4                          
    0x4010, //  3: in     pins, 16                   
    0x2096, //  4: wait   1 gpio, 22                 
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program dvp_program = {
    .instructions = dvp_program_instructions,
    .length = 5,
    .origin = -1,
};

static inline pio_sm_config dvp_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + dvp_wrap_target, offset + dvp_wrap);
    return c;
}

static inline void dvp_program_init(PIO pio, uint sm, float div) {
    for(uint i=dvp_PIN_BASE; i<dvp_PIN_BASE+dvp_PIN_COUNT; i++) {
        pio_gpio_init(pio, i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, dvp_PIN_BASE, dvp_PIN_COUNT, false);
    uint offset = pio_add_program(pio, &dvp_program);
    pio_sm_config c = dvp_program_get_default_config(offset);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_in_pins(&c, dvp_PIN_BASE);
    sm_config_set_jmp_pin(&c, dvp_PIN_LV);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
}

#endif

// --- //
// clk //
// --- //

#define clk_wrap_target 0
#define clk_wrap 1

#define clk_PIN_BASE 28
#define clk_PIN_COUNT 1

static const uint16_t clk_program_instructions[] = {
            //     .wrap_target
    0xe001, //  0: set    pins, 1                    
    0xe000, //  1: set    pins, 0                    
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program clk_program = {
    .instructions = clk_program_instructions,
    .length = 2,
    .origin = -1,
};

static inline pio_sm_config clk_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + clk_wrap_target, offset + clk_wrap);
    return c;
}

static inline void clk_program_init(PIO pio, uint sm, float div) {
    for(uint i=clk_PIN_BASE; i<clk_PIN_BASE+clk_PIN_COUNT; i++) {
        pio_gpio_init(pio, i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, clk_PIN_BASE, clk_PIN_COUNT, true);
    uint offset = pio_add_program(pio, &clk_program);
    pio_sm_config c = clk_program_get_default_config(offset);
    sm_config_set_set_pins(&c, clk_PIN_BASE, clk_PIN_COUNT);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
}

#endif

