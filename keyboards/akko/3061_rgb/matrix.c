/*
Copyright 2021 Adam Honse <calcprogrammer1@gmail.com>
Copyright 2011 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Ported to QMK by Stephen Peery <https://github.com/smp4488/>
*/

#include <stdint.h>
#include <stdbool.h>
#include <SN32F240B.h>
#include "ch.h"
#include "hal.h"
#include "CT16.h"

#include "color.h"
#include "wait.h"
#include "util.h"
#include "matrix.h"
#include "debounce.h"
#include "quantum.h"

static const pin_t row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;
static const pin_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;
static const pin_t led_row_pins[LED_MATRIX_ROWS_HW] = LED_MATRIX_ROW_PINS;
static const pin_t led_col_pins[LED_MATRIX_COLS] = LED_MATRIX_COL_PINS;
static uint16_t row_ofsts[LED_MATRIX_ROWS];
static uint8_t mr_offset[24] = {0};
static uint32_t pwm_en_msk = 0;

matrix_row_t raw_matrix[MATRIX_ROWS]; //raw values
matrix_row_t last_matrix[MATRIX_ROWS] = {0};  // raw values
matrix_row_t matrix[MATRIX_ROWS]; //debounced values

static bool matrix_changed = false;
static uint8_t current_row = 0;
extern volatile LED_TYPE led_state[LED_MATRIX_ROWS * LED_MATRIX_COLS];

__attribute__((weak)) void matrix_init_kb(void) { matrix_init_user(); }

__attribute__((weak)) void matrix_scan_kb(void) { matrix_scan_user(); }

__attribute__((weak)) void matrix_init_user(void) {}

__attribute__((weak)) void matrix_scan_user(void) {}

inline matrix_row_t matrix_get_row(uint8_t row) { return matrix[row]; }

void matrix_print(void) {}

static void init_pins(void) {
#if(DIODE_DIRECTION == ROW2COL)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinInput(row_pins[x]);
        writePinHigh(row_pins[x]);
    }
#elif(DIODE_DIRECTION == COL2ROW)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinOutput(row_pins[x]);
        writePinHigh(row_pins[x]);
    }
#else
#error DIODE_DIRECTION must be one of COL2ROW or ROW2COL!
#endif

    // Unselect COLs
    for (uint8_t x = 0; x < MATRIX_COLS; x++) {
        setPinOutput(col_pins[x]);
        writePinHigh(col_pins[x]);
    }

   for (uint8_t x = 0; x < LED_MATRIX_ROWS_HW; x++) {
        setPinOutput(led_row_pins[x]);
        writePinHigh(led_row_pins[x]);
   }
}

void matrix_init(void) {
    // initialize key pins
    init_pins();

    // initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        raw_matrix[i] = 0;
        matrix[i]     = 0;
    }

    for (uint8_t i = 0; i < LED_MATRIX_ROWS; i++) {
        row_ofsts[i] = i * LED_MATRIX_COLS;
    }

    debounce_init(MATRIX_ROWS);

    matrix_init_quantum();

    // Enable Timer Clock
    SN_SYS1->AHBCLKEN_b.CT16B1CLKEN = 1;

    // PFPA - Map all PWM outputs to their PWM A pins
    // Pins will be remapped to their PWM B pins if present in the LED column pin list
    SN_PFPA->CT16B1 = 0x00000000;

    pwm_en_msk = 0;
    
    // Enable PWM function, IOs and select the PWM modes for the LED column pins
    for(uint8_t i = 0; i < LED_MATRIX_COLS; i++) {
        switch(led_col_pins[i]) {
            // Intentional fall-through for the PWM B-pin mapping
            case B8:
                SN_PFPA->CT16B1 |= mskCT16_PWM0EN_EN;
            case A0:
                pwm_en_msk |= mskCT16_PWM0EN_EN;
                mr_offset[0] = i;
                break;

            case B9:
                SN_PFPA->CT16B1 |= mskCT16_PWM1EN_EN;
            case A1:
                pwm_en_msk |= mskCT16_PWM1EN_EN;
                mr_offset[1] = i;
                break;
            
            case B10:
                SN_PFPA->CT16B1 |= mskCT16_PWM2EN_EN;
            case A2:
                pwm_en_msk |= mskCT16_PWM2EN_EN;
                mr_offset[2] = i;
                break;

            case B11:
                SN_PFPA->CT16B1 |= mskCT16_PWM3EN_EN;
            case A3:
                pwm_en_msk |= mskCT16_PWM3EN_EN;
                mr_offset[3] = i;
                break;

            case B12:
                SN_PFPA->CT16B1 |= mskCT16_PWM4EN_EN;
            case A4:
                pwm_en_msk |= mskCT16_PWM4EN_EN;
                mr_offset[4] = i;
                break;

            case B13:
                SN_PFPA->CT16B1 |= mskCT16_PWM5EN_EN;
            case A5:
                pwm_en_msk |= mskCT16_PWM5EN_EN;
                mr_offset[5] = i;
                break;

            case B14:
                SN_PFPA->CT16B1 |= mskCT16_PWM6EN_EN;
            case A6:
                pwm_en_msk |= mskCT16_PWM6EN_EN;
                mr_offset[6] = i;
                break;

            case B15:
                SN_PFPA->CT16B1 |= mskCT16_PWM7EN_EN;
            case A7:
                pwm_en_msk |= mskCT16_PWM7EN_EN;
                mr_offset[7] = i;
                break;

            case C0:
                SN_PFPA->CT16B1 |= mskCT16_PWM8EN_EN;
            case A8:
                pwm_en_msk |= mskCT16_PWM8EN_EN;
                mr_offset[8] = i;
                break;

            case C1:
                SN_PFPA->CT16B1 |= mskCT16_PWM9EN_EN;
            case A9:
                pwm_en_msk |= mskCT16_PWM9EN_EN;
                mr_offset[9] = i;
                break;

            case C2:
                SN_PFPA->CT16B1 |= mskCT16_PWM10EN_EN;
            case A10:
                pwm_en_msk |= mskCT16_PWM10EN_EN;
                mr_offset[10] = i;
                break;

            case C3:
                SN_PFPA->CT16B1 |= mskCT16_PWM11EN_EN;
            case A11:
                pwm_en_msk |= mskCT16_PWM11EN_EN;
                mr_offset[11] = i;
                break;

            case C4:
                SN_PFPA->CT16B1 |= mskCT16_PWM12EN_EN;
            case A12:
                pwm_en_msk |= mskCT16_PWM12EN_EN;
                mr_offset[12] = i;
                break;

            case C5:
                SN_PFPA->CT16B1 |= mskCT16_PWM13EN_EN;
            case A13:
                pwm_en_msk |= mskCT16_PWM13EN_EN;
                mr_offset[13] = i;
                break;

            case C6:
                SN_PFPA->CT16B1 |= mskCT16_PWM14EN_EN;
            case A14:
                pwm_en_msk |= mskCT16_PWM14EN_EN;
                mr_offset[14] = i;
                break;

            case C7:
                SN_PFPA->CT16B1 |= mskCT16_PWM15EN_EN;
            case A15:
                pwm_en_msk |= mskCT16_PWM15EN_EN;
                mr_offset[15] = i;
                break;

            case C8:
                SN_PFPA->CT16B1 |= mskCT16_PWM16EN_EN;
            case B0:
                pwm_en_msk |= mskCT16_PWM16EN_EN;
                mr_offset[16] = i;
                break;

            case C9:
                SN_PFPA->CT16B1 |= mskCT16_PWM17EN_EN;
            case B1:
                pwm_en_msk |= mskCT16_PWM17EN_EN;
                mr_offset[17] = i;
                break;

            case C10:
                SN_PFPA->CT16B1 |= mskCT16_PWM18EN_EN;
            case B2:
                pwm_en_msk |= mskCT16_PWM18EN_EN;
                mr_offset[18] = i;
                break;

            case C11:
                SN_PFPA->CT16B1 |= mskCT16_PWM19EN_EN;
            case B3:
                pwm_en_msk |= mskCT16_PWM19EN_EN;
                mr_offset[19] = i;
                break;

            case C12:
                SN_PFPA->CT16B1 |= mskCT16_PWM20EN_EN;
            case B4:
                pwm_en_msk |= mskCT16_PWM20EN_EN;
                mr_offset[20] = i;
                break;

            case C13:
                SN_PFPA->CT16B1 |= mskCT16_PWM21EN_EN;
            case B5:
                pwm_en_msk |= mskCT16_PWM21EN_EN;
                mr_offset[21] = i;
                break;

            case C14:
                SN_PFPA->CT16B1 |= mskCT16_PWM22EN_EN;
            case B6:
                pwm_en_msk |= mskCT16_PWM22EN_EN;
                mr_offset[22] = i;
                break;

            case C15:
                SN_PFPA->CT16B1 |= mskCT16_PWM23EN_EN;
            case B7:
                pwm_en_msk |= mskCT16_PWM23EN_EN;
                mr_offset[23] = i;
                break;
        }
    }

    SN_CT16B1->PWMENB   |= pwm_en_msk;
    SN_CT16B1->PWMIOENB |= pwm_en_msk;

    // Set match interrupts and TC rest
    SN_CT16B1->MCTRL3 = (mskCT16_MR24IE_EN | mskCT16_MR24STOP_EN);

    // COL match register
    SN_CT16B1->MR24 = 0xFF;

    // Set prescale value
    SN_CT16B1->PRE = 0x10;

    //Set CT16B1 as the up-counting mode.
	SN_CT16B1->TMRCTRL = (mskCT16_CRST);

    // Wait until timer reset done.
    while (SN_CT16B1->TMRCTRL & mskCT16_CRST);

    // Let TC start counting.
    SN_CT16B1->TMRCTRL |= mskCT16_CEN_EN;

    NVIC_ClearPendingIRQ(CT16B1_IRQn);
    nvicEnableVector(CT16B1_IRQn, 0);
}

uint8_t matrix_scan(void) {
    matrix_changed = false;
    for (uint8_t current_col = 0; current_col < MATRIX_COLS; current_col++) {
        for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
            // Determine if the matrix changed state
            if ((last_matrix[row_index] != raw_matrix[row_index])) {
                matrix_changed         = true;
                last_matrix[row_index] = raw_matrix[row_index];
            }
        }
    }

    debounce(raw_matrix, matrix, MATRIX_ROWS, matrix_changed);

    matrix_scan_quantum();

    return matrix_changed;
}

/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_CT16B1_HANDLER) {

    chSysDisable();

    OSAL_IRQ_PROLOGUE();

    // Disable PWM outputs on column pins
    SN_CT16B1->PWMIOENB = 0;

    SN_CT16B1->IC = mskCT16_MR24IC; // Clear match interrupt status

    // Turn the selected row off
    writePinLow(led_row_pins[current_row]);

    // Turn the next row on
    current_row = (current_row + 1) % LED_MATRIX_ROWS_HW;

#if(DIODE_DIRECTION == ROW2COL)
    if(current_row == 0)
    {
        // Read the key matrix
        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
            // Enable the column
            writePinLow(col_pins[col_index]);

            for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
                // Check row pin state
                if (readPin(row_pins[row_index]) == 0) {
                    // Pin LO, set col bit
                    raw_matrix[row_index] |= (MATRIX_ROW_SHIFTER << col_index);
                } else {
                    // Pin HI, clear col bit
                    raw_matrix[row_index] &= ~(MATRIX_ROW_SHIFTER << col_index);
                }
            }

            // Disable the column
            writePinHigh(col_pins[col_index]);
        }
    }
#elif(DIODE_DIRECTION == COL2ROW)
    if(current_row == 0)
    {
        // Set all column pins input high
        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
            setPinInputHigh(col_pins[col_index]);
        }

        // Read the key matrix
        for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
            // Enable the row
            for (int i = 0; i < 32; i++) {
                writePinLow(row_pins[row_index]);
            }

            for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
                // Check row pin state
                if (readPin(col_pins[col_index]) == 0) {
                    // Pin LO, set col bit
                    raw_matrix[row_index] |= (MATRIX_ROW_SHIFTER << col_index);
                } else {
                    // Pin HI, clear col bit
                    raw_matrix[row_index] &= ~(MATRIX_ROW_SHIFTER << col_index);
                }
            }

            // Disable the row
            writePinHigh(row_pins[row_index]);
        }
    }
#endif

    uint8_t row_idx = ( current_row / 3 );
    uint16_t row_ofst = row_ofsts[row_idx];

    //Set CT16B1 as the up-counting mode.
    SN_CT16B1->TMRCTRL = (mskCT16_CRST);

    // Wait until timer reset done.
    while (SN_CT16B1->TMRCTRL & mskCT16_CRST);

    if(current_row % 3 == 0)
    {
        SN_CT16B1->MR0  = led_state[row_ofst + mr_offset[0] ].r;
        SN_CT16B1->MR1  = led_state[row_ofst + mr_offset[1] ].r;
        SN_CT16B1->MR2  = led_state[row_ofst + mr_offset[2] ].r;
        SN_CT16B1->MR3  = led_state[row_ofst + mr_offset[3] ].r;
        SN_CT16B1->MR4  = led_state[row_ofst + mr_offset[4] ].r;
        SN_CT16B1->MR5  = led_state[row_ofst + mr_offset[5] ].r;
        SN_CT16B1->MR6  = led_state[row_ofst + mr_offset[6] ].r;
        SN_CT16B1->MR7  = led_state[row_ofst + mr_offset[7] ].r;
        SN_CT16B1->MR8  = led_state[row_ofst + mr_offset[8] ].r;
        SN_CT16B1->MR9  = led_state[row_ofst + mr_offset[9] ].r;
        SN_CT16B1->MR10 = led_state[row_ofst + mr_offset[10]].r;
        SN_CT16B1->MR11 = led_state[row_ofst + mr_offset[11]].r;
        SN_CT16B1->MR12 = led_state[row_ofst + mr_offset[12]].r;
        SN_CT16B1->MR13 = led_state[row_ofst + mr_offset[13]].r;
        SN_CT16B1->MR14 = led_state[row_ofst + mr_offset[14]].r;
        SN_CT16B1->MR15 = led_state[row_ofst + mr_offset[15]].r;
        SN_CT16B1->MR16 = led_state[row_ofst + mr_offset[16]].r;
        SN_CT16B1->MR17 = led_state[row_ofst + mr_offset[17]].r;
        SN_CT16B1->MR18 = led_state[row_ofst + mr_offset[18]].r;
        SN_CT16B1->MR19 = led_state[row_ofst + mr_offset[19]].r;
        SN_CT16B1->MR20 = led_state[row_ofst + mr_offset[20]].r;
        SN_CT16B1->MR21 = led_state[row_ofst + mr_offset[21]].r;
        SN_CT16B1->MR22 = led_state[row_ofst + mr_offset[22]].r;
        SN_CT16B1->MR23 = led_state[row_ofst + mr_offset[23]].r;
    }

    if(current_row % 3 == 1)
    {
        SN_CT16B1->MR0  = led_state[row_ofst + mr_offset[0] ].b;
        SN_CT16B1->MR1  = led_state[row_ofst + mr_offset[1] ].b;
        SN_CT16B1->MR2  = led_state[row_ofst + mr_offset[2] ].b;
        SN_CT16B1->MR3  = led_state[row_ofst + mr_offset[3] ].b;
        SN_CT16B1->MR4  = led_state[row_ofst + mr_offset[4] ].b;
        SN_CT16B1->MR5  = led_state[row_ofst + mr_offset[5] ].b;
        SN_CT16B1->MR6  = led_state[row_ofst + mr_offset[6] ].b;
        SN_CT16B1->MR7  = led_state[row_ofst + mr_offset[7] ].b;
        SN_CT16B1->MR8  = led_state[row_ofst + mr_offset[8] ].b;
        SN_CT16B1->MR9  = led_state[row_ofst + mr_offset[9] ].b;
        SN_CT16B1->MR10 = led_state[row_ofst + mr_offset[10]].b;
        SN_CT16B1->MR11 = led_state[row_ofst + mr_offset[11]].b;
        SN_CT16B1->MR12 = led_state[row_ofst + mr_offset[12]].b;
        SN_CT16B1->MR13 = led_state[row_ofst + mr_offset[13]].b;
        SN_CT16B1->MR14 = led_state[row_ofst + mr_offset[14]].b;
        SN_CT16B1->MR15 = led_state[row_ofst + mr_offset[15]].b;
        SN_CT16B1->MR16 = led_state[row_ofst + mr_offset[16]].b;
        SN_CT16B1->MR17 = led_state[row_ofst + mr_offset[17]].b;
        SN_CT16B1->MR18 = led_state[row_ofst + mr_offset[18]].b;
        SN_CT16B1->MR19 = led_state[row_ofst + mr_offset[19]].b;
        SN_CT16B1->MR20 = led_state[row_ofst + mr_offset[20]].b;
        SN_CT16B1->MR21 = led_state[row_ofst + mr_offset[21]].b;
        SN_CT16B1->MR22 = led_state[row_ofst + mr_offset[22]].b;
        SN_CT16B1->MR23 = led_state[row_ofst + mr_offset[23]].b;
    }

    if(current_row % 3 == 2)
    {
        SN_CT16B1->MR0  = led_state[row_ofst + mr_offset[0] ].g;
        SN_CT16B1->MR1  = led_state[row_ofst + mr_offset[1] ].g;
        SN_CT16B1->MR2  = led_state[row_ofst + mr_offset[2] ].g;
        SN_CT16B1->MR3  = led_state[row_ofst + mr_offset[3] ].g;
        SN_CT16B1->MR4  = led_state[row_ofst + mr_offset[4] ].g;
        SN_CT16B1->MR5  = led_state[row_ofst + mr_offset[5] ].g;
        SN_CT16B1->MR6  = led_state[row_ofst + mr_offset[6] ].g;
        SN_CT16B1->MR7  = led_state[row_ofst + mr_offset[7] ].g;
        SN_CT16B1->MR8  = led_state[row_ofst + mr_offset[8] ].g;
        SN_CT16B1->MR9  = led_state[row_ofst + mr_offset[9] ].g;
        SN_CT16B1->MR10 = led_state[row_ofst + mr_offset[10]].g;
        SN_CT16B1->MR11 = led_state[row_ofst + mr_offset[11]].g;
        SN_CT16B1->MR12 = led_state[row_ofst + mr_offset[12]].g;
        SN_CT16B1->MR13 = led_state[row_ofst + mr_offset[13]].g;
        SN_CT16B1->MR14 = led_state[row_ofst + mr_offset[14]].g;
        SN_CT16B1->MR15 = led_state[row_ofst + mr_offset[15]].g;
        SN_CT16B1->MR16 = led_state[row_ofst + mr_offset[16]].g;
        SN_CT16B1->MR17 = led_state[row_ofst + mr_offset[17]].g;
        SN_CT16B1->MR18 = led_state[row_ofst + mr_offset[18]].g;
        SN_CT16B1->MR19 = led_state[row_ofst + mr_offset[19]].g;
        SN_CT16B1->MR20 = led_state[row_ofst + mr_offset[20]].g;
        SN_CT16B1->MR21 = led_state[row_ofst + mr_offset[21]].g;
        SN_CT16B1->MR22 = led_state[row_ofst + mr_offset[22]].g;
        SN_CT16B1->MR23 = led_state[row_ofst + mr_offset[23]].g;
    }

    uint32_t new_pwm_en = 0;
    if(SN_CT16B1->MR0 > 0)
    {
        new_pwm_en |= mskCT16_PWM0EN_EN;
    }
    if(SN_CT16B1->MR1 > 0)
    {
        new_pwm_en |= mskCT16_PWM1EN_EN;
    }
    if(SN_CT16B1->MR2 > 0)
    {
        new_pwm_en |= mskCT16_PWM2EN_EN;
    }
    if(SN_CT16B1->MR3 > 0)
    {
        new_pwm_en |= mskCT16_PWM3EN_EN;
    }
    if(SN_CT16B1->MR4 > 0)
    {
        new_pwm_en |= mskCT16_PWM4EN_EN;
    }
    if(SN_CT16B1->MR5 > 0)
    {
        new_pwm_en |= mskCT16_PWM5EN_EN;
    }
    if(SN_CT16B1->MR6 > 0)
    {
        new_pwm_en |= mskCT16_PWM6EN_EN;
    }
    if(SN_CT16B1->MR7 > 0)
    {
        new_pwm_en |= mskCT16_PWM7EN_EN;
    }
    if(SN_CT16B1->MR8 > 0)
    {
        new_pwm_en |= mskCT16_PWM8EN_EN;
    }
    if(SN_CT16B1->MR9 > 0)
    {
        new_pwm_en |= mskCT16_PWM9EN_EN;
    }
    if(SN_CT16B1->MR10 > 0)
    {
        new_pwm_en |= mskCT16_PWM10EN_EN;
    }
    if(SN_CT16B1->MR11 > 0)
    {
        new_pwm_en |= mskCT16_PWM11EN_EN;
    }
    if(SN_CT16B1->MR12 > 0)
    {
        new_pwm_en |= mskCT16_PWM12EN_EN;
    }
    if(SN_CT16B1->MR13 > 0)
    {
        new_pwm_en |= mskCT16_PWM13EN_EN;
    }
    if(SN_CT16B1->MR14 > 0)
    {
        new_pwm_en |= mskCT16_PWM14EN_EN;
    }
    if(SN_CT16B1->MR15 > 0)
    {
        new_pwm_en |= mskCT16_PWM15EN_EN;
    }
    if(SN_CT16B1->MR16 > 0)
    {
        new_pwm_en |= mskCT16_PWM16EN_EN;
    }
    if(SN_CT16B1->MR17 > 0)
    {
        new_pwm_en |= mskCT16_PWM17EN_EN;
    }
    if(SN_CT16B1->MR18 > 0)
    {
        new_pwm_en |= mskCT16_PWM18EN_EN;
    }
    if(SN_CT16B1->MR19 > 0)
    {
        new_pwm_en |= mskCT16_PWM19EN_EN;
    }
    if(SN_CT16B1->MR20 > 0)
    {
        new_pwm_en |= mskCT16_PWM20EN_EN;
    }
    if(SN_CT16B1->MR21 > 0)
    {
        new_pwm_en |= mskCT16_PWM21EN_EN;
    }
    if(SN_CT16B1->MR22 > 0)
    {
        new_pwm_en |= mskCT16_PWM22EN_EN;
    }
    if(SN_CT16B1->MR23 > 0)
    {
        new_pwm_en |= mskCT16_PWM23EN_EN;
    }

    SN_CT16B1->PWMIOENB = pwm_en_msk & new_pwm_en;

    // Set match interrupts and TC rest
    SN_CT16B1->MCTRL3 = (mskCT16_MR24IE_EN | mskCT16_MR24STOP_EN);

    writePinHigh(led_row_pins[current_row]);

    // Let TC start counting.
    SN_CT16B1->TMRCTRL |= mskCT16_CEN_EN;

    chSysEnable();

    OSAL_IRQ_EPILOGUE();
}