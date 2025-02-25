/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 /***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v15.0
processor: LPC55S16
package_id: LPC55S16JBD100
mcu_data: ksdk2_0
processor_version: 15.0.0
board: LPCXpresso55S16
pin_labels:
- {pin_num: '81', pin_signal: PIO0_2/FC3_TXD_SCL_MISO_WS/CT_INP1/SCT0_OUT0/SCT_GPI2/SECURE_GPIO0_2, label: 'J10[20]/U24[12]/FC3_SPI_MISO', identifier: COL2}
- {pin_num: '83', pin_signal: PIO0_3/FC3_RXD_SDA_MOSI_DATA/CTIMER0_MAT1/SCT0_OUT1/SCT_GPI3/SECURE_GPIO0_3, label: 'J10[18]/U24[11]/FC3_SPI_MOSI', identifier: COL3}
- {pin_num: '86', pin_signal: PIO0_4/CAN0_RD/FC4_SCK/CT_INP12/SCT_GPI4/FC3_CTS_SDA_SSEL0/SECURE_GPIO0_4, label: 'J10[16]/U24[14]/FC3_SPI_SSEL0', identifier: COL4}
- {pin_num: '89', pin_signal: PIO0_6/FC3_SCK/CT_INP13/CTIMER4_MAT0/SCT_GPI6/SECURE_GPIO0_6, label: 'J10[14]/U24[13]/FC3_SPI_SCK', identifier: COL1}
- {pin_num: '71', pin_signal: PIO0_13/FC1_CTS_SDA_SSEL0/UTICK_CAP0/CT_INP0/SCT_GPI0/FC1_RXD_SDA_MOSI_DATA/PLU_IN0/SECURE_GPIO0_13, label: 'J13[9]/J13[10]/J18[8]/JS5[2]/FC1_I2C_SDA',
  identifier: ROW1}
- {pin_num: '72', pin_signal: PIO0_14/FC1_RTS_SCL_SSEL1/UTICK_CAP1/CT_INP1/SCT_GPI1/FC1_TXD_SCL_MISO_WS/PLU_IN1/SECURE_GPIO0_14, label: 'J13[11]/J13[12]/J18[6]/JS4[2]/FC1_I2C_SCL',
  identifier: ROW2}
- {pin_num: '58', pin_signal: PIO1_19/SCT0_OUT7/CTIMER3_MAT1/SCT_GPI7/FC4_SCK/PLU_OUT1/ACMPVREF, label: 'J18[4]/ISP_P1_19', identifier: ROW3}
- {pin_num: '73', pin_signal: PIO1_28/FC7_SCK/CT_INP2/PLU_IN3, label: 'J9[17]/J18[2]/EXP_ISP_P1_28', identifier: ROW4}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
    PinsFunc_keyscan();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '92', peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI_DATA/CTIMER2_MAT3/SCT0_OUT8/CMP0_OUT/PLU_OUT2/SECURE_GPIO0_29,
    mode: inactive, slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: '94', peripheral: FLEXCOMM0, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_30/FC0_TXD_SCL_MISO_WS/CTIMER0_MAT0/SCT0_OUT9/SECURE_GPIO0_30, mode: inactive,
    slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: '21', peripheral: SWD, signal: SWO, pin_signal: PIO0_10/FC6_SCK/CT_INP10/CTIMER2_MAT0/FC1_TXD_SCL_MISO_WS/SCT0_OUT2/SWO/SECURE_GPIO0_10/ADC0_1, mode: inactive,
    slew_rate: standard, invert: disabled, open_drain: disabled, asw: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 */
void BOARD_InitPins(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port0_pin10_config = (/* Pin is configured as SWO */
                                         IOCON_PIO_FUNC6 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* Analog switch is open (disabled) */
                                         IOCON_PIO_ASW_DI);
    /* PORT0 PIN10 (coords: 21) is configured as SWO */
    IOCON_PinMuxSet(IOCON, 0U, 10U, port0_pin10_config);

    const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                         IOCON_PIO_FUNC1 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN29 (coords: 92) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

    const uint32_t port0_pin30_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
                                         IOCON_PIO_FUNC1 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN30 (coords: 94) is configured as FC0_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
PinsFunc_keyscan:
- options: {callFromInitBoot: 'true', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '89', peripheral: GPIO, signal: 'PIO0, 6', pin_signal: PIO0_6/FC3_SCK/CT_INP13/CTIMER4_MAT0/SCT_GPI6/SECURE_GPIO0_6, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: '81', peripheral: GPIO, signal: 'PIO0, 2', pin_signal: PIO0_2/FC3_TXD_SCL_MISO_WS/CT_INP1/SCT0_OUT0/SCT_GPI2/SECURE_GPIO0_2, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: '83', peripheral: GPIO, signal: 'PIO0, 3', pin_signal: PIO0_3/FC3_RXD_SDA_MOSI_DATA/CTIMER0_MAT1/SCT0_OUT1/SCT_GPI3/SECURE_GPIO0_3, direction: OUTPUT,
    gpio_init_state: 'true'}
  - {pin_num: '86', peripheral: GPIO, signal: 'PIO0, 4', pin_signal: PIO0_4/CAN0_RD/FC4_SCK/CT_INP12/SCT_GPI4/FC3_CTS_SDA_SSEL0/SECURE_GPIO0_4, direction: OUTPUT,
    gpio_init_state: 'true'}
  - {pin_num: '71', peripheral: GPIO, signal: 'PIO0, 13', pin_signal: PIO0_13/FC1_CTS_SDA_SSEL0/UTICK_CAP0/CT_INP0/SCT_GPI0/FC1_RXD_SDA_MOSI_DATA/PLU_IN0/SECURE_GPIO0_13,
    direction: INPUT, mode: pullUp}
  - {pin_num: '72', peripheral: GPIO, signal: 'PIO0, 14', pin_signal: PIO0_14/FC1_RTS_SCL_SSEL1/UTICK_CAP1/CT_INP1/SCT_GPI1/FC1_TXD_SCL_MISO_WS/PLU_IN1/SECURE_GPIO0_14,
    direction: INPUT, mode: pullUp}
  - {pin_num: '58', peripheral: GPIO, signal: 'PIO1, 19', pin_signal: PIO1_19/SCT0_OUT7/CTIMER3_MAT1/SCT_GPI7/FC4_SCK/PLU_OUT1/ACMPVREF, direction: INPUT, mode: pullUp}
  - {pin_num: '73', peripheral: GPIO, signal: 'PIO1, 28', pin_signal: PIO1_28/FC7_SCK/CT_INP2/PLU_IN3, direction: INPUT, mode: pullUp}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : PinsFunc_keyscan
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 */
void PinsFunc_keyscan(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* Enables the clock for the GPIO0 module */
    CLOCK_EnableClock(kCLOCK_Gpio0);

    /* Enables the clock for the GPIO1 module */
    CLOCK_EnableClock(kCLOCK_Gpio1);

    gpio_pin_config_t COL2_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PIO0_2 (pin 81)  */
    GPIO_PinInit(PINSFUNC_KEYSCAN_COL2_GPIO, PINSFUNC_KEYSCAN_COL2_PORT, PINSFUNC_KEYSCAN_COL2_PIN, &COL2_config);

    gpio_pin_config_t COL3_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PIO0_3 (pin 83)  */
    GPIO_PinInit(PINSFUNC_KEYSCAN_COL3_GPIO, PINSFUNC_KEYSCAN_COL3_PORT, PINSFUNC_KEYSCAN_COL3_PIN, &COL3_config);

    gpio_pin_config_t COL4_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PIO0_4 (pin 86)  */
    GPIO_PinInit(PINSFUNC_KEYSCAN_COL4_GPIO, PINSFUNC_KEYSCAN_COL4_PORT, PINSFUNC_KEYSCAN_COL4_PIN, &COL4_config);

    gpio_pin_config_t COL1_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PIO0_6 (pin 89)  */
    GPIO_PinInit(PINSFUNC_KEYSCAN_COL1_GPIO, PINSFUNC_KEYSCAN_COL1_PORT, PINSFUNC_KEYSCAN_COL1_PIN, &COL1_config);

    gpio_pin_config_t ROW1_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_13 (pin 71)  */
    GPIO_PinInit(PINSFUNC_KEYSCAN_ROW1_GPIO, PINSFUNC_KEYSCAN_ROW1_PORT, PINSFUNC_KEYSCAN_ROW1_PIN, &ROW1_config);

    gpio_pin_config_t ROW2_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_14 (pin 72)  */
    GPIO_PinInit(PINSFUNC_KEYSCAN_ROW2_GPIO, PINSFUNC_KEYSCAN_ROW2_PORT, PINSFUNC_KEYSCAN_ROW2_PIN, &ROW2_config);

    gpio_pin_config_t ROW3_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_19 (pin 58)  */
    GPIO_PinInit(PINSFUNC_KEYSCAN_ROW3_GPIO, PINSFUNC_KEYSCAN_ROW3_PORT, PINSFUNC_KEYSCAN_ROW3_PIN, &ROW3_config);

    gpio_pin_config_t ROW4_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_28 (pin 73)  */
    GPIO_PinInit(PINSFUNC_KEYSCAN_ROW4_GPIO, PINSFUNC_KEYSCAN_ROW4_PORT, PINSFUNC_KEYSCAN_ROW4_PIN, &ROW4_config);

    IOCON->PIO[0][13] = ((IOCON->PIO[0][13] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT013 (pin 71) is configured as PIO0_13. */
                         | IOCON_PIO_FUNC(PIO0_13_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO0_13_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_13_DIGIMODE_DIGITAL));

    IOCON->PIO[0][14] = ((IOCON->PIO[0][14] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT014 (pin 72) is configured as PIO0_14. */
                         | IOCON_PIO_FUNC(PIO0_14_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO0_14_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_14_DIGIMODE_DIGITAL));

    IOCON->PIO[0][2] = ((IOCON->PIO[0][2] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT02 (pin 81) is configured as PIO0_2. */
                        | IOCON_PIO_FUNC(PIO0_2_FUNC_ALT0)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_2_DIGIMODE_DIGITAL));

    IOCON->PIO[0][3] = ((IOCON->PIO[0][3] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT03 (pin 83) is configured as PIO0_3. */
                        | IOCON_PIO_FUNC(PIO0_3_FUNC_ALT0)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_3_DIGIMODE_DIGITAL));

    IOCON->PIO[0][4] = ((IOCON->PIO[0][4] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT04 (pin 86) is configured as PIO0_4. */
                        | IOCON_PIO_FUNC(PIO0_4_FUNC_ALT0)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_4_DIGIMODE_DIGITAL));

    IOCON->PIO[0][6] = ((IOCON->PIO[0][6] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT06 (pin 89) is configured as PIO0_6. */
                        | IOCON_PIO_FUNC(PIO0_6_FUNC_ALT0)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_6_DIGIMODE_DIGITAL));

    IOCON->PIO[1][19] = ((IOCON->PIO[1][19] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT119 (pin 58) is configured as PIO1_19. */
                         | IOCON_PIO_FUNC(PIO1_19_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO1_19_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_19_DIGIMODE_DIGITAL));

    IOCON->PIO[1][28] = ((IOCON->PIO[1][28] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT128 (pin 73) is configured as PIO1_28. */
                         | IOCON_PIO_FUNC(PIO1_28_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO1_28_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_28_DIGIMODE_DIGITAL));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
