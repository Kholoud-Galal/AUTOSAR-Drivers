 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Kholoud Galal
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H


/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)


/******************************************************************************
 *                      Pre-Compiler Configurations                           *
 ******************************************************************************/
   
/* Pre-compile Configuration for Development Error Detect API*/
#define PORT_DEV_ERROR_DETECT                           (STD_ON)

/* Pre-compile Configuration for Version Info API */
#define PORT_VERSION_INFO_API                           (STD_ON)
   
/* pre-compile Configuration for Port Set Pin DIRECTION API */
#define PORT_SET_PIN_DIRECTION_API                      (STD_ON)
   
/* Pre-compile Configuration for Pin Direction Changeable API */
#define PORT_PIN_DIRECTION_CHANGEABLE_API               (STD_ON)
   
/* Number of port In the Microcontroller */
#define PORT_CONFIGURED_NUMBERS                         (6U)

/* Number of Configured Pins In the Microcontroller */
#define PORT_CONFIGURED_PINS                            (48U)

/******************************************************************************
 *                      ECU [PORT/PIN/MODE] Index                              *
 ******************************************************************************/

/* name of Port-Pin in the Microcontroller */ 
/*PORT INDEXS*/
#define PORT_A_ID                                  (0U)
#define PORT_B_ID                                  (1U)
#define PORT_C_ID                                  (2U)
#define PORT_D_ID                                  (3U)
#define PORT_E_ID                                  (4U)
#define PORT_F_ID                                  (5U)

/*PIN INDEXS*/
#define PIN_A0_ID                                  (Port_PinType)0
#define PIN_A1_ID                                  (Port_PinType)1
#define PIN_A2_ID                                  (Port_PinType)2
#define PIN_A3_ID                                  (Port_PinType)3
#define PIN_A4_ID                                  (Port_PinType)4
#define PIN_A5_ID                                  (Port_PinType)5
#define PIN_A6_ID                                  (Port_PinType)6
#define PIN_A7_ID                                  (Port_PinType)7
#define PIN_B0_ID                                  (Port_PinType)8
#define PIN_B1_ID                                  (Port_PinType)9
#define PIN_B2_ID                                  (Port_PinType)10
#define PIN_B3_ID                                  (Port_PinType)11
#define PIN_B4_ID                                  (Port_PinType)12
#define PIN_B5_ID                                  (Port_PinType)13
#define PIN_B6_ID                                  (Port_PinType)14
#define PIN_B7_ID                                  (Port_PinType)15
#define PIN_C0_ID                                  (Port_PinType)16
#define PIN_C1_ID                                  (Port_PinType)17
#define PIN_C2_ID                                  (Port_PinType)18
#define PIN_C3_ID                                  (Port_PinType)19
#define PIN_C4_ID                                  (Port_PinType)20
#define PIN_C5_ID                                  (Port_PinType)21
#define PIN_C6_ID                                  (Port_PinType)22
#define PIN_C7_ID                                  (Port_PinType)23
#define PIN_D0_ID                                  (Port_PinType)24
#define PIN_D1_ID                                  (Port_PinType)25
#define PIN_D2_ID                                  (Port_PinType)26
#define PIN_D3_ID                                  (Port_PinType)27
#define PIN_D4_ID                                  (Port_PinType)28
#define PIN_D5_ID                                  (Port_PinType)29
#define PIN_D6_ID                                  (Port_PinType)30
#define PIN_D7_ID                                  (Port_PinType)31
#define PIN_E0_ID                                  (Port_PinType)32
#define PIN_E1_ID                                  (Port_PinType)33
#define PIN_E2_ID                                  (Port_PinType)34
#define PIN_E3_ID                                  (Port_PinType)35
#define PIN_E4_ID                                  (Port_PinType)36
#define PIN_E5_ID                                  (Port_PinType)37
#define PIN_E6_ID                                  (Port_PinType)38
#define PIN_E7_ID                                  (Port_PinType)39
#define PIN_F0_ID                                  (Port_PinType)40
#define PIN_F1_ID                                  (Port_PinType)41
#define PIN_F2_ID                                  (Port_PinType)42
#define PIN_F3_ID                                  (Port_PinType)43
#define PIN_F4_ID                                  (Port_PinType)44
#define PIN_F5_ID                                  (Port_PinType)45
#define PIN_F6_ID                                  (Port_PinType)46
#define PIN_F7_ID                                  (Port_PinType)47

/*PIN DIFFERENT MODES INDEX*/
#define DIO_MODE		                   (Port_PinModeType)0
#define ADC_MODE		                   (Port_PinModeType)1
#define UART_MODE		                   (Port_PinModeType)2
#define USB_MODE		                   (Port_PinModeType)3
#define I2C_MODE		                   (Port_PinModeType)4
#define CAN_MODE		                   (Port_PinModeType)5
#define PWM_MODE		                   (Port_PinModeType)6
#define SSI_MODE		                   (Port_PinModeType)7
#define QEI_MODE		                   (Port_PinModeType)8
#define GPT_MODE		                   (Port_PinModeType)9   
#define NMI_MODE		                   (Port_PinModeType)10
#define ANALOG_COMPARATOR_MODE	                   (Port_PinModeType)11
                         
#endif /* PORT_CFG_H */
