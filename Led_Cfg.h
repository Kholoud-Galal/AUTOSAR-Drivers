 /******************************************************************************
 *
 * Module: Led
 *
 * File Name: Led_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Led Driver
 *
 * Author: Kholoud Galal
 ******************************************************************************/

#ifndef LED_CFG_H
#define LED_CFG_H

#include "std_Types.h"

/* Set the led ON/OFF according to its configuration Positive logic or negative logic */
#define LED_ON        STD_HIGH

#define LED_OFF       STD_LOW

/* Set the LED Port */
#define LED_PORT       DioConf_LED1_PORT_NUM

/* Set the LED Pin Number */
#define LED_PIN_NUM    DioConf_LED1_CHANNEL_NUM


#endif /* LED_CFG_H */
