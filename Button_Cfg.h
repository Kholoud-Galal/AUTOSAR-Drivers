 /******************************************************************************
 *
 * Module: Button
 *
 * File Name: Button_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Button Driver
 *
 * Author: Kholoud Galal
 ******************************************************************************/

#ifndef BUTTON_CFG_H
#define BUTTON_CFG_H

#include "std_Types.h"

/* Button State according to its configuration PULL UP/Down */
#define BUTTON_PRESSED           STD_LOW

#define BUTTON_RELEASED          STD_HIGH

/* Set the Button Port */
#define BUTTON_PORT              DioConf_SW1_PORT_NUM

/* Set the Button Pin Number */
#define BUTTON_PIN_NUM           DioConf_SW1_CHANNEL_NUM


#endif /* BUTTON_CFG_H */
