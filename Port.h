 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Kholoud Galal
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Kholoud Galal's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Dio Module Id */
#define PORT_MODULE_ID    (120U)

/* Dio Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)


/* Number of Pins in the Single Port (MCU Specific) */
#define NUM_OF_PIN_SINGLE_PORT         (8U)


/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* service ID for PORT init */
#define PORT_INIT_ID                       (uint8)0x00

/* service ID for PORT set pin direction */
#define PORT_SET_PIN_DIRECTION_ID          (uint8)0x01

/* service ID for PORT refresh pin direction */
#define PORT_REFRESH_PORT_DIRECTION_ID     (uint8)0x02

/* service ID for PORT get version info */
#define PORT_GET_VERSION_INFO_ID           (uint8)0x03

/* service ID for PORT set pin mode */
#define PORT_SET_PIN_MODE_ID               (uint8)0x04


/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN                   (uint8)0x0A
   
/* DET Code for API Port_SetPinDirection service called when direction is unchangeable */
#define PORT_E_DIRECTION_UNCHANGEABLE      (uint8)0x0B
   
/* DET Code for API Port_Init called with invalid parameter */
#define PORT_E_PARAM_CONFIG                (uint8)0x0C

/* DET Code for API Port_SetPinMode service called with invalid Mode */
#define PORT_E_PARAM_INVALID_MODE          (uint8)0x0D

/* DET Code for API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE           (uint8)0x0E

/* DET Code for API service called if module initialization */
#define PORT_E_UNINIT                      (uint8)0x0F

/* DET Code for APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER               (uint8)0x10

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Data type for the port pin */
typedef uint8 Port_PinType;

/* Data type for Different port pin modes */
typedef uint8 Port_PinModeType;

/* Data type for the level of port pin */
typedef uint8 Port_PinLevelType;

/* Directions of a port pin */
typedef enum{
  PORT_PIN_IN,          /* Set port pin as input */
  PORT_PIN_OUT,         /* Set port pin as output */
}Port_PinDirectionType;


/* Internal resistor types for Port Pin */
typedef enum
{
    DISABLE_INTERNAL_RESISTOR,
    INTERNAL_PULL_UP,
    INTERNAL_PULL_DOWN
}Port_InternalResistorType;

/*  #Structure to configure each individual PIN:
 *      1. Port Number -->[A,B,C,D,E,F].
 *      2. Pin Number -->[A0,A1,A2,A3,A4,A5,A6,A7......F7].
 *	3. Direction of pin -->[INPUT Direction/OUTPUT Direction].
 *      4. Pin Mode -->[DIO/ADC/UART/ .... PWM].
 *      5. Initial value -->[STD_HIGH/STD_LOW].
 *      6. Internal resistor -->[Disabled/Internal Pull up/Internal Pull down].
 *      7. Pin direction changeability -->[STD_HIGH/STD_LOW].
 *      8. Pin mode changeability -->[STD_HIGH/STD_LOW].
 */
typedef struct 
{
    uint8 port_num;                       
    Port_PinType pin_num; 
    Port_PinDirectionType direction;         
    Port_PinModeType pin_mode;                        
    Port_PinLevelType initial_level;                   
    Port_InternalResistorType resistor;           
    uint8 Is_Pin_direction_Changable;              
    uint8 Is_Pin_mode_changable;                      
    
}Port_ConfigPins;

/* structure for initialization API */
typedef struct
{
	Port_ConfigPins Pins[PORT_CONFIGURED_PINS];     
}Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* function for Port initialization API */
void Port_Init( const Port_ConfigType* ConfigPtr );

/* function for setting Port pin direction API */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)

void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction );

#endif

/* function for refreshing Port direction API */
void Port_RefreshPortDirection( void );

/* function for getting version info for this module API */
#if (PORT_VERSION_INFO_API == STD_ON)

void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );

#endif

/* function for setting Port pin mode API */
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern the structure to be used by Port and other modules */
extern const Port_ConfigType Port_pinConfiguration;

#endif /* PORT_H */
