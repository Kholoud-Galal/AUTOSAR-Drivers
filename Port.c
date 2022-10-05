 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Kholoud Galal
 ******************************************************************************/


#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif
/*variable that indicate port state*/
STATIC uint8 Port_State = PORT_NOT_INITIALIZED;
/*pointer to the configuration structure*/
STATIC  const Port_ConfigPins * ptr_PortConfiguration = NULL_PTR;


/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]:0x00
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initializes the Port module
*       
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr )
{
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	 if (ConfigPtr == NULL_PTR) 
            {
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID, PORT_INIT_ID,PORT_E_PARAM_CONFIG);
	    } 
         else
#endif
            { 
    /* point to the required Port Registers base address */
    volatile uint32 * PortGpio_Ptr = NULL_PTR; 
    volatile uint32 delay = 0;
    
    /*points to the first element of the array of structure*/
    ptr_PortConfiguration = ConfigPtr->Pins; 
    
    /* Update port statement*/    
    Port_State = PORT_INITIALIZED;
    
    /* index to configure each pin (43)*/
    uint8 Pin_ID;                
    
  /*loop on all microntroller port pins,and configure them all*/  
 for (Pin_ID = 0; Pin_ID < PORT_CONFIGURED_PINS  ; Pin_ID++) 
     {
         /* select the required Port Registers base address */
          switch(ptr_PortConfiguration[Pin_ID].port_num)
                {
                       case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		                break; 
	               case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		                break;
	               case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		                break;
	               case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		                break;
                       case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		                break;
                       case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		                break;
                }
    
            /* Enable clock for selected Port and allow time for clock to start*/
          SYSCTL_REGCGC2_REG |= (1<<ptr_PortConfiguration[Pin_ID].port_num);
          delay = SYSCTL_REGCGC2_REG;
          
          /* check if the required pins is PD7 or PF0 to unlock the commit register for them*/
          if( ((ptr_PortConfiguration[Pin_ID].port_num== PORT_D_ID) && (ptr_PortConfiguration[Pin_ID].pin_num== PIN_D7_ID)) || ((ptr_PortConfiguration[Pin_ID].port_num == PORT_F_ID) && (ptr_PortConfiguration[Pin_ID].pin_num == PIN_F0_ID)) ) 
            {
                 /* Unlock the PORT LOCK register */ 
                 *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;   
                 /* Set the corresponding bit in PORT COMMIT register to allow changes on this pin */
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);  
            }
          
          /* check if the required pins are PC0 to PC3 */
         else if( (ptr_PortConfiguration[Pin_ID].port_num == PORT_C_ID) && (ptr_PortConfiguration[Pin_ID].pin_num <= PIN_C3_ID) ) 
            {
                  /* Do Nothing ...  this is the JTAG pins */
            }
         else
            {
                 /* Do Nothing ... No need to unlock the commit register for this pin */
            }
    /*****************************************************************************Direction**********************************************************************************/
         /*check if the required direction for this pin is OUTPUT*/
         if(ptr_PortConfiguration[Pin_ID].direction == PORT_PIN_OUT)
           {
                /*Set the corresponding bit in the PORT DIRECTION register to configure it as output pin */
	        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);                
            /*check if the required initial level for this pin is High*/
               /*********************************Initial Value*******************************/ 
            if(ptr_PortConfiguration[Pin_ID].initial_level == STD_HIGH)
              {
                  /*Set the corresponding bit in the PORT DATA register to give it initial level 1 */
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);          
              }
            /*check if the required initial level for this pin is Low*/
            else
              {
                  /*Clear the corresponding bit in the PORT DATA register to give it initial level 0 */
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);        
              }
          }
          /*check if the required direction for this pin is INPUT*/
         else if(ptr_PortConfiguration[Pin_ID].direction == PORT_PIN_IN)
          {
              /*Clear the corresponding bit in the PORT DIRECTION register to configure it as input pin */
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);             
            /*check if the required resistor for this pin is pull up*/
              /*********************************Internal Resistor*******************************/ 
            if(ptr_PortConfiguration[Pin_ID].resistor == INTERNAL_PULL_UP)
              {
                 /*Set the corresponding bit in the PORT PULL UP register to give it internal pull up*/
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);       
              }
            /*check if the required resistor for this pin is pull down*/
            else if(ptr_PortConfiguration[Pin_ID].resistor == INTERNAL_PULL_DOWN)
              {
                 /*Set the corresponding bit in the PORT PULL DOWN register to give it internal pull down*/
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);     
              }
             /*check if the required resistor for this pin is disabled*/
            else
              {
                 /*Clear the corresponding bit in the PORT PULL UP and PORT PULL DOWN registers to Disable them*/
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);     
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);   
              }
         }
        else
         {
             /* Do Nothing */
         }
    
    /**********************************************************************Mode****************************************************************************/
     if(ptr_PortConfiguration[Pin_ID].pin_mode==DIO_MODE)
       { 
            /* Clear the corresponding bit in the PORT ANALOG MODE register to disable analog functionality of this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num); 
            /* Clear the corresponding bit in ALTERNATIVE register to Disable Alternative function for this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);   
            /* Clear the CONTROL register bits for this pin-->[DIO MODE] */ 
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ptr_PortConfiguration[Pin_ID].pin_num * 4)); 
            /* Set the corresponding bit in the PORT DIGITAL ENABLE register to enable digital functionality of this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);        
       }
       
     else if(ptr_PortConfiguration[Pin_ID].pin_mode==ADC_MODE)
        {  
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [ADC MODE] */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);               
             /* Clear the corresponding bit in the PORT DIGITAL ENABLE register to disable digital functionality of this pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);    
             /* Set the corresponding bit in the PORT ANALOG MODE register to Enable analog functionality of this pin */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);   
              /*In other port pins, to enable ADC functionality of them give PORT CONTROL register value equal to -->(F) */
             *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ptr_PortConfiguration[Pin_ID].pin_num * 4));     
        }
        
     else if(ptr_PortConfiguration[Pin_ID].pin_mode == UART_MODE)
       { 
            /* Clear the corresponding bit in the PORT ANALOG MODE register to disable analog functionality of this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
            /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [UART MODE] */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);               
             /* Set the corresponding bit in the PORT DIGITAL ENABLE register to enable digital functionality of this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
              /*Check if the required pin is C4 OR C5 because we have to Give PORT CONTROL register value equal to -->(2) to enable UART functionality in these pins*/
              if(Pin_ID==PIN_C4_ID||Pin_ID==PIN_C5_ID) 
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (ptr_PortConfiguration[Pin_ID].pin_num * 4));
                }
             else
               /*In other port pins that support this mode, to enable UART functionality of them give PORT CONTROL register value equal to -->(1) */
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (ptr_PortConfiguration[Pin_ID].pin_num * 4));
                }
         
        }
         
       else if(ptr_PortConfiguration[Pin_ID].pin_mode==USB_MODE)
       {
           /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [USB MODE] */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
           /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);        
           /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
           /*In port pins that support this mode, to enable USB functionality of them give PORT CONTROL register value equal to -->(8) */
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008<< (ptr_PortConfiguration[Pin_ID].pin_num * 4));
       }   
       
       else if(ptr_PortConfiguration[Pin_ID].pin_mode==I2C_MODE)
       {
            /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [I2C MODE] */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
            /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
            /*In port pins that support this mode, to enable I2C functionality of them give PORT CONTROL register value equal to -->(3) */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (ptr_PortConfiguration[Pin_ID].pin_num * 4));  
       }
       
       else if(ptr_PortConfiguration[Pin_ID].pin_mode==CAN_MODE)
         {
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [CAN MODE] */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num); 
             /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
             /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);        
                /*Check if the required pin is F0 OR F3 because we have to Give PORT CONTROL register value equal to -->(3) to enable CAN functionality in these pins*/
               if(Pin_ID==PIN_F0_ID ||Pin_ID==PIN_F3_ID)
                 {
                      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (ptr_PortConfiguration[Pin_ID].pin_num * 4));
                 }
               /*In other port pins that support this mode, to enable CAN functionality of them give PORT CONTROL register value equal to -->(8) */
              else
                {
                      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (ptr_PortConfiguration[Pin_ID].pin_num * 4));         
                }
         }
        
        else if(ptr_PortConfiguration[Pin_ID].pin_mode==PWM_MODE)
          { 
         
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [PWM MODE] */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
            /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);        
            /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
               /*Check if the required pin is B4 OR B5 OR B6 OR B7 OR C4 OR C5 OR D0 OR D1 OR E4 OR E5 because we have to Give PORT CONTROL register value equal to -->(4) to enable PWM functionality in these pins*/
               if(Pin_ID==PIN_B4_ID ||Pin_ID==PIN_B5_ID ||Pin_ID==PIN_B6_ID ||Pin_ID==PIN_B7_ID||Pin_ID==PIN_C4_ID ||Pin_ID==PIN_C5_ID ||Pin_ID==PIN_D0_ID||Pin_ID==PIN_D1_ID|Pin_ID==PIN_E4_ID ||Pin_ID==PIN_E5_ID)
                  {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (ptr_PortConfiguration[Pin_ID].pin_num * 4));
                  }
                 /*In other port pins that support this mode, to enable PWM functionality of them give PORT CONTROL register value equal to -->(5) */
               else
                  {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005<< (ptr_PortConfiguration[Pin_ID].pin_num * 4));
                  }
         }        
      
        else if(ptr_PortConfiguration[Pin_ID].pin_mode==SSI_MODE)
          {   
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [SSI MODE] */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
             /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num); 
             /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
               /*Check if the required pin is D0 OR D1 OR D2 OR D3 because we have to Give PORT CONTROL register value equal to -->(1) to enable SSI functionality in these pins*/
               if(Pin_ID==PIN_D0_ID ||Pin_ID==PIN_D1_ID ||Pin_ID==PIN_D2_ID ||Pin_ID==PIN_D3_ID)
                 { 
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (ptr_PortConfiguration[Pin_ID].pin_num * 4));  
                 }
               /*In other port pins that support this mode, to enable SSI functionality of them give PORT CONTROL register value equal to -->(2) */
               else
                 {
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (ptr_PortConfiguration[Pin_ID].pin_num * 4)); 
                 }
         }
               
      else if(ptr_PortConfiguration[Pin_ID].pin_mode==QEI_MODE) 
       {  
           /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [QEI MODE] */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
           /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
            /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);     
           /*In port pins that support this mode, to enable QEI functionality of them give PORT CONTROL register value equal to -->(6) */
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006<< (ptr_PortConfiguration[Pin_ID].pin_num * 4));
       }
  
      else if(ptr_PortConfiguration[Pin_ID].pin_mode==GPT_MODE) 
       { 
          /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [GPT MODE] */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);               
          /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
          /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
          /*In port pins that support this mode, to enable GPT functionality of them give PORT CONTROL register value equal to -->(7) */
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007<< (ptr_PortConfiguration[Pin_ID].pin_num * 4));       
       } 
      else if(ptr_PortConfiguration[Pin_ID].pin_mode==NMI_MODE) 
       {  
          /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [NMI MODE] */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
          /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
          /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
          /*In port pins that support this mode, to enable NMI functionality of them give PORT CONTROL register value equal to -->(8) */
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008<< (ptr_PortConfiguration[Pin_ID].pin_num * 4));
       }  
  
       else if(ptr_PortConfiguration[Pin_ID].pin_mode==ANALOG_COMPARATOR_MODE) 
        {    
              /*Analog pins  C1-,C1+,C0+,C0-*/
              if(Pin_ID==PIN_C4_ID ||Pin_ID==PIN_C5_ID ||Pin_ID==PIN_C6_ID ||Pin_ID==PIN_C7_ID )  
                { 
                    /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [ANALOG COMPARATOR MODE] */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
                    /* Set the corresponding bit in the PORT ANALOG MODE register to Enable analog functionality of this pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);   
                    /* Clear the corresponding bit in the PORT DIGITAL ENABLE register to Disable digital functionality of this pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);  
                } 
              else
                {
                   /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [ANALOG COMPARATOR MODE] */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
                   /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);
                   /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num);      
                   /*In port pins that support this mode, to enable ANALOG COMPARATOR functionality of them give PORT CONTROL register value equal to -->(9) */
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000009<< (ptr_PortConfiguration[Pin_ID].pin_num * 4));
                }  
         }
       
        
        
       } 
        }       
}  

/******************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]:0x01
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): pin (Port Pin ID number), Direction (Port Pin Direction).
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction       
******************************************************************************************/

#if (PORT_SET_PIN_DIRECTION_API==STD_ON)   

void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{ 
 
  /* point to the required Port Registers base address */
  volatile uint32 * PortGpio_Ptr = NULL_PTR; 
  
  /* Variable to detect the det_error state */
  boolean Det_Error_State = FALSE;
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)
         /* check if port initialized or not before setting the pin direction*/
	if (Port_State == PORT_NOT_INITIALIZED)
	  {
	      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_ID,PORT_E_UNINIT);
              Det_Error_State = TRUE;
	  }
	else
	  {
               //DO NOTHING....
	  }
        /* check if the given pn parameter greater than the number of configured pins*/
	if (Pin >= PORT_CONFIGURED_PINS  )
	  {
	      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_ID,PORT_E_PARAM_PIN);
              Det_Error_State = TRUE;
	  } 
        else
	  {
             //DO NOTHING....
	  }
        /* check if changging pin direction for this pin is allowed or not*/
        if(ptr_PortConfiguration[Pin].Is_Pin_direction_Changable == STD_OFF)
          {
              Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_ID,PORT_E_DIRECTION_UNCHANGEABLE);
              Det_Error_State = TRUE;
          }
        else 
          {
             //DO NOTHING....
          }
#endif

        /* In case there are not any Det errors*/
	if(FALSE == Det_Error_State)
         {
             /*Update port state*/
	     Port_State = PORT_INITIALIZED;
	
	    switch(ptr_PortConfiguration[Pin].port_num)
                  {
                       case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		                break; 
	               case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		                break;
	               case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		                break;
	               case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		                break;
                       case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		                break;
                       case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		                break;
                  }
           /*check if the required pin direction is INPUT*/
	   if(Direction == PORT_PIN_IN)
             {
                    /*Clear the corresponding bit in the PORT DIRECTION registor to Configure the pin as input direction*/
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
             }
            /*check if the required pin direction is OUTPUT*/
	  else if(Direction == PORT_PIN_OUT)
             {
                    /*Set the corresponding bit in the PORT DIRECTION registor to Configure the pin as output direction*/
	            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num); 
             }

        
      }

}
#endif

/*******************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]:0x02
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshe port direction.
*******************************************************************************************/
void Port_RefreshPortDirection( void )
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  
        /* check if port initialized or not before refreshing port direction*/
	if (Port_State == PORT_NOT_INITIALIZED)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_ID,PORT_E_UNINIT);    
	}
	else 
        {
            //DO NOTHING....
        }
#endif       
	/* point to the required Port Registers base address */
        volatile uint32 * PortGpio_Ptr = NULL_PTR;
        
        /* index to configure each pin (43)*/  
        uint8 Pin_ID;             
     /*loop on all  microcontroller port pins and return them to their initial state*/   
     for (Pin_ID = 0; Pin_ID < PORT_CONFIGURED_PINS  ; Pin_ID++) 
     {
          switch(ptr_PortConfiguration[Pin_ID].port_num)
            {
                 case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		          break; 
	         case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		          break;
	         case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		          break;
	         case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		          break;
                 case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		          break;
                 case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		          break; 
            }
          
        /*check if the required pins are PC0 to PC3 */ 
        if( (ptr_PortConfiguration[Pin_ID].port_num == PORT_C_ID) && (ptr_PortConfiguration[Pin_ID].pin_num <= PIN_C3_ID) ) 
         {
                 /* Do Nothing ...  this is the JTAG pins */
         }
         
         /*Before refreshing ports direction, check if the direction of this pin is changable in run-time or not*/
         /*we can only refresh port direction if the pin is changable in Run-time*/
        if(ptr_PortConfiguration[Pin_ID].Is_Pin_direction_Changable == STD_OFF)  
          {
            /*check if the initial direction is output direction*/
            if(ptr_PortConfiguration[Pin_ID].direction == PORT_PIN_OUT)
              {
                 /*Set the corresponding bit in the PORT DIRECTION registor to Configure the pin as Output direction*/
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num); 
              }
            else if(ptr_PortConfiguration[Pin_ID].direction == PORT_PIN_IN)
              {
                 /*Clear the corresponding bit in the PORT DIRECTION registor to Configure the pin as input direction*/
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ptr_PortConfiguration[Pin_ID].pin_num); 
              }
            else 
              {
                   //DO NOTHING....
              }
    
          }
       /*check If the Port pin is changable in Run-time exclude it*/
     else if(ptr_PortConfiguration[Pin_ID].Is_Pin_direction_Changable == STD_ON)
      {   
             // DO NOTHING....
      }
    
    }  
} 

/******************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]:0x03
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): versioninfo which is Pointer to the version information of this module.
* Return value: None
* Description: Returns the version information of this module.       
******************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
	void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
	{
	
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        /* Check if the pointer is a Null pointer */
	if(NULL_PTR == versioninfo)
	  {
	        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,PORT_GET_VERSION_INFO_ID ,PORT_E_PARAM_POINTER);
	  }
	else
	  {
                  //DO NOTHING....
          }
                  
         /* Check if the port is initialized first */
	if(Port_State == PORT_NOT_INITIALIZED)
	  {
	      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_ID, PORT_E_UNINIT);
	  }
	else
	  {	
                  //DO NOTHING....	
          }
                
#endif
	        /* Get the vendor Id */
		versioninfo->vendorID =(uint16)PORT_VENDOR_ID; 
        
		/* Get the module Id */	
		versioninfo->moduleID =(uint16)PORT_MODULE_ID; 
                
		/* Get the SW major version */	
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION; 
                
		/* Get the SW minor version */	
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION; 
                
		/* Get the SW patch version */	
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION; 
		
	}
#endif

/**********************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]:0x04
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin (Port Pin ID number), Mode (New Port Pin mode to be set on port pin).
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the pin mode.
*******************************************************************************************/
                                
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )

{ 
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  
        /* Check if the Port is initialized before using this function */
        if (Port_State == PORT_NOT_INITIALIZED) 
	   {
	        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_ID,PORT_E_UNINIT);        
	   }
	else
	   {
                //DO NOTHING....
	   }
  
        /* check if Port Pin mode is not changable in Run-time */
	if(ptr_PortConfiguration[Pin].Is_Pin_mode_changable==STD_OFF)
           {
                Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_ID ,PORT_E_MODE_UNCHANGEABLE);
           }
       else
           {
              //DO NOTHING....
           }
        
        /* check if an Invalid Port Pin ID passed*/
        if (Pin >= NUM_OF_PIN_SINGLE_PORT )
	   {
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_ID,PORT_E_PARAM_PIN);
	   } 
        else
	   {
               //DO NOTHING....
	   }
        
         /* check if an invalid Port Pin ID passed */
        if(Pin >= PORT_CONFIGURED_PINS)
           {
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_ID, PORT_E_PARAM_PIN);
	   }
	else
	   {	
              //DO NOTHING....
           }
        
         /* check if an Invalid Port Pin Mode passed*/
        if ((Mode < DIO_MODE ) || (Mode > ANALOG_COMPARATOR_MODE))
	   {
                Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_ID ,PORT_E_PARAM_INVALID_MODE);
           } 
        else
           {
              //DO NOTHING....  
           }
#endif 
      
  /* point to the required Port Registers base address */
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  
  switch(ptr_PortConfiguration[Pin].port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break; 
	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }

     if(Mode==DIO_MODE)
       { 
            /* Clear the corresponding bit in the PORT ANALOG MODE register to disable analog functionality of this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num); 
            /* Clear the corresponding bit in ALTERNATIVE register to Disable Alternative function for this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);   
            /* Clear the CONTROL register bits for this pin-->[DIO MODE] */ 
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ptr_PortConfiguration[Pin].pin_num * 4)); 
            /* Set the corresponding bit in the PORT DIGITAL ENABLE register to enable digital functionality of this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);        
       }
    
          
     else if(Mode==ADC_MODE)
        {  
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [ADC MODE] */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);               
             /* Clear the corresponding bit in the PORT DIGITAL ENABLE register to disable digital functionality of this pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);    
             /* Set the corresponding bit in the PORT ANALOG MODE register to Enable analog functionality of this pin */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);   
              /*In other port pins, to enable ADC functionality of them give PORT CONTROL register value equal to -->(F) */
             *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ptr_PortConfiguration[Pin].pin_num * 4));     
        }  
  
     else if(Mode==UART_MODE)
       { 
            /* Clear the corresponding bit in the PORT ANALOG MODE register to disable analog functionality of this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
            /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [UART MODE] */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);               
             /* Set the corresponding bit in the PORT DIGITAL ENABLE register to enable digital functionality of this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
              /*Check if the required pin is C4 OR C5 because we have to Give PORT CONTROL register value equal to -->(2) to enable UART functionality in these pins*/
              if(Pin==PIN_C4_ID||Pin==PIN_C5_ID) 
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (ptr_PortConfiguration[Pin].pin_num * 4));
                }
             else
               /*In other port pins that support this mode, to enable UART functionality of them give PORT CONTROL register value equal to -->(1) */
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (ptr_PortConfiguration[Pin].pin_num * 4));
                }
         
        }

       else if(Mode==USB_MODE)
         {
           /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [USB MODE] */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
           /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);        
           /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
           /*In port pins that support this mode, to enable USB functionality of them give PORT CONTROL register value equal to -->(8) */
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008<< (ptr_PortConfiguration[Pin].pin_num * 4));
         }     
  
       else if(Mode==I2C_MODE)
         {
            /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [I2C MODE] */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
            /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
            /*In port pins that support this mode, to enable I2C functionality of them give PORT CONTROL register value equal to -->(3) */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (ptr_PortConfiguration[Pin].pin_num * 4));  
         }
  
       else if(Mode==CAN_MODE)
         {
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [CAN MODE] */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num); 
             /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
             /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);        
                /*Check if the required pin is F0 OR F3 because we have to Give PORT CONTROL register value equal to -->(3) to enable CAN functionality in these pins*/
               if(Pin==PIN_F0_ID ||Pin==PIN_F3_ID)
                 {
                      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (ptr_PortConfiguration[Pin].pin_num * 4));
                 }
               /*In other port pins that support this mode, to enable CAN functionality of them give PORT CONTROL register value equal to -->(8) */
              else
                {
                      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (ptr_PortConfiguration[Pin].pin_num * 4));         
                }
         }
        
        else if(Mode==PWM_MODE)
       { 
         
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [PWM MODE] */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
            /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);        
            /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
               /*Check if the required pin is B4 OR B5 OR B6 OR B7 OR C4 OR C5 OR D0 OR D1 OR E4 OR E5 because we have to Give PORT CONTROL register value equal to -->(4) to enable PWM functionality in these pins*/
               if(Pin==PIN_B4_ID ||Pin==PIN_B5_ID ||Pin==PIN_B6_ID ||Pin==PIN_B7_ID||Pin==PIN_C4_ID ||Pin==PIN_C5_ID ||Pin==PIN_D0_ID||Pin==PIN_D1_ID|Pin==PIN_E4_ID ||Pin==PIN_E5_ID)
                  {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (ptr_PortConfiguration[Pin].pin_num * 4));
                  }
                 /*In other port pins that support this mode, to enable PWM functionality of them give PORT CONTROL register value equal to -->(5) */
               else
                  {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005<< (ptr_PortConfiguration[Pin].pin_num * 4));
                  }
        }
  
        else if(Mode==SSI_MODE)
          {   
             /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [SSI MODE] */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
             /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num); 
             /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
               /*Check if the required pin is D0 OR D1 OR D2 OR D3 because we have to Give PORT CONTROL register value equal to -->(1) to enable SSI functionality in these pins*/
               if(Pin==PIN_D0_ID ||Pin==PIN_D1_ID ||Pin==PIN_D2_ID ||Pin==PIN_D3_ID)
                 { 
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (ptr_PortConfiguration[Pin].pin_num * 4));  
                 }
               /*In other port pins that support this mode, to enable SSI functionality of them give PORT CONTROL register value equal to -->(2) */
               else
                 {
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (ptr_PortConfiguration[Pin].pin_num * 4)); 
                 }
         }       
               
      else if(Mode==QEI_MODE) 
       {  
           /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [QEI MODE] */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
           /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
            /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);     
           /*In port pins that support this mode, to enable QEI functionality of them give PORT CONTROL register value equal to -->(6) */
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006<< (ptr_PortConfiguration[Pin].pin_num * 4));
       }
  
      else if(Mode==GPT_MODE) 
       { 
          /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [GPT MODE] */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);               
          /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
          /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
          /*In port pins that support this mode, to enable GPT functionality of them give PORT CONTROL register value equal to -->(7) */
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007<< (ptr_PortConfiguration[Pin].pin_num * 4));       
       } 
      else if(Mode==NMI_MODE) 
       {  
          /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [NMI MODE] */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
          /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
          /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
          /*In port pins that support this mode, to enable NMI functionality of them give PORT CONTROL register value equal to -->(8) */
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008<< (ptr_PortConfiguration[Pin].pin_num * 4));
       }  
  
       else if(Mode==ANALOG_COMPARATOR_MODE) 
        {    
              /*Analog pins  C1-,C1+,C0+,C0-*/
              if(Pin==PIN_C4_ID ||Pin==PIN_C5_ID ||Pin==PIN_C6_ID ||Pin==PIN_C7_ID )  
                { 
                    /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [ANALOG COMPARATOR MODE] */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
                    /* Set the corresponding bit in the PORT ANALOG MODE register to Enable analog functionality of this pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);   
                    /* Clear the corresponding bit in the PORT DIGITAL ENABLE register to Disable digital functionality of this pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);  
                } 
              else
                {
                   /* Set the corresponding bit in ALTERNATIVE register to Enable Alternative function for this pin [ANALOG COMPARATOR MODE] */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
                   /* Set the corresponding bit in the PORT DIGITAL ENABLE register to Enable digital functionality of this pin */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);
                   /* Clear the corresponding bit in the PORT ANALOG MODE register to Disable analog functionality of this pin */
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr_PortConfiguration[Pin].pin_num);      
                   /*In port pins that support this mode, to enable ANALOG COMPARATOR functionality of them give PORT CONTROL register value equal to -->(9) */
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000009<< (ptr_PortConfiguration[Pin].pin_num * 4));
                }  
         }
       
  
        
 }