
#ifndef  __Led_Eval_h__
#define  __Led_Eval_h__

#include "main.h"

/*************LED**********************/
#define LEDn 7

//------LED0------//
#define  LED0_GPIO_PORT	      GPIOA
#define  LED0_GPIO_PIN	      GPIO_Pin_4
//------LED1------//
#define  LED1_GPIO_PORT	      GPIOA
#define  LED1_GPIO_PIN	      GPIO_Pin_5
//------LED2------//
#define  LED2_GPIO_PORT	      GPIOA
#define  LED2_GPIO_PIN	      GPIO_Pin_6
//------LED3------//
#define  LED3_GPIO_PORT	      GPIOA
#define  LED3_GPIO_PIN	      GPIO_Pin_7
//------LED4------//
#define  LED4_GPIO_PORT	      GPIOC
#define  LED4_GPIO_PIN	      GPIO_Pin_4
//------STN------//
#define  STN_RST_GPIO_PORT	  GPIOA
#define  STN_RST_GPIO_PIN	  GPIO_Pin_12
//------CHIPEN------//
#define  CHIPEN_GPIO_PORT	  GPIOA
#define  CHIPEN_GPIO_PIN	  GPIO_Pin_8

typedef enum 
{
	LED0   = 0,
	LED1   = 1,
    LED2   = 2,
	LED3   = 3,
	LED4   = 4,
    STN_RST   = 5,
    CHIPEN    = 6
} Led_TypeDef;

typedef enum 
{
	On = 0,
	Off = 1,
	Toggle=2
} Led_Status;


/**************************************/

/***************Function**************************/
void LED_Init(void);
void LED_Control(Led_TypeDef Led,Led_Status Status);

#endif

