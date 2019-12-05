// A COMPLETER

/*
Service permettant de chornom�trer jusqu'� 59mn 59s 99 1/100
Utilise un timer au choix (TIMER1 � TIMER4).
Utilise la lib MyTimers.h /.c
*/



#include "Chrono.h"
#include "MyTimer.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_spi.h"

// variable priv�e de type Time qui m�morise la dur�e mesur�e
static Time Chrono_Time; // rem : static rend la visibilit� de la variable Chrono_Time limit�e � ce fichier 

// variable priv�e qui m�morise pour le module le timer utilis� par le module
static TIM_TypeDef * Chrono_Timer=TIM1; // init par d�faut au cas o� l'utilisateur ne lance pas Chrono_Conf avant toute autre fct.

// d�claration callback appel� toute les 10ms
void Chrono_Task_10ms(void);

/**
	* @brief  Configure le chronom�tre. 
  * @note   A lancer avant toute autre fonction.
	* @param  Timer : indique le timer � utiliser par le chronom�tre, TIM1, TIM2, TIM3 ou TIM4
  * @retval None
  */
void Chrono_Conf(TIM_TypeDef * Timer)
{
	// Reset Time
	Chrono_Time.Hund=0;
	Chrono_Time.Sec=0;
	Chrono_Time.Min=0;
	
	// Fixation du Timer
	Chrono_Timer=Timer;

	// R�glage Timer pour un d�bordement � 10ms
	MyTimer_Conf(Chrono_Timer,999, 719);
	
	// R�glage GPIOC 
	Chrono_Conf_io();
	
	
	// configuration de l'UART
	USART_Conf();
	
	// R�glage interruption du Timer avec callback : Chrono_Task_10ms()
	MyTimer_IT_Conf(Chrono_Timer, Chrono_Task_10ms,3);
	
	// Validation IT
	MyTimer_IT_Enable(Chrono_Timer);
	
}

void SPI_Conf(void){
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
	LL_SPI_Enable(SPI2);
	LL_SPI_SetMode(SPI2,LL_SPI_MODE_MASTER);
}

void USART_Conf (void){
	
	//uart clk enable
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	
	// init struct def
	LL_USART_InitTypeDef usart;
	
	
	// usart init value
	LL_USART_StructInit(&usart);
	usart.BaudRate =9600U;	 
	usart.DataWidth=LL_USART_DATAWIDTH_8B;
	usart.HardwareFlowControl=LL_USART_HWCONTROL_NONE;
	usart.Parity =LL_USART_PARITY_NONE; 
	usart.StopBits=LL_USART_STOPBITS_1;
	usart.TransferDirection = LL_USART_DIRECTION_TX;
	
	
	
	//uart init
	
	LL_USART_Init(USART2,&usart);
	LL_USART_Enable(USART2);
	
	//uart => GPIOA.2
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_2,LL_GPIO_MODE_ALTERNATE);
		
}

	
	//LL_USART_TransmitData8(USART2,'a');
	//LL_USART_EnableIT_TXE(USART2);
	

//}


/**
	* @brief  D�marre le chronom�tre. 
  * @note   si la dur�e d�passe 59mn 59sec 99 Hund, elle est remise � z�ro et repart
	* @param  Aucun
  * @retval Aucun
  */
void Chrono_Start(void)
{
	MyTimer_Start(Chrono_Timer);
}


/**
	* @brief  Arr�te le chronom�tre. 
  * @note   
	* @param  Aucun
  * @retval Aucun
  */
void Chrono_Stop(void)
{
	MyTimer_Stop(Chrono_Timer);
}


/**
	* @brief  Remet le chronom�tre � 0 
  * @note   
	* @param  Aucun
  * @retval Aucun
  */
void Chrono_Reset(void)
{
  // Arr�t Chrono
	MyTimer_Stop(Chrono_Timer);

	// Reset Time
	Chrono_Time.Hund=0;
	Chrono_Time.Sec=0;
	Chrono_Time.Min=0;
}


/**
	* @brief  Renvoie l'adresse de la variable Time priv�e g�r�e dans le module Chrono.c
  * @note   
	* @param  Aucun
  * @retval adresse de la variable Time
  */
Time * Chrono_Read(void)
{
	return &Chrono_Time;
}




/**
	* @brief  incr�mente la variable priv�e Chron_Time modulo 60mn 
  * @note   
	* @param  Aucun
  * @retval Aucun
  */
void Chrono_Task_10ms(void)
{ 
	Chrono_Time.Hund++;
	if (Chrono_Time.Hund==100)
	{
		Chrono_Time.Sec++;
		Chrono_Time.Hund=0;
		//Chrono_Background();
			// envoyer la lettre 'a'
		LL_USART_TransmitData8(USART2,'a');
	}
	if (Chrono_Time.Sec==60)
	{
		Chrono_Time.Min++;
		Chrono_Time.Sec=0;
	
	}
	if (Chrono_Time.Min==60)
	{
		Chrono_Time.Hund=0;
	}
}

void Chrono_Conf_io(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
	// config Start PC8 : Floating Input
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(GPIOC,  LL_GPIO_PIN_8, LL_GPIO_MODE_FLOATING);
	
	// config Stop PC6: Input Pulldown
	LL_GPIO_SetPinMode(GPIOC,  LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOC,  LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);
	
	/*// Config LED PC10 : output pushpull
	LL_GPIO_SetPinMode(GPIOC,  LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_10,LL_GPIO_OUTPUT_PUSHPULL);
	*/

	// Config LED PC10 : output opendrain
	LL_GPIO_SetPinMode(GPIOC,  LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_10,LL_GPIO_OUTPUT_OPENDRAIN);
	
}

void Chrono_Background(void) 
{
	//if (LL_GPIO_GetPinOutputType(GPIOC, LL_GPIO_PIN_10)==LL_GPIO_OUTPUT_PUSHPULL)
	//{
		// En Pushpull: rallumer la LED lorsque le PIN8 = 1
//		if ((LL_GPIO_ReadInputPort(GPIOC) & (1<< 8)) == (1<< 8) ) {
//			LL_GPIO_SetOutputPin(GPIOC,  LL_GPIO_PIN_10 );
//		}
//		else {
//			LL_GPIO_ResetOutputPin (GPIOC,  LL_GPIO_PIN_10 );
//		}
	//}	
	//else {
		// En Opendrain : �taindre la LED lorsque le PIN8 = 1
		
		if ((LL_GPIO_ReadInputPort(GPIOC) & (1<< 8)) == (1<< 8) ) {
			LL_GPIO_ResetOutputPin(GPIOC,  LL_GPIO_PIN_10 );
		}
		else {
			LL_GPIO_SetOutputPin (GPIOC,  LL_GPIO_PIN_10 );
	}
//}
}

