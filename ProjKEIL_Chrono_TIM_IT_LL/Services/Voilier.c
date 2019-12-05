#include "Chrono.h"
#include "MyTimer.h"
#include "Voilier.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_tim.h" 
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_adc.h"
#define POURCENTAGE_MAX 13.7
#define POURCENTAGE_MIN 8.6


// Configurer la girouette
void GirouetteConfig(void){
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Enable clock TIM4
	TIM3->PSC = 0;
	TIM3->ARR = 1440;
	TIM3->CR1 |=TIM_CR1_CEN; //Allumer TIM3
	
	TIM3->SMCR |=(0x3 << TIM_SMCR_SMS_Pos);
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA->CRL &= ~(0xF<<20);
	GPIOA->CRL |=  (0x4 <<20);//floating
}


// Configurer le servo voile
void ServoConfig(void){
	
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //Enable clock TIM4
	TIM1->PSC = 300;
	TIM1->ARR = 4784;
	TIM1->BDTR = TIM_BDTR_MOE;
	TIM1->CR1 |=TIM_CR1_CEN; //Allumer TIM4
	
	TIM1->CCMR1 &= ~(0xF << 4);
	TIM1->CCMR1 |=  (0x6 << 4);
	TIM1->CCER |= 0x1;
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //Enable clock GPIOA
	GPIOA->CRH &= ~0xF;
	GPIOA->CRH |= 0xB;
}


//Configurer le moteur
void MoteurConf(void){
	//TIM2 CH2
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	// GPIOA PIN 1 : Mode Aternate Output Push-Pull
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
	MyTimer_Conf(TIM2,4784, 300);
	// TIM2 CH2 : PWM Output
	LL_TIM_OC_SetMode (TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
	TIM2->CCER |=0x1<<4;
	TIM2->CR1 |= TIM_CR1_CEN;
	// GPIOA PIN 2 : Mode Output Push-Pull
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
}


//Configurer la télécommande
void RxTelecommandeConf(void){
	// PB6 : TIM4 CH1
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_FLOATING);
	LL_TIM_IC_SetActiveInput( TIM4,LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	
	// Select the active input for TIMx_CCR1
	TIM4->CCMR1 |= 0x1;
  TIM4->CCMR1 &= ~(0x1<<1);
	//Select the active polarity for TI1FP1 
	TIM4->CCER &= ~(0x1 <<1);
	// Select the active input for TIMx_CCR2
	TIM4->CCMR1 |= 0x1<<9;
  TIM4->CCMR1 &= ~(0x1<<8);
	//Select the active polarity for TI1FP2 
	TIM4->CCER |= (0x1 <<5);
	// Select the valid trigger input
	TIM4->SMCR |= (0x1<<4) | (0x1<<6);
	TIM4->SMCR &= ~(0x1<<5);
	//Configure the slave mode controller in reset mode
	TIM4->SMCR |= (0x1<<2) ;
	TIM4->SMCR &= ~(0x3);
	//Enable the capture
	TIM4->CCER |= (0x11); 
	
	
	//PB7 : TIM4 CH2

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_FLOATING);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	LL_TIM_IC_SetActiveInput( TIM4,LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
	MyTimer_Conf(TIM4,21599,100);
	// Select the active input for TIMx_CCR1
	TIM4->CCMR1 |= 0x1;
  TIM4->CCMR1 &= ~(0x1<<1);
	//Select the active polarity for TI1FP1 
	TIM4->CCER &= ~(0x1 <<1);
	// Select the active input for TIMx_CCR2
	TIM4->CCMR1 |= 0x1<<9;
  TIM4->CCMR1 &= ~(0x1<<8);
	//Select the active polarity for TI1FP2 
	TIM4->CCER |= (0x1 <<5);
	// Select the valid trigger input
	TIM4->SMCR |= (0x1<<4) | (0x1<<6);
	TIM4->SMCR &= ~(0x1<<5);
	//Configure the slave mode controller in reset mode
	TIM4->SMCR |= (0x1<<2) ;
	TIM4->SMCR &= ~(0x3);
	//Enable the capture
	TIM4->CCER |= (0x11); 
	TIM4->CCER |=(0x1)<<4;
	TIM4->CR1 |= TIM_CR1_CEN;	
}

//Configurer le sens de rotation du plateau
void SensPlateau(int sens){
	if(sens){
		//Rotation contre le sens de l'horloge
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
	}else{
		//Rotation sens de l'horloge
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
	}
}

//Configurer la vitesse de rotation
void VitessePlateau(int pr){
	
	//Pas de rotation 	
	if(pr == 0){
		TIM2->CCR2 = 0;
	}
	
	//Rotation à 25% de la vitesse maximal
	else if(pr ==25){
		TIM2->CCR2 = TIM2->ARR /4;
	}
	
	//Rotation à 50% de la vitesse maximal
	else if(pr==50){
		TIM2->CCR2 = TIM2->ARR /2;		
	}
	
	//Rotation à 100% de la vitesse
	else if(pr==100){
		TIM2->CCR2 = TIM2->ARR;		
	} 
}

//Liaison entre la télecommande et le moteur
void Moteur_telecommande (void){

	float pourcentage ;
	float moy = (POURCENTAGE_MAX + POURCENTAGE_MIN)/2;
	
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
		
		 pourcentage = (((float)TIM4->CCR2) / ((float)TIM4-> CCR1))*100;
		
		if (pourcentage>POURCENTAGE_MIN && pourcentage < (POURCENTAGE_MIN + 1.0) ) {
			SensPlateau(0);
			VitessePlateau(100);
		}
		else if(pourcentage >(POURCENTAGE_MIN + 1.0) && pourcentage < (POURCENTAGE_MIN + 2.0)) {
			SensPlateau(0);
			VitessePlateau(50);
		} 
		else if(pourcentage > (POURCENTAGE_MIN + 2.0) && pourcentage < (moy - 0.3)) {
			SensPlateau(0);
			VitessePlateau(25);
		} 
		else if(pourcentage> (moy - 0.3) && pourcentage < (moy + 0.3)) {
			VitessePlateau(0);
		} 
		else if(pourcentage> (moy + 0.3) && pourcentage < (POURCENTAGE_MAX -2)) {
			SensPlateau(1);
			VitessePlateau(25);
		} 
		else if(pourcentage>(POURCENTAGE_MAX -2) && pourcentage < (POURCENTAGE_MAX -1) ) {
			SensPlateau(1);
			VitessePlateau(50);
		} 
		else if(pourcentage> (POURCENTAGE_MAX -1) && pourcentage < POURCENTAGE_MAX ) {
			SensPlateau(1);
			VitessePlateau(100);
		} 
			
}

// Récupérer de l'angle de la girouette
void InitAngleGirouette(void){
	while (((GPIOA->IDR)&(1<<5)) == (1<<5)){
	}		
	TIM3->CNT = 0;
}

int GetAngleGirouette(void){
	return (TIM3->CNT / 4);
}


// Configurer le pulse du servo
void SetPulseServo(void){
	int angle ;
	angle = GetAngleGirouette();
	if (angle > 180)
	{
		angle = 180-(angle-180);
	}
		
	
		if (angle < 45) {
			TIM1->CCR1 = ((float)(0.1 * (TIM1->ARR)));
		} else if (angle <60)  {
			TIM1->CCR1 = ((float)(0.09 * (TIM1->ARR)));
		} else if (angle < 90 ) {
			TIM1->CCR1 = ((float)(0.08 * (TIM1->ARR)));
		} else if (angle < 120) {
			TIM1->CCR1 = ((float)(0.07 * (TIM1->ARR)));
		} else if (angle <150) {
			TIM1->CCR1 = ((float)(0.06 * (TIM1->ARR)));
		} else 
			TIM1->CCR1 = ((float)(0.05 * (TIM1->ARR)));
		
}

//void AlimentationConfig(void){
//	//Configuration GPIO C
//	LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
//	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
//	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);
//	
//	//Configuration ADC
////	ADC_TypeDef ADC;
//	LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
////	LL_ADC_InitTypeDef ADC;
////	LL_ADC_StructInit();
//	LL_ADC_Enable(ADC1);
//	
//}

//Configurer l'accelero
void AcceleroConfig (void){
		//Configuration GPIO C
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

	//Configuration ADC
//	ADC_TypeDef ADC;
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
//	LL_ADC_InitTypeDef ADC;
//	LL_ADC_StructInit();
	//Diviser la fréqunece par 6( 36/6 <14)
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
	LL_ADC_Enable(ADC2);

	//Selection de la voie
	//ADC2->SQR1 &= ADC_SQR1_L;
	ADC2->SQR3 |=10 ;
	//calibration
//	ADC2->CR2 |= ADC_CR2_CAL;
//	while (ADC2->CR2 & ADC_CR2_CAL);
	
}


int Conversion(void){
	//Allumer ADC
	ADC2-> CR2 |= ADC_CR2_ADON; //déjà initialiser
	// Attendre la fin de la conversion
	while( !(ADC2 -> SR & ADC_SR_EOC));
	return ADC2 -> DR ;
}


