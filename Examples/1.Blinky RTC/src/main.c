/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/
#include <stm32f103xb.h>
void wait_ms(unsigned int time);
void wait_s(unsigned int time);

int main(void){
	//--Inicialización interface debug--------------------------------------------------
	RCC->APB2ENR |= (0b1<<0); 			// AFIOEN = 1, activar reloj de modulo AFIO
	AFIO->MAPR |= (0b010<<24); 			// SWJ_CFG = 010 JTAG disabled y SW-DP enable
	// Inicializacion de reloj

	RCC->APB1ENR	|= (0b1<<28);		// PWREN=1, Habilitar el reloj del modulo PWR
	RCC->APB1ENR	|= (0b1<<27);		// BKPEN=1, Habilitar el reloj de los registros BKP
	PWR->CR			|= (0b1<<8 );		// DBP Deshabilitar proteccion de escritura de registros de Backup
	RCC->BDCR		|= (0b1<<15);		// RTCEN=1  Habilitar reloj RTC
	RCC->BDCR		|= (0b1<<0 );		// LSEON=1 Habilitar reloj LSE de 32.768 Khz
	while((RCC->BDCR & 0b10)!=1);		// LSERDY Verificar que el oscilador esterno esta en modo estable
	//RCC->CSR		|= (0b1<<0 );		// LSION, Encender reloj LS interno
	//while((RCC->CSR & 0b1)!=1);		// LSIRDY Verificar que el oscilador interno esta en modo estable
	RCC->BDCR		|= (0b01<<8);		// RTCSEL Seleccionar LSE como reloj de RTC
	
	// Inicializacion del RTC
	while((((RTC->CRL)>>5) & 1)!=1);	// Esperar a que RTOFF sea 1
	RTC->CRL 		|= (1<<4);			// CNF=1 Habilitar configuracion del RTC
	RTC->PRLH		 = 0x0000;
	RTC->PRLL        = 0x8000;          // PRL = 32, frecuencia del contador, 1.007 ms
	//RTC->CNTH		 = 0;				// CNTH=0, Reiniciar contador
	//RTC->CNTL		 = 0;				// CNTL=0, Reiniciar contador
	RTC->CRL        ^= (1<<4);			// CNF=0 Deshabilitar configuracion del RTC
	while((((RTC->CRL)>>5) & 1)!=1);	// Esperar a que RTOFF sea 1
	// Incializacion de Puerto C-Pin13
	RCC->APB2ENR    |= 0x10u;
	GPIOC->CRH  	 = 0x00000000;
	GPIOC->CRH 		|= (0b00 <<22);		// CNF Modo General Purpuse push-pull PC13
	GPIOC->CRH 		|= (0b10 <<20);		// MODE Modo de salida de baja velocidad PC13
	GPIOC->ODR 		|= (0x01u<<13);
	while(1){
		GPIOC->ODR 	^= (0x1<<13);		// Toggle
		wait_s(1);

	}
}

void wait_ms(unsigned int time){
	while((((RTC->CRL)>>5) & 1)!=1);	// Esperar a que RTOFF sea 1
	RTC->CRL 		|= (1<<4);			// CNF=1 Habilitar configuracion del RTC
	RTC->CNTH		 = 0;				// CNTH=0, Reiniciar contador
	RTC->CNTL		 = 0;				// CNTL=0, Reiniciar contador
	RTC->CRL        ^= (1<<4);			// CNF=0 Deshabilitar configuracion del RTC
	while((((RTC->CRL)>>5) & 1)!=1);	// Esperar a que RTOFF sea 1
	unsigned int count=0;
	while(count<time){
		count=(RTC->CNTH <<16) + (RTC->CNTL); // Leer el contador de RTC
	}
	count=0;
}


void wait_s(unsigned int time){
	while((((RTC->CRL)>>5) & 1)!=1);	// Esperar a que RTOFF sea 1
	RTC->CRL 		|= (1<<4);			// CNF=1 Habilitar configuracion del RTC
	RTC->CNTH		 = 0;				// CNTH=0, Reiniciar contador
	RTC->CNTL		 = 0;				// CNTL=0, Reiniciar contador
	RTC->CRL        ^= (1<<4);			// CNF=0 Deshabilitar configuracion del RTC
	while((((RTC->CRL)>>5) & 1)!=1);	// Esperar a que RTOFF sea 1
	unsigned int count=0;
	while(count<time){
		count=(RTC->CNTH <<16) + (RTC->CNTL); // Leer el contador de RTC
	}
	count=0;
}