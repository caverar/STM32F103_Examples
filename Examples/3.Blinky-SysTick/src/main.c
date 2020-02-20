#include <stm32f103xb.h>

void wait_ms(unsigned int time);

int main(void){

	//--Inicialización interface debug---------------------------------------------------------------------

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 					// AFIOEN = 1, activar reloj de modulo AFIO
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;						// SWJ_CFG = 010 JTAG disabled y SW-DP enable

	//--Inicializacion de reloj externo a 72 Mhz-----------------------------------------------------------

	FLASH->ACR |=FLASH_ACR_LATENCY_1;						// Ajustar Latencia de memoria Flash en 2 (Debido al ajuste de alta frecuencia)
	RCC->CR |= RCC_CR_HSEON; 								// HSE0N = 1, Habilitar reloj externo
    while(!(RCC->CR & RCC_CR_HSERDY));   					// HSERDY = 1 ?, Esperar a que el reloj externo se estabilice
    RCC->CFGR |= RCC_CFGR_PLLSRC; 							// PLLSRC=1, Seleccionar reloj externo como fuente de reloj de PLL
    RCC->CFGR |= RCC_CFGR_PLLMULL9;							// PLLMUL=7, Seleccionar 9 como factor PLL
    RCC->CR	  |= RCC_CR_PLLON;								// PLLON=1 Encender PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));						// PLLRDY=1 ? Esperar a que se ajuste el PLL
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 						// PPRE1=0b100, Ajustar el prescaler de APB1, para obtener 36Mhz
    RCC->CFGR |= RCC_CFGR_SW_1;    							// SW=2, PLL seleccionado como reloj del sistema
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); 				// SWS=2? , Esperar a que el reloj del sistema se configure

    //--Incializacion de Puerto C-Pin13---------------------------------------------------------------------------

    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;						// Habilitar reloj de Puerto C
    GPIOC->CRH |= GPIO_CRH_CNF13_0;							// CNF13=1,  Modo General Purpuse push-pull PC13
    GPIOC->CRH |= GPIO_CRH_MODE13_1;						// MODE13=2, Modo de salida de baja velocidad PC13
    GPIOC->ODR |= GPIO_ODR_ODR13;							// ODR13=1, Estado inicial del led= apagado

    //--Inicializacion de SysTick---------------------------------------------------------------------------------

    //SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;			// CLKSOURCE=1, Seleccionar prescaler de 1 para el reloj del sistema (72mhz)

    //----------------------------------------------------------------------------------------------------------
    while(1){
    	GPIOC->ODR ^= GPIO_ODR_ODR13;						// Toggle PC13
    	wait_ms(9000);										// espera de 1000ms
   	}
}
void wait_ms(unsigned int time){
	SysTick->LOAD =(time*9000)-1; 							// Load=Time*8-1, ajustar valor del contador
	SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;				// Enable=1, Habilitar contador
	while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));	// COUNTFLAG=1?,Preguntar si ya se alcanzo 0
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;				// Enable=0, Deshabilitar el contador

}
