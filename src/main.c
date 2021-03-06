#include <stddef.h>
#include "stm32l1xx.h"
#include "vrs_cv5.h"
#include <stdio.h>


uint16_t value;
uint16_t value2;
uint8_t zmena = 1;

int main(void)
{

	gpio_init();
	adc_init();
	usart_init();

	char hodnota[10];

  while (1)
  {
	  if(zmena == 0)
	  {

		  sprintf(hodnota, "%d mV", value2);
	  }
	  else
	  {
		  sprintf(hodnota, "%d", value);
	  }
	  Posielanie(hodnota);
	  delay(400000);
  }
  return 0;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}
