/*
 * vrs_cv5.h
 *
 *  Created on: 18. 10. 2016
 *      Author: Georgio
 */

#ifndef VRS_CV5_H_
#define VRS_CV5_H_

void gpio_init();
void adc_init();
void usart_init();
void Posielanie(char *hodnota);
void delay(uint32_t time);

#endif /* VRS_CV5_H_ */
