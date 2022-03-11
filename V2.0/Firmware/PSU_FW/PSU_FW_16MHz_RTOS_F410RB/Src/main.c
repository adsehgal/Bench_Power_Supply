#include <stdio.h>

#include "os.h"
#include "config.h"

#include "uart.h"
#include "analog.h"
#include "led.h"
#include "pwr.h"

tim_t timer = { .name = "TIMER" };
sem_t sem;

void callback(void *arg) {
	sem_signal(&sem);
}

void task(void *arg) {

	sem_new(&sem, 0);

	timer_new(&timer, callback, osTimerPeriodic, NULL);
	timer_start(&timer, 500);

	led_off(LED_VI);
	led_on(LED_OE);
	led_off(LED_CC);
	for (;;) {
		sem_wait(&sem, osWaitForever);
		led_toggle(LED_VI);
		led_toggle(LED_OE);
		led_toggle(LED_CC);
		for (pwrIdx_t i = 0; i < ANA_COUNT; i++) {
			printf("%s %d %s\n", pwr_getName(i), pwr_getVal(i),
					pwr_getUnits(i));
		}
		printf("\n");
	}
}

int main(void) {

	config_sysInit();

	uart_init();
	led_init();
	analog_init();
	pwr_init();

	printf("Initialization complete\n  Built on %s at %s\n\n",
	__DATE__, __TIME__);

	thread_new("TASK", task, NULL, 256, 0);

	osKernelStart();

	while (1) {
		// Should never get here!
		// osThreadSuspendAll();
	}

}

