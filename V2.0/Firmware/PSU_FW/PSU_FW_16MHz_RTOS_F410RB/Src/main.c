#include <stdio.h>

#include "os.h"
#include "config.h"

#include "uart.h"
#include "led.h"

tim_t timer = { .name = "TIMER" };
sem_t sem;

void callback(void *arg) {
	sem_signal(&sem);
}

void task(void *arg) {

	sem_new(&sem, 0);

	timer_new(&timer, callback, osTimerPeriodic, NULL);
	timer_start(&timer, 100);

	led_off(LED_VI);
	led_on(LED_OE);
	led_off(LED_CC);
	for (;;) {
		sem_wait(&sem, osWaitForever);
		led_toggle(LED_VI);
		led_toggle(LED_OE);
		led_toggle(LED_CC);
	}
}

int main(void) {

	config_sysInit();

	uart_init();
	led_init();

	printf("Initialization complete\n  Built on %s at %s\n\n",
	__DATE__, __TIME__);

	thread_new("TASK", task, NULL, 256, 0);

	osKernelStart();

	while (1) {
		// Should never get here!
		// osThreadSuspendAll();
	}

}

