/*
 * os.c
 *
 *  Created on: 28 Feb 2022
 *      Author: adityasehgal
 */

#include "os.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// THREAD MANAGEMENT ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
thread_t thread_new(const char *name, thread_fn thread, void *arg,
		int stacksize, int prio) {

	const osThreadDef_t os_thread_def = { (char*) name, (os_pthread) thread,
			(osPriority) prio, 0, stacksize };

	return osThreadCreate(&os_thread_def, arg);

}

err_t thread_suspend(void) {
	return osThreadSuspend(osThreadGetId());
}

err_t thread_resume(void) {
	return osThreadResume(osThreadGetId());
}

err_t thread_delete(void) {
	return osThreadTerminate(osThreadGetId());
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// MUTEX MANAGEMENT ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
err_t mutex_new(mut_t *mut) {
	osMutexDef(MUTEX);
	*mut = osMutexCreate(osMutex(MUTEX));

	if (mut == NULL) {
		return osErrorNoMemory;
	}
	return osOK;
}

err_t mutex_lock(mut_t *mut) {
	return osMutexWait(*mut, osWaitForever);
}

err_t mutex_unlock(mut_t *mut) {
	return osMutexRelease(*mut);
}

err_t mutex_delete(mut_t *mut) {
	return osMutexDelete(*mut);
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// SEMAPHORE MANAGEMENT /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
err_t sem_new(sem_t *sem, uint8_t count) {
	osSemaphoreDef(SEM);
	*sem = osSemaphoreCreate(osSemaphore(SEM), 1);

	if (*sem == NULL) {
		return osErrorNoMemory;
	}

	if (!count) {
		osSemaphoreWait(*sem, 0);
	}

	return osOK;
}

uint32_t sem_wait(sem_t *sem, uint32_t timeout) {
	uint32_t start = osKernelSysTick();
	uint32_t end = 0;
	if (timeout && (timeout != osWaitForever)) {
		if (osSemaphoreWait(*sem, timeout) == osOK) {
			end = osKernelSysTick() - start;
		} else {
			return 0;
		}
	} else {
		// wait forever
		while (osSemaphoreWait(*sem, osWaitForever) != osOK)
			;
		end = osKernelSysTick() - start;
	}
	return end;
}

err_t sem_signal(sem_t *sem) {
	return osSemaphoreRelease(*sem);
}

err_t sem_delete(sem_t *sem) {
	return osSemaphoreDelete(*sem);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// SLEEP MANAGEMENT ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void sys_sleep(uint32_t time) {
	osDelay(time);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// MBOX MANAGEMENT ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
err_t mbox_new(mbox_t *mbox, uint32_t size) {
	osMessageQDef(QUEUE, size, void*);
	*mbox = osMessageCreate(osMessageQ(QUEUE), NULL);

	if (mbox == NULL) {
		return osErrorNoMemory;
	}

	return osOK;
}

err_t mbox_tryPost(mbox_t *mbox, void *msg) {
	return osMessagePut(*mbox, (uint32_t) msg, 0);
}

void mbox_post(mbox_t *mbox, void *msg) {
	while (osMessagePut(*mbox, (uint32_t) msg, osWaitForever) != osOK)
		;
}

err_t mbox_tryFetch(mbox_t *mbox, void *msg) {
	osEvent event = osMessageGet(*mbox, 0);

	if (event.status == osEventMessage) {
		msg = (void*) event.value.v;
		return osOK;
	}

	return osErrorValue;
}

uint32_t mbox_fetch(mbox_t *mbox, void *msg, uint32_t timeout) {
	uint32_t start = osKernelSysTick();
	uint32_t end = 0;
	osEvent event;
	if (timeout && (timeout != osWaitForever)) {
		event = osMessageGet(*mbox, timeout);
		if (event.status == osEventMessage) {
			msg = (void*) event.value.v;
			end = osKernelSysTick() - start;
		} else {
			return 0;
		}
	} else {
		event = osMessageGet(*mbox, osWaitForever);
		if (event.status == osEventMessage) {
			msg = (void*) event.value.v;
			end = osKernelSysTick() - start;
		} else {
			return 0;
		}
	}

	return end;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// MBOX MANAGEMENT ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
err_t timer_new(tim_t *timer, timer_fn callback, timer_type type, void *arg) {
	err_t err = osOK;
	timer->def.ptimer = (os_ptimer) callback;
	timer->id = xTimerCreate(								//
			(const char*) timer->name,						//
			1, 												//
			(type == osTimerPeriodic) ? pdTRUE : pdFALSE,	//
			(void*) arg,								//
			(TimerCallbackFunction_t) callback);

	if (timer->id == NULL) {
		err = osErrorNoMemory;
	}
	return err;
}

err_t timer_start(tim_t *timer, uint32_t time) {
	err_t err = osOK;
	err = osTimerStart(timer->id, time);
	return err;
}

err_t timer_stop(tim_t *timer) {
	err_t err = osOK;
	err = osTimerStop(timer->id);
	return err;
}

err_t timer_delete(tim_t *timer) {
	err_t err = osOK;
	err = osTimerDelete(timer->id);
	return err;
}
