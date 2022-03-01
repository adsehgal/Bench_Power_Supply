/*
 * os.h
 *
 *  Created on: 28 Feb 2022
 *      Author: adityasehgal
 *
 *  This file is an abstraction layer to CMSIS OS
 */

#ifndef OS_H_
#define OS_H_

#include "cmsis_os.h"

typedef void (*thread_fn)(void *arg);
typedef void (*timer_fn)(void *argument);

#define MBOX_NULL (osMessageQId)0
#define SEMAPHORE_NULL  (osSemaphoreId)0

typedef osThreadId thread_t;
typedef osSemaphoreId sem_t;
typedef osSemaphoreId mut_t;
typedef osMessageQId mbox_t;

typedef osStatus err_t;
typedef os_timer_type timer_type;

#define __OS_TIMER_NAME_LEN 15
typedef struct sys_timer_s {
	osTimerId id;
	osTimerDef_t def;
	char name[__OS_TIMER_NAME_LEN ];
} tim_t;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// THREAD MANAGEMENT ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/**
 * @fn thread_t thread_new(const char*, thread_fn, void*, int, int)
 * @brief			Create and start a new thread with the defined params
 * @param name		The name of the thread
 * @param thread	Function pointer to the actual thread
 * @param arg		Argument to be passed to the thread during creation
 * @param stacksize	Allocated stack size (bytes) for the thread
 * @param prio		The priority at which the thread should run
 * @return			CMSISOS/Freertos thread handle or NULL on failure
 */
thread_t thread_new(const char *name, thread_fn thread, void *arg,
		int stacksize, int prio);

/**
 * @fn err thread_suspend(void)
 * @brief	Suspend the thread this function is called from
 * @return	Positive error code upon failure or 0 on success
 */
err_t thread_suspend(void);

/**
 * @fn void thread_resume(void)
 * @brief	Resume the thread this function is called from
 * @return	Positive error code upon failure or 0 on success
 */
err_t thread_resume(void);

/**
 * @fn void thread_delete(void)
 * @brief	Delete and free up the resources of the thread this function is called from
 * @return	Positive error code upon failure or 0 on success
 */
err_t thread_delete(void);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// MUTEX MANAGEMENT ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/**
 * @fn err_t mutex_new(mut_t*)
 * @brief	Create a new mutex
 * @param mut	Pointer to the mutex to create
 * @return		Positive error code upon failure or 0 on success
 */
err_t mutex_new(mut_t *mut);

/**
 * @fn err_t mutex_lock(mut_t*)
 * @brief		Block the thread indefinitely until the mutex can be grabbed
 * @param mut	Pointer to the mutex being referred to
 * @return		Positive error code upon failure or 0 on success
 */
err_t mutex_lock(mut_t *mut);

/**
 * @fn err_t mutex_unlock(mut_t*)
 * @brief 		Release the previously locked mutex, effectively unblocking the thread
 * @param mut	Pointer to the mutex being referred to
 * @return		Positive error code upon failure or 0 on success
 */
err_t mutex_unlock(mut_t *mut);

/**
 * @fn err_t mutex_delete(mut_t*)
 * @brief		Delete and free up resource of a mutex
 * @param mut	Poiter to the mutex being referred to
 * @return		Positive error code upon failure or 0 on success
 */
err_t mutex_delete(mut_t *mut);

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// SEMAPHORE MANAGEMENT /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/**
 * @fn err_t sem_new(sem_t*, uint8_t)
 * @brief 		Create a new semaphore with a given initial count
 * @param sem	Pointer to the semaphore to create
 * @param count	Initial count of the semaphore
 * @return		Positive error code upon failure or 0 on success
 */
err_t sem_new(sem_t *sem, uint8_t count);

/**
 * @fn uint32_t sem_wait(sem_t*, uint32_t)
 * @brief 			Wait for a given time for a semaphore to signal or timeout
 * @param sem		Point to the semaphore to be waited on
 * @param timeout	Timeout in ms to be waited for before timeout
 * @return			0 on timeout or time waited otherwise
 */
uint32_t sem_wait(sem_t *sem, uint32_t timeout);

/**
 * @fn err_t sem_signal(sem_t*)
 * @brief 		Signal the semaphore in order to unblock the thread
 * @param sem	Pointer to the semaphore to be signaled
 * @return		Positive error code upon failure or 0 on success
 */
err_t sem_signal(sem_t *sem);

/**
 * @fn err_t sem_delete(sem_t*)
 * @brief 		Delete and free up the resources for a semaphore
 * @param sem	Pointer to the semaphore being deleted
 * @return		Positive error code upon failure or 0 on success
 */
err_t sem_delete(sem_t *sem);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// SLEEP MANAGEMENT ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/**
 * @fn void sys_sleep(uint32_t)
 * @brief 		Sleep the thread for a given amount of time
 * @param time	Time in ms for the thread to sleep/delay
 */
void sys_sleep(uint32_t time);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// MBOX MANAGEMENT ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/**
 * @fn err_t mbox_new(mbox_t*, uint32_t)
 * @brief		Create a new message box of a specified size
 * @param mbox	Pointer to the message box being created
 * @param size	Size in bytes of the message box being created
 * @return		Positive error code upon failure or 0 on success
 */
err_t mbox_new(mbox_t *mbox, uint32_t size);

/**
 * @fn err_t mbox_tryPost(mbox_t*, void*)
 * @brief		Try to post a message or return failure
 * @param mbox	Pointer to message box being posted to
 * @param msg	Pointer to the message being posted
 * @return		Positive error code upon failure or 0 on success
 */
err_t mbox_tryPost(mbox_t *mbox, void *msg);

/**
 * @fn void mbox_post(mbox_t*, void*)
 * @brief 		Wait forever until a message can be posted and post it
 * @param mbox	Pointer to the message box being posted to
 * @param msg	Pointer to the message being posted
 */
void mbox_post(mbox_t *mbox, void *msg);

/**
 * @fn err_t mbox_tryFetch(mbox_t*, void*)
 * @brief 		Try to fetch a new message or return failure
 * @param mbox	Pointer to the message box being fetched from
 * @param msg	Pointer to the receiving message
 * @return		Positive error code upon failure or 0 on success
 */
err_t mbox_tryFetch(mbox_t *mbox, void *msg);

/**
 * @fn uint32_t mbox_fetch(mbox_t*, void*, uint32_t)
 * @brief			Wait until timeout to receiver a message
 * @param mbox		Pointer to the message box being fetched from
 * @param msg		Pointer to the receiving message
 * @param timeout	Timeout in ms to be waited for to fetch a message
 * @return			0 on timeout or time waited otherwise
 */
uint32_t mbox_fetch(mbox_t *mbox, void *msg, uint32_t timeout);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// MBOX MANAGEMENT ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/**
 * @fn err_t timer_new(timer_t*, timer_fn, timer_type, void*)
 * @brief 			Create a new OS timer
 * @param timer		Handle to the timer being created
 * @param callback	Callback function upon capture complete of timer
 * @param type		osTimerOnce or osTimerPeriodic
 * @param arg		Argument to pass to the timer callback
 * @return			Positive error code upon failure or 0 on success
 */
err_t timer_new(tim_t *timer, timer_fn callback, timer_type type, void *arg);

/**
 * @fn err_t timer_start(timer_t*, uint32_t)
 * @brief 		Start a timer
 * @param timer	Handle to timer being started
 * @param time	Time in ms before the timer expires
 * @return		Positive error code upon failure or 0 on success
 */
err_t timer_start(tim_t *timer, uint32_t time);

/**
 * @fn err_t timer_stop(timer_t*)
 * @brief 		Stop a timer
 * @param timer	Handle to timer being stopped
 * @return		Positive error code upon failure or 0 on success
 */
err_t timer_stop(tim_t *timer);

/**
 * @fn err_t timer_delete(timer_t*)
 * @brief 		Delete a timer
 * @param timer	Handle to timer being deleted
 * @return		Positive error code upon failure or 0 on success
 */
err_t timer_delete(tim_t *timer);
#endif /* OS_H_ */
