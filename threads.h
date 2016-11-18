// ----------------------------------------------------------------------------
// Copyright 2006-2007, Martin D. Flynn
// All rights reserved
// ----------------------------------------------------------------------------
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ----------------------------------------------------------------------------

#ifndef _THREADS_H
#define _THREADS_H
#ifdef __cplusplus
extern "C" {
#endif

#if defined(TARGET_WINCE)
#include <windows.h>
#else
#include <pthread.h>
#endif

#include "stdtypes.h"

// ----------------------------------------------------------------------------
/* maximum expected number of threads */
#define MAX_THREADS         12

// ----------------------------------------------------------------------------
// Thread structure definitions

// ------------------------------------
/* thread structure */
typedef struct 
{
    UInt32                  didInit;
#if defined(TARGET_WINCE)
    HANDLE                  thread;
    DWORD                   id;
#else
    pthread_t               thread;
#endif
    char                    name[20];
} threadThread_t;

// ------------------------------------
/* mutex structure */
typedef struct 
{
    UInt32                  didInit;
#if defined(TARGET_WINCE)
    CRITICAL_SECTION        mutex;
#else
    pthread_mutex_t         mutex;
#endif
    UInt16                  syncCount;  // used by 'threadSync'
} threadMutex_t;

// ------------------------------------
/* condition structure */
typedef struct 
{
    UInt32                  didInit;
#if defined(TARGET_WINCE)
    HANDLE                  semaphore;
    CRITICAL_SECTION        numWaitersLock; 
    int                     numWaiters;     // number of waiting threads
#else
    pthread_cond_t          semaphore;
#endif
} threadCond_t;

// ----------------------------------------------------------------------------

#define THREAD_SRC                  __FILE__,__LINE__
//#define THREAD_SRC                "",__LINE__

#define MUTEX_LOCK(M)               threadMutexLock(THREAD_SRC,(M));
#define MUTEX_UNLOCK(M)             threadMutexUnlock(THREAD_SRC,(M));
//#define EXAMPLE_LOCK              MUTEX_LOCK(&exampleMutex)
//#define EXAMPLE_UNLOCK            MUTEX_UNLOCK(&exampleMutex)
// EXAMPLE_LOCK {
//    <perform critical code here>
// } EXAMPLE_UNLOCK

#define MUTEX_SYNCHRONIZED(M)       for(threadSync((M),0);threadSync((M),1);)
// [Note: this method not yet fully tested]
//#define EXAMPLE_SYNCHRONIZED  MUTEX_SYNCHRONIZED(&exampleMutex)
// EXAMPLE_SYNCHRONIZE {
//    <perform critical code here>
// }

#define CONDITION_WAIT(C,M)         threadConditionWait((C),(M));
#define CONDITION_TIMED_WAIT(C,M,T) threadConditionTimedWait((C),(M),(T));
#define CONDITION_NOTIFY(C)         threadConditionNotify(C);

// ----------------------------------------------------------------------------
// global definitions 
#define SIG_TIMEOUT		SIGRTMIN 
#define TIMER_PROTOCOL_1	101		//protocol timer 1
#define TIMER_PROTOCOL_2	106		//protocol timer 2
#define TIMER_GPS_1		102		//gps timer 1
#define TIMER_UPDATE		103		//update timer
#define TIMER_RFID_1		104		//rfid timer 1
#define TIMER_NETWORK_MANAGER	105		//network manager timer
#define TIMER_POWER_SAVING	107		//power-saving timer 
// ----------------------------------------------------------------------------

/* get thread count */
int threadGetCount();

// ----------------------------------------------------------------------------

/* create thread */
int threadCreate(threadThread_t *thread, void *(*runnable)(void*), void *arg, const char *name);

/* exit thread */
void threadExit();

/* add function to thread-stop list */
utBool threadAddThreadStopFtn(void (*ftn)(void*), void *arg);

/* invoke all thread-stop functions */
void threadStopThreads(void);

// ----------------------------------------------------------------------------

/* mutex support */
int threadMutexInit(threadMutex_t *mutex);
void threadMutexFree(threadMutex_t *mutex);
int threadMutexLock(const char *fn, int line, threadMutex_t *mutex);
int threadMutexUnlock(const char *fn, int line, threadMutex_t *mutex);
utBool threadSync(const char *fn, int line, threadMutex_t *mutex, int mode);

// ----------------------------------------------------------------------------

/* condition support */
int threadConditionInit(threadCond_t *cond);
void threadConditionFree(threadCond_t *cond);
int threadConditionWait(threadCond_t *cond, threadMutex_t *mutex);
int threadConditionTimedWait(threadCond_t *cv, threadMutex_t *mutex, struct timespec *tm);
int threadConditionNotify(threadCond_t *cond);

// ----------------------------------------------------------------------------

/* sleep specified number of milliseconds */
void threadSleepMS(UInt32 msec);

// ----------------------------------------------------------------------------

/* initialize vars */
void threadInitialize();

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif // _THREADS_H
