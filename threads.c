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
// Description:
//  Thread interface
// ---
// Change History:
//  2007/01/28  Martin D. Flynn
//     -Initial release
//     -WindowsCE port
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

#if defined(TARGET_WINCE)
#  include <aygshell.h>
#  include <kfuncs.h>
#endif

#include "log.h"

#include "stdtypes.h"
#include "utctools.h"
#include "strtools.h"
#include "threads.h"

// ----------------------------------------------------------------------------

// It doesn't matter what the value of this magic number is as long as it is 'unlikely' 
// that it will appear in random, unitilized memory.  The purpose of this 'magic number'
// is to attempt to detect an uninitialized thread/condition/mutex when it is being used.
// Possible 'magic' words: BEEF, DEAD, CAFE, BABE, C001, D00D, C0DE, FEED, FACE, BADC0FEE
#define MAGIC_INIT_VALUE    ((UInt32)0xDEADCAFE)

// ----------------------------------------------------------------------------

static int                  threadCreateCount = 0;
static utBool               _threadDidInit = utFalse;

// ----------------------------------------------------------------------------

typedef struct
{
    void (*ftn)(void*);
    void *arg;
} StopThreadFtn_t;
static StopThreadFtn_t stopThreadFtn[MAX_THREADS];
static int stopThreadFtnCount = 0;

/* add function to call to indicate to a given thread that it should terminate */
utBool threadAddThreadStopFtn(void (*ftn)(void*), void *arg)
{
    if (ftn) {
        int i;
        for (i = 0; i < stopThreadFtnCount; i++) {
            if ((stopThreadFtn[i].ftn == ftn) && (stopThreadFtn[i].arg == arg)) {
                return utTrue; // already present
            }
        }
        if (stopThreadFtnCount < MAX_THREADS) {
            stopThreadFtn[stopThreadFtnCount].ftn = ftn;
            stopThreadFtn[stopThreadFtnCount].arg = arg;
            stopThreadFtnCount++;
            return utTrue; // newly added
        }
    }
    return utFalse;
}

/* call the stop thread function in all threads */
void threadStopThreads(void)
{
    int i;
    for (i = 0; i < stopThreadFtnCount; i++) {
        if (stopThreadFtn[i].ftn) {
            (*(stopThreadFtn[i].ftn))(stopThreadFtn[i].arg);
        }
    }
    // TODO: wait unit all threads acknowledge that they have stopped
/*    threadSleepMS(20000L); // wait until threads stop */
}

// ----------------------------------------------------------------------------

/* return number of created thread */
int threadGetCount()
{
    return threadCreateCount;
}

// ----------------------------------------------------------------------------

/* create thread */
int threadCreate(threadThread_t *thread, void *(*runnable)(void*), void *arg, const char *name)
{
    if (!_threadDidInit) {
        logWARNING(LOGSRC,"Attempting to create a thread before threads have been initialized!!!");
        threadInitialize();
    }
    if (thread) {
        int rtn = -1;
        memset(thread, 0, sizeof(threadThread_t));
        thread->didInit = MAGIC_INIT_VALUE;
        strCopy(thread->name, sizeof(thread->name), name, -1);
#if defined(TARGET_WINCE)
        thread->thread = CreateThread(NULL,0,(unsigned long(*)(void*))runnable,arg,0,&(thread->id));
        if (thread->thread) {
            rtn = 0; // success
        }
#else
        int createErr = pthread_create(&thread->thread, NULL, runnable, arg);
        if (createErr == 0) {
            rtn = 0; // success
        }
#endif
        return rtn;
    } else {
        logCRITICAL(LOGSRC,"Null thread buffer specified");
        return -1;
    }
}

/* exit current thread */
void threadExit()
{
#if defined(TARGET_WINCE)
    ExitThread(0);
#else
    pthread_exit((void*)0);
#endif
}

// ----------------------------------------------------------------------------

/* initialize mutex */
int threadMutexInit(threadMutex_t *mutex)
{
    if (mutex) {
        memset(mutex, 0, sizeof(mutex));
        mutex->didInit = MAGIC_INIT_VALUE;
#if defined(TARGET_WINCE)
        InitializeCriticalSection(&(mutex->mutex));
#else
        pthread_mutex_init(&(mutex->mutex), (pthread_mutexattr_t*)0);
#endif
        return 0;
    } else {
        logCRITICAL(LOGSRC,"Null mutex buffer specified");
        return -1;
    }
}

/* free mutex */
void threadMutexFree(threadMutex_t *mutex)
{
    if (mutex) {
        if (mutex->didInit == MAGIC_INIT_VALUE) {
#if defined(TARGET_WINCE)
            DeleteCriticalSection(&(mutex->mutex));
#else
            pthread_mutex_destroy(&(mutex->mutex));
#endif
        }
        memset(mutex, 0, sizeof(threadMutex_t));
    }
}

/* lock mutex */
int threadMutexLock(const char *fn, int line, threadMutex_t *mutex)
{
/*    if (mutex && (mutex->didInit == MAGIC_INIT_VALUE)) {
#if defined(TARGET_WINCE)
        EnterCriticalSection(&(mutex->mutex));
#else
        pthread_mutex_lock(&(mutex->mutex));
#endif
        mutex->syncCount = 0;
        return 0;
    } else {
        logCRITICAL(LOGSRC,"Unitialized mutex! [%s:%d]", logSrcFile(fn), line);
        return -1;
    }
*/
	if (mutex->didInit == MAGIC_INIT_VALUE) {
		pthread_mutex_lock(&(mutex->mutex));
		mutex->syncCount = 0;
	}
	return 0;
}

/* unlock mutex */
int threadMutexUnlock(const char *fn, int line, threadMutex_t *mutex)
{
    // Note: no checking is made to insure that this function is only called
    // by the thread that maintains the lock.
/*    if (mutex && (mutex->didInit == MAGIC_INIT_VALUE)) {
#if defined(TARGET_WINCE)
        LeaveCriticalSection(&(mutex->mutex));
#else
        pthread_mutex_unlock(&(mutex->mutex));
#endif
        return 0;
    } else {
        logCRITICAL(LOGSRC,"Unitialized mutex! [%s:%d]", logSrcFile(fn), line);
        return -1;
    }
*/
	if (mutex->didInit == MAGIC_INIT_VALUE)
        pthread_mutex_unlock(&(mutex->mutex));
	return 0;
}

/* mutex synchronize */
utBool threadSync(const char *fn, int line, threadMutex_t *mutex, int mode)
{
    if (mutex && (mutex->didInit == MAGIC_INIT_VALUE)) {
        if (mode == 0) {
            // initial lock
            threadMutexLock(fn,line,mutex);
            return utTrue;
        } else
        if (++(mutex->syncCount) <= 1) {
            // first pass
            return utTrue;
        } else {
            // second/last pass
            threadMutexUnlock(fn,line,mutex);
            return utFalse;
        }
    } else {
        logCRITICAL(LOGSRC,"Null or unitialized mutex buffer specified");
        return utFalse;
    }
}

// ----------------------------------------------------------------------------

/* initialize conditions */
int threadConditionInit(threadCond_t *cond)
{
    if (cond) {
    /*    memset(cond, 0, sizeof(threadCond_t)); */
        cond->didInit = MAGIC_INIT_VALUE;
#if defined(TARGET_WINCE)
        cond->numWaiters = 0;
        InitializeCriticalSection(&(cond->numWaitersLock));
        cond->semaphore = (void*)CreateSemaphore(NULL, 0, 0x7FFFFFFF, NULL);
#else
        pthread_cond_init(&(cond->semaphore), (pthread_condattr_t*)0);
#endif
        return 0;
    } else {
        logCRITICAL(LOGSRC,"Null condition buffer specified");
        return -1;
    }
}

/* free condition */
void threadConditionFree(threadCond_t *cond)
{
    if (cond) {
        if (cond->didInit == MAGIC_INIT_VALUE) {
#if defined(TARGET_WINCE)
            DeleteCriticalSection(&(cond->numWaitersLock));
            CloseHandle(&(cond->semaphore));
#else
            pthread_cond_destroy(&(cond->semaphore));
#endif
        }
        memset(cond, 0, sizeof(threadCond_t));
    }
}

/* wait for condition */
int threadConditionWait(threadCond_t *cond, threadMutex_t *mutex)
{
    return threadConditionTimedWait(cond, mutex, (struct timespec*)0);
}

/* wait for condition with timeout */
int threadConditionTimedWait(threadCond_t *cond, threadMutex_t *mutex, struct timespec *tm)
{
	int err = 0;
	if (cond->didInit == MAGIC_INIT_VALUE && mutex->didInit == MAGIC_INIT_VALUE) {
#if defined(TARGET_WINCE)
        /* check timeout millis */
        // 1 second == 1000 milliseconds == 1000,000 microseconds = 1000,000,000 nanoseconds
        Int32 deltaMS = INFINITE;
        if (tm) {
            struct timeval n;
            utcGetTimestamp(&n);
            deltaMS = ((tm->tv_sec - n.tv_sec) * 1000L) + ((tm->tv_nsec/1000L - n.tv_usec) / 1000L);
            if (deltaMS <= 0L) {
                return 1; // already timed-out
            }
        }
        /* increment waiter */
        EnterCriticalSection(&(cond->numWaitersLock));
        cond->numWaiters++;
        LeaveCriticalSection(&(cond->numWaitersLock));
        /* release lock on mutex */
        LeaveCriticalSection(&(mutex->mutex));
        /* wait for notification */
        DWORD err = WaitForSingleObject(cond->semaphore, (DWORD)deltaMS); // WAIT_TIMEOUT
        /* decrement waiter */
        EnterCriticalSection(&(cond->numWaitersLock));
        cond->numWaiters--;
        LeaveCriticalSection(&(cond->numWaitersLock));
        /* re-aquire lock on mutex */
        EnterCriticalSection(&(mutex->mutex));
        return (err == WAIT_OBJECT_0)? 0 : 1;
	}
	return -1;
#else
		if (tm) {
			err = pthread_cond_timedwait(&(cond->semaphore), &(mutex->mutex), tm);
		} else {
			err = pthread_cond_wait(&(cond->semaphore), &(mutex->mutex));
		}
	}
	return err;
#endif
}

/* notify condition */
int threadConditionNotify(threadCond_t *cond)
{
	if (cond->didInit == MAGIC_INIT_VALUE) {
#if defined(TARGET_WINCE)
		/* check waiters */
		EnterCriticalSection(&(cond->numWaitersLock));
		int haveWaiters = (cond->numWaiters > 0);
		LeaveCriticalSection(&(cond->numWaitersLock));
		/* release Semaphore */
		if (haveWaiters) {
			ReleaseSemaphore(cond->semaphore, 1, 0);
		}
#else
		pthread_cond_signal(&(cond->semaphore));
#endif
		return 0;
	} else {
		return -1;
	}
}

// ----------------------------------------------------------------------------

/* sleep for the specified amount of milliseconds */
void threadSleepMS(UInt32 msec)
{
    // 1 second == 1000 milliseconds == 1000,000 microseconds = 1000,000,000 nanoseconds
    //logDEBUG(LOGSRC,"Sleeping %lu ms ...", msec);
#if defined(TARGET_WINCE)
    Sleep((DWORD)msec);
#else
    struct timeval delay;
    delay.tv_sec  = msec / 1000L;               // usec / 1000000L;
    delay.tv_usec = (msec % 1000L) * 1000L;     // usec % 1000000L;
    select(0, 0, 0, 0, &delay);
#endif
}

// ----------------------------------------------------------------------------

/* initialize */
// Note: It may not be necessary to call this function directly
void threadInitialize()
{

    /* already initialized? */
    if (_threadDidInit) {
        return;
    }
    _threadDidInit = utTrue;

    /* clear structures */
    memset(stopThreadFtn, 0, sizeof(stopThreadFtn));

}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// Example use code

// static threadThread_t       exampleThread;
// static threadMutex_t        exampleMutex;
// static threadCond_t         exampleCond;
// #define EXAMPLE_LOCK        MUTEX_LOCK(&exampleMutex);
// #define EXAMPLE_UNLOCK      MUTEX_UNLOCK(&exampleMutex);
// #define EXAMPLE_WAIT        CONDITION_WAIT(&exampleCond, &exampleMutex);
// #define EXAMPLE_NOTIFY      CONDITION_NOTIFY(&exampleCond);
//
// static void _exampleThreadRunnable(void *arg)
// {
//     while (1) {
//         // ...
//         EXAMPLE_LOCK {
//             while (<SomeConditionNotMet>) {
//                 EXAMPLE_WAIT // wait for someone else to call 'EXAMPLE_NOTIFY'
//                 // 'EXAMPLE_NOTIFY' must be called within another 'EXAMPLE_LOCK' block
//             }
//             // perform critical section code here
//         } EXAMPLE_UNLOCK
//         // ...
//     }
//     threadExit();
// }
//
// utBool exampleStartThread()
// {
//     // create mutex's
//     threadMutexInit(&exampleMutex);
//     threadConditionInit(&exampleCond);
//     // start thread
//     exampleRunThread = utTrue; // must set BEFORE we start the thread!
//     if (threadCreate(&exampleThread,&_exampleThreadRunnable,0,"Example") == 0) {
//         // thread started
//     } else {
//         // Unable to start thread
//         exampleRunThread = utFalse;
//     }
//     return exampleRunThread;
// }

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
