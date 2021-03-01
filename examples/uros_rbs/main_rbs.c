// Copyright (c) 2020 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <time.h>

/* copied from sporadic.c*/
/****************************************************************************
 * Included Files
 ****************************************************************************/
/*
#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <time.h>

#include "ostest.h"
*/
#ifdef CONFIG_SCHED_SPORADIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* It is actually a better test without schedule locking because that
 * forces the scheduler into an uninteresting fallback mode.
 */

#undef sched_lock
#undef sched_unlock
#define sched_lock()
#define sched_unlock()

#ifndef MIN
# define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_sporadic_sem;
static time_t g_start_time;

static unsigned int sporadic_1_ms_cnt = 0;
static unsigned int sporadic_2_ms_cnt = 0;
static unsigned int fifo_ms_cnt = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void my_mdelay(unsigned int milliseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++)
        {
        }
    }
}

static void *nuisance_func(void *parameter)
{
  /* Synchronized start */

  while (sem_wait(&g_sporadic_sem) < 0);

  /* Sleep until we are cancelled */

  for (;;)
    {
      /* Sleep gracefully for awhile */

      usleep(500*1000);

      /* Then hog some CPU time */

      my_mdelay(100);
    }

  return NULL;
}

static void *fifo_func(void *parameter)
{
  struct sched_param param;
  time_t last;
  time_t now;
  int ret;

  while (sem_wait(&g_sporadic_sem) < 0);

  last  = g_start_time;

  for (;;)
    {
      do
        {
          my_mdelay(1);
          fifo_ms_cnt++;
          sched_lock(); /* Just to exercise more logic */
          ret = sched_getparam(0, &param);
          if (ret < 0)
            {
              printf("ERROR: sched_getparam failed\n");
              return NULL;
            }

          now = time(NULL);
          sched_unlock();
        }
      while (now == last);

      sched_lock(); /* Just to exercise more logic */
      // printf("%4lu FIFO:     %d\n",
      //       (unsigned long)(now-g_start_time), param.sched_priority);
      last = now;
      sched_unlock();
    }
}

static void *sporadic_func(void *parameter)
{
  struct sched_param param;
  time_t last;
  time_t now;
  int prio = 0;
  int ret;
  unsigned int * counter = (unsigned int *) parameter;
  while (sem_wait(&g_sporadic_sem) < 0);

  last  = g_start_time;

  for (;;)
    {
      do
        {
          my_mdelay(1);
          (*counter)++;
          sched_lock(); /* Just to exercise more logic */
          ret = sched_getparam(0, &param);
          if (ret < 0)
            {
              printf("ERROR: sched_getparam failed\n");
              return NULL;
            }

          now = time(NULL);
          sched_unlock();
        }
      while (now == last && prio == param.sched_priority);

      sched_lock(); /* Just to exercise more logic */
      // printf("%4lu SPORADIC 1: %d->%d\n",
      //       (unsigned long)(now-g_start_time), prio, param.sched_priority);
      prio = param.sched_priority;
      last = now;
      sched_unlock();
    }
}

static void *sporadic_func2(void *parameter)
{
  struct sched_param param;
  time_t last;
  time_t now;
  int prio = 0;
  int ret;
  unsigned int * counter = (unsigned int *) parameter;
  while (sem_wait(&g_sporadic_sem) < 0);

  last  = g_start_time;

  for (;;)
    {
      do
        {
          my_mdelay(1);
          (*counter)++;
          sched_lock(); /* Just to exercise more logic */
          ret = sched_getparam(0, &param);
          if (ret < 0)
            {
              printf("ERROR: sched_getparam failed\n");
              return NULL;
            }

          now = time(NULL);
          sched_unlock();
        }
      while (now == last && prio == param.sched_priority);

      sched_lock(); /* Just to exercise more logic */
      // printf("%4lu SPORADIC 2: %d->%d\n",
      //        (unsigned long)(now-g_start_time), prio, param.sched_priority);
      prio = param.sched_priority;
      last = now;
      sched_unlock();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sporadic_test(int budget_1_ns, int budget_2_ns)
{
  pthread_t nuisance_thread = (pthread_t)0;
  pthread_t sporadic_thread = (pthread_t)0;
  pthread_t sporadic_thread2 = (pthread_t)0;
  pthread_t fifo_thread = (pthread_t)0;
#ifdef SDCC
  pthread_addr_t result;
#endif
  FAR void *result;
  struct sched_param myparam;
  struct sched_param sparam;

  int prio_low;
  int prio_mid;
  int prio_high;
  pthread_attr_t attr;
  int ret;

#if CONFIG_SCHED_SPORADIC_MAXREPL < 5
  printf("sporadic_test: CONFIG_SCHED_SPORADIC_MAXREPL is small: %d\n",
         CONFIG_SCHED_SPORADIC_MAXREPL);
  printf("  -- There will some errors in the replenishment interval\n");
#endif

  // printf("sporadic_test: Initializing semaphore to 0\n");
  sem_init(&g_sporadic_sem, 0, 0);

  // initilize global worker-thread millisecons-counters
  sporadic_1_ms_cnt = 0;
  sporadic_2_ms_cnt = 0;
  fifo_ms_cnt = 0;
  
  prio_low = 20;
  prio_mid = 120;
  prio_high = 180;

  ret = sched_getparam(0, &myparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: sched_getparam failed, ret=%d\n", ret);
    }
  /* Temporarily set our priority to prio_high + 2 */
  sparam.sched_priority = prio_high + 2;
  ret = sched_setparam(0, &sparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: sched_setparam failed, ret=%d\n", ret);
    }

  ret = pthread_attr_init(&attr);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_init failed, ret=%d\n",
             ret);
    }

  /* This semaphore will prevent anything from running until we are ready */

  sched_lock();
  sem_init(&g_sporadic_sem, 0, 0);

  /* Start a sporadic thread, with the following parameters: */

  printf("Sporadic thread 1: prio high: %d, low: %d, budget: %d\n",
         prio_high, prio_low, budget_1_ns);

  ret = pthread_attr_setschedpolicy(&attr, SCHED_SPORADIC);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedpolicy failed, ret=%d\n",
             ret);
    }

  sparam.sched_priority               = prio_high;
  sparam.sched_ss_low_priority        = prio_low;
  sparam.sched_ss_repl_period.tv_sec  = 0;
  sparam.sched_ss_repl_period.tv_nsec = 100000000;
  sparam.sched_ss_init_budget.tv_sec  = 0;
  sparam.sched_ss_init_budget.tv_nsec = budget_1_ns;
  sparam.sched_ss_max_repl            = CONFIG_SCHED_SPORADIC_MAXREPL;

  ret = pthread_attr_setschedparam(&attr, &sparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setsched param failed, ret=%d\n",
             ret);
    }

  ret = pthread_create(&sporadic_thread, &attr, sporadic_func, &sporadic_1_ms_cnt);
  if (ret != 0)
    {
      printf("sporadic_test: ERROR: sporadic thread creation failed: %d\n",
             ret);
    }


  printf("sporadic thread 2: at prio high %d low: %d, budget: %d\n",
         prio_high, prio_low, budget_2_ns);

  ret = pthread_attr_setschedpolicy(&attr, SCHED_SPORADIC);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedpolicy failed, ret=%d\n",
             ret);
    }
  struct sched_param sparam2;
  sparam2.sched_priority               = prio_high;
  sparam2.sched_ss_low_priority        = prio_low;
  sparam2.sched_ss_repl_period.tv_sec  = 0;
  sparam2.sched_ss_repl_period.tv_nsec = 100000000;
  sparam2.sched_ss_init_budget.tv_sec  = 0;
  sparam2.sched_ss_init_budget.tv_nsec = budget_2_ns;
  sparam2.sched_ss_max_repl            = CONFIG_SCHED_SPORADIC_MAXREPL;

  ret = pthread_attr_setschedparam(&attr, &sparam2);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setsched param failed, ret=%d\n",
             ret);
    }

  ret = pthread_create(&sporadic_thread2, &attr, sporadic_func2, (pthread_addr_t)&sporadic_2_ms_cnt);
  if (ret != 0)
    {
      printf("sporadic_test: ERROR: sporadic thread creation failed: %d\n",
             ret);
    }

  printf("FIFO thread: prio %d\n", prio_mid);

  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedpolicy failed, ret=%d\n",
             ret);
    }
  struct sched_param sparam3;
  sparam3.sched_priority = prio_mid;
  ret = pthread_attr_setschedparam(&attr, &sparam3);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedparam failed, ret=%d\n",
             ret);
    }

  ret = pthread_create(&fifo_thread, &attr, fifo_func, NULL);
  if (ret != 0)
    {
      printf("sporadic_test: ERROR: FIFO thread creation failed: %d\n",
             ret);
    }

  /*
  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedpolicy failed, ret=%d\n",
             ret);
    }

  sparam.sched_priority = prio_high + 1;
  printf("nuisance thread at priority %d\n", sparam.sched_priority);
  ret = pthread_attr_setschedparam(&attr, &sparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedparam failed, ret=%d\n",
             ret);
    }

  ret = pthread_create(&nuisance_thread, &attr, nuisance_func, NULL);
  if (ret != 0)
    {
      printf("sporadic_test: ERROR: FIFO thread creation failed: %d\n",
             ret);
    }
*/

  g_start_time = time(NULL);

  sem_post(&g_sporadic_sem);
  sem_post(&g_sporadic_sem);
  sem_post(&g_sporadic_sem);

  sleep(10);

  ret = pthread_cancel(sporadic_thread);
  pthread_join(sporadic_thread, &result);

  ret = pthread_cancel(sporadic_thread2);
  pthread_join(sporadic_thread, &result);
  
  ret = pthread_cancel(fifo_thread);
  pthread_join(fifo_thread, &result);

  // ret = pthread_cancel(fifo_thread);
  // pthread_join(fifo_thread, &result);
  sched_unlock();

  printf("Result: sporadic 1 %d ms sporadic 2 %d FIFO %d ms\n",sporadic_1_ms_cnt, sporadic_2_ms_cnt, fifo_ms_cnt);

  sem_destroy(&g_sporadic_sem);
  ret = sched_setparam(0, &myparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: sched_setparam failed, ret=%d\n", ret);
    }
}

#endif /* CONFIG_SCHED_SPORADIC */



#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int uros_rbs_main(int argc, char* argv[])
#endif
{
  int ret;
  int budget_1_ms = 10;
  int budget_2_ms = 30;


  if (argc ==2){
    budget_1_ms = atoi(argv[1]);
    // budget_2_ms = atoi(argv[1]);
  }

  // printf("budget 1: %d ms\n", budget_1_ms);
  // printf("budget 2: %d ms\n", budget_2_ms);
  sporadic_test((budget_1_ms*1000000), (budget_2_ms*1000000));
 return 0;
}

/* Experimental Results: 
Setup 1:
config:
thread 1: 
- SCHED_SPORADIC, 
- prio high 180, prio low 20,
- budget 10ms, period 100ms
- max replenishments = 100

thread 2: 
- FIFO-thread, 
- prio: 120

thread 3: 
- SCHED_SPORADIC, 
- prio high 179, prio low 19,
- budget 10ms, period 100ms
- max replenishments = 100

Hardware: Olimex board (STM32), NuttX OS

Experiment:
- callback function in each thread has a busy_loop of 1ms and increments a counter
- experiment runs for 10 seconds
- at the end the counter values of all threads are reported, e.g. the number of milliseconds
  the thread could execute in interval of 10 seconds (total 10000 milliseconds)

Exp 1: (one sporadic thread and FIFO thread)
configuration with 
- thread 1:  sporadic thread with budget = x ms and period=100ms
- thread 2: low-prio FIFO thread

config      result      result
sporadic 1  sporadic 1  fifo
budget(ms)  (ms)        (ms) 
---------------------------------
0            96         9815
10         1074         8837
20         2043         7868
30         3014         6896
40         3985         5925
50         4956         4953
60         5920         3990
70         6899         3010
80         7870         2039
90         8804         1105
100        9910            0

Exp 2 (two sporadic threads and FIFO thread)

Keep sporadic thread 2 with 30/100ms budget/period, vary budget of thread 1 from 0 - 100ms

configuration with 
- thread 1:  sporadic thread with budget = x ms and  period=100ms, prio see above
- thread 3:  sporadic thread with budget = 30 ms and period=100ms, prio see above
- thread 2: low-prio FIFO thread, prio see above

config      result      result      result
sporadic 1  sporadic 1  sporadic 2  fifo
budget(ms)  (ms)        (ms)        (ms) 
------------------------------------------
0            145         981       8784
10          1073         971       7864
20          2044          10       7854
30          3013           0       6895
40          9909           0          0
50          9909           0          0
60          9909           0          0
70          9909           0          0       
80          9908           0          0
90          9908           0          0
100         9909           0          0

Exp 3 (two sporadic threads and FIFO thread)

Keep sporadic thread 1 with 30/100ms budget, vary budget of thread 2 from 0 - 100ms

configuration with 
- thread 1:  sporadic thread with budget = 30 ms and period=100ms, prio see above
- thread 3:  sporadic thread with budget = x ms and  period=100ms, prio see above
- thread 2: low-prio FIFO thread, prio see above


config      result      result      result
sporadic 2  sporadic 1  sporadic 2  fifo
budget(ms)  (ms)        (ms)        (ms) 
----------------------------------------
0           5246        4661            0    
10          7091        2816            0         
20          9132        776             0
30          3015           0         6892
40          3016           9         6883
50          3015          49         6844
60          3016        2311         4581
70          3065        2484         4359    
80          3015          48         6845
90          3015          91         6802 
100         3053        4726         2128 



Exp 4 (two sporadic threads and FIFO thread)

Same as Experiment 1, but both sporadic threads with the same priority settings
- sporadic 1: high prio 180, low prio 20
- sporadic 2: high prio 180, low prio 20
- fifo      : prio 120

config      result      result      result
sporadic 1  sporadic 1  sporadic 2  fifo
budget(ms)  (ms)        (ms)        (ms) 
------------------------------------------
0          144            981         8784
10        1073            971         7864
20        2044             10         7854
30        3015              0         6892
40          39           3044         6825   
50        4957           4950            0
60        4427           1559         3923
70        6880           3028            0
80        7840           2066            0
90        8802           1105            0
100       9861             47            0


Example raw output:
Sporadic thread 1: prio high: 180, low: 20, budget: 10000000
pthread_create: budget 0 s 10000000 ns ticks: 10 , period 0 s 100000000 ns ticks 100 
thread id 8
sporadic thread 2: at prio high 179 low: 19, budget: 30000000
pthread_create: budget 0 s 30000000 ns ticks: 30 , period 0 s 100000000 ns ticks 100 
thread id 9
FIFO thread: prio 120
thread id 10
Result: sporadic 1 1074 ms sporadic 2 19 FIFO 8816 ms

*/
