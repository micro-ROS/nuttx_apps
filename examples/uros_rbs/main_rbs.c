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


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>


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
          my_mdelay(100);  
          usleep(100000);  
          //fifo_ms_cnt += 4;
          fifo_ms_cnt +=1;
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
      //printf("%4lu FIFO:     %d\n",
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
          (sporadic_1_ms_cnt)++;
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
          (sporadic_2_ms_cnt)++;
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
      //       (unsigned long)(now-g_start_time), prio, param.sched_priority);
      prio = param.sched_priority;
      last = now;
      sched_unlock();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sporadic_test(void)
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
  int prio_min;
  int prio_max;
  int prio_low;
  int prio_mid;
  int prio_high;
  pthread_attr_t attr, attr_spor, attr_spor2, attr_fifo;
  int ret;

#if CONFIG_SCHED_SPORADIC_MAXREPL < 5
  printf("sporadic_test: CONFIG_SCHED_SPORADIC_MAXREPL is small: %d\n",
         CONFIG_SCHED_SPORADIC_MAXREPL);
  printf("  -- There will some errors in the replenishment interval\n");
#endif

  printf("sporadic_test: Initializing semaphore to 0\n");
  sem_init(&g_sporadic_sem, 0, 0);

  sporadic_1_ms_cnt = 0;
  sporadic_2_ms_cnt = 0;
  fifo_ms_cnt = 0;
  
  prio_min  = sched_get_priority_min(SCHED_FIFO);
  prio_max  = sched_get_priority_max(SCHED_FIFO);

  
  // prio_low  = prio_min + ((prio_max - prio_min) >> 2);
  // prio_mid  = (prio_min + prio_max) >> 1;
  // prio_high = prio_max - ((prio_max - prio_min) >> 2);
  
  prio_low = 20;
  prio_mid = 120;
  prio_high = 180;
  // Temporarily set our priority to prio_high + 2 

  ret = sched_getparam(0, &myparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: sched_getparam failed, ret=%d\n", ret);
    }

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

  // This semaphore will prevent anything from running until we are ready 

  sched_lock();
  sem_init(&g_sporadic_sem, 0, 0);

  // Start a FIFO thread at the highest priority (prio_max + 1)

  printf("sporadic_test: Starting FIFO thread at priority %d\n", prio_mid);

  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedpolicy failed, ret=%d\n",
             ret);
    }

  sparam.sched_priority = prio_high + 1;
  ret = pthread_attr_setschedparam(&attr, &sparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedparam failed, ret=%d\n",
             ret);
    }
/*
  ret = pthread_create(&nuisance_thread, &attr, nuisance_func, NULL);
  if (ret != 0)
    {
      printf("sporadic_test: ERROR: FIFO thread creation failed: %d\n",
             ret);
    }
*/
  /* Start a FIFO thread at the middle priority */
/*
  sparam.sched_priority = prio_mid;
  ret = pthread_attr_setschedparam(&attr, &sparam);
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
*/
  /* Start a sporadic thread, with the following parameters: */

  printf("sporadic_test: Starting sporadic thread at priority %d\n",
         prio_high, prio_low);
  ret = pthread_attr_init(&attr_spor);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_init failed, ret=%d\n",
             ret);
    }
  ret = pthread_attr_setschedpolicy(&attr_spor, SCHED_SPORADIC);
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
  sparam.sched_ss_init_budget.tv_nsec = 30000000;
  sparam.sched_ss_max_repl            = CONFIG_SCHED_SPORADIC_MAXREPL;

  ret = pthread_attr_setschedparam(&attr_spor, &sparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setsched param failed, ret=%d\n",
             ret);
    }

  ret = pthread_create(&sporadic_thread, &attr_spor, sporadic_func, &sporadic_1_ms_cnt);
  if (ret != 0)
    {
      printf("sporadic_test: ERROR: sporadic thread creation failed: %d\n",
             ret);
    }
/*
  // second sporadic thread
  printf("sporadic_test: Starting sporadic thread 2 at priority %d\n",
         prio_high, prio_low);
  ret = pthread_attr_init(&attr_spor2);
  ret = pthread_attr_setschedpolicy(&attr_spor2, SCHED_SPORADIC);
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
  sparam2.sched_ss_init_budget.tv_nsec = 30000000;
  sparam2.sched_ss_max_repl            = CONFIG_SCHED_SPORADIC_MAXREPL;

  ret = pthread_attr_setschedparam(&attr_spor2, &sparam2);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setsched param failed, ret=%d\n",
             ret);
    }

  ret = pthread_create(&sporadic_thread2, &attr_spor2, sporadic_func2, (pthread_addr_t)&sporadic_2_ms_cnt);
  if (ret != 0)
    {
      printf("sporadic_test: ERROR: sporadic thread creation failed: %d\n",
             ret);
    }
*/
  printf("sporadic_test: Starting FIFO thread at priority %d\n", prio_mid);
  ret = pthread_attr_init(&attr_fifo);
  ret = pthread_attr_setschedpolicy(&attr_fifo, SCHED_FIFO);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedpolicy failed, ret=%d\n",
             ret);
    }
  struct sched_param sparam3;
  sparam3.sched_priority = prio_mid;
  ret = pthread_attr_setschedparam(&attr_fifo, &sparam3);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedparam failed, ret=%d\n",
             ret);
    }

  ret = pthread_create(&fifo_thread, &attr_fifo, fifo_func, NULL);
  if (ret != 0)
    {
      printf("sporadic_test: ERROR: FIFO thread creation failed: %d\n",
             ret);
    }
  g_start_time = time(NULL);

  sem_post(&g_sporadic_sem);
  sem_post(&g_sporadic_sem);
  sem_post(&g_sporadic_sem);
/*
  // Wait a while then kill the FIFO thread 

  sleep(15);
  ret = pthread_cancel(fifo_thread);
  pthread_join(fifo_thread, &result);

  /// Wait a bit longer then kill the nuisance thread 

  sleep(10);
  ret = pthread_cancel(nuisance_thread);
  pthread_join(nuisance_thread, &result);

  // Wait a bit longer then kill the sporadic thread 
*/
  sleep(10);

  ret = pthread_cancel(sporadic_thread);
  //pthread_join(sporadic_thread, &result);

  // ret = pthread_cancel(sporadic_thread2);
  //pthread_join(sporadic_thread, &result);
  
  ret = pthread_cancel(fifo_thread);
  //pthread_join(fifo_thread, &result);
  sched_unlock();
  unsigned int sum = sporadic_1_ms_cnt + fifo_ms_cnt;
  printf("sporadic_test: sporadic 1 %d ms sporadic 2 %d FIFO %d ms \n \
          percentages: sporadic %.2f  FIFO %.2f \n",
  sporadic_1_ms_cnt, sporadic_2_ms_cnt, fifo_ms_cnt,
  ((float)sporadic_1_ms_cnt/(float)sum)*100,
  ((float)fifo_ms_cnt/(float)sum)*100);

  printf("sporadic_test: Done\n");
  sem_destroy(&g_sporadic_sem);

  ret = sched_setparam(0, &myparam);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: sched_setparam failed, ret=%d\n", ret);
    }
}

#endif /* CONFIG_SCHED_SPORADIC */

/*
#define STRING_BUFFER_LEN 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){\
   printf("Failed status on line %d: %d: %s. Aborting.\n",__LINE__,(int)temp_rc, rcl_get_error_string().str);\
   rcutils_reset_error(); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d:\
   %s. Continuing.\n",__LINE__,(int)temp_rc, rcl_get_error_string().str);rcutils_reset_error();}}
#define RCUNUSED(fn) { rcl_ret_t temp_rc __attribute__((unused)); temp_rc = fn; }


rcl_subscription_t high_ping_subscription_;
std_msgs__msg__Int32 high_ping_msg_;
rcl_publisher_t high_pong_publisher_;
rcl_subscription_t low_ping_subscription_;
std_msgs__msg__Int32 low_ping_msg_;
rcl_publisher_t low_pong_publisher_;
*/
/*
void burn_cpu_cycles_high(long duration)
{
  if (duration > 0) {
    clockid_t clockId;
    clockId = CLOCK_REALTIME;
    // clock_getcpuclockid(pthread_self(), &clockId);
    // NuttX does not implement this function
    // -lrt  is not available , I did not find this function in 
    // NuttX subfolders
    // adding EXTRA_LIBS to the Makefile did not resolve the error.
    struct timespec startTimeP;
    clock_gettime(clockId, &startTimeP);
    int x = 0;
    bool doAgain = true;
    while (doAgain) {
      while (x != rand() && x % 10 != 0) {
        x++;
      }
      struct timespec currentTimeP;
      clock_gettime(clockId, &currentTimeP);
      long currentDuration = (currentTimeP.tv_sec - startTimeP.tv_sec) * 1000000000 + (currentTimeP.tv_nsec - startTimeP.tv_nsec);
      doAgain = (currentDuration < duration);
    }
  }
}

void burn_cpu_cycles_low(long duration)
{
  if (duration > 0) {
    clockid_t clockId;
    clockId = CLOCK_REALTIME;
    // clock_getcpuclockid(pthread_self(), &clockId);
    // NuttX does not implement this function
    // -lrt  is not available , I did not find this function in 
    // NuttX subfolders
    // adding EXTRA_LIBS to the Makefile did not resolve the error.
    struct timespec startTimeP;
    clock_gettime(clockId, &startTimeP);
    int x = 0;
    bool doAgain = true;
    while (doAgain) {
      while (x != rand() && x % 10 != 0) {
        x++;
      }
      struct timespec currentTimeP;
      clock_gettime(clockId, &currentTimeP);
      long currentDuration = (currentTimeP.tv_sec - startTimeP.tv_sec) * 1000000000 + (currentTimeP.tv_nsec - startTimeP.tv_nsec);
      doAgain = (currentDuration < duration);
    }
  }
}



void high_ping_received(const void * pong_msg)
{
  burn_cpu_cycles_high(10000000);  // 10ms TODO: Get this value from parameter 'high_busyloop'.
  printf("high ping received.\n");
  RCUNUSED(rcl_publish(&high_pong_publisher_, pong_msg, NULL));
}


void low_ping_received(const void * pong_msg)
{
  burn_cpu_cycles_low(10000000);  // 10ms TODO: Get this value from parameter 'low_busyloop'.
  printf("low ping received.\n");
  RCUNUSED(rcl_publish(&low_pong_publisher_, pong_msg, NULL));
}
*/
/*
void * thread_run(void * arg)
{
  int * value = (int *) arg;
  printf("thread_run 1 %d\n",(*value));
    int duration = 100000000;  // 100ms
    while(1) {
    clockid_t clockId;
    clockId = CLOCK_REALTIME;
    // clock_getcpuclockid(pthread_self(), &clockId);
    // NuttX does not implement this function
    // -lrt  is not available , I did not find this function in 
    // NuttX subfolders
    // adding EXTRA_LIBS to the Makefile did not resolve the error.
    struct timespec startTimeP;
    clock_gettime(clockId, &startTimeP);
    int x = 0;
    bool doAgain = true;
    while (doAgain) {
      while (x != rand() && x % 10 != 0) {
        x++;
      }
      struct timespec currentTimeP;
      clock_gettime(clockId, &currentTimeP);
      long currentDuration = (currentTimeP.tv_sec - startTimeP.tv_sec) * 1000000000 + (currentTimeP.tv_nsec - startTimeP.tv_nsec);
      doAgain = (currentDuration < duration);
    }
  }
}
*/

/*
static
unsigned int thread_1_cnt =0, thread_2_cnt = 0;

static
void * worker_thread_run(void * arg)
{
  int ret;
  int * thread_id = (int *) arg;
  int sched_policy = 0;
  struct sched_param p;

#ifdef CONFIG_SCHED_SPORADIC
  printf ("CONFIG_SCHED_SPORADIC defined.\n");
#endif

  ret = nxsched_getparam(pthread_self(), &p);
  if (ret != 0)
  {
    printf("worker thread %d: nxsched_getparam failed: %d\n" , (*thread_id), ret);
    return;
  }
    printf("running worker-thread %d -1-: prio %d budget %d ns period %d ns\n",(*thread_id), p.sched_priority, 
    p.sched_ss_init_budget.tv_nsec,p.sched_ss_repl_period.tv_nsec);


  ret = pthread_getschedparam(pthread_self(), &sched_policy, &p);
  if (ret != 0)
  {
    printf("worker thread %d: sched_getparam failed: %d\n" , (*thread_id), ret);
    return;
  }
    printf("running worker-thread %d:-2- policy %d prio %d budget %d ns period %d ns\n",(*thread_id), sched_policy, p.sched_priority, 
    p.sched_ss_init_budget.tv_nsec,p.sched_ss_repl_period.tv_nsec);

  // printf("running worker-thread %d: prio %d budget %lf ms period %lf ms\n",(*thread_id), p.sched_priority, 
  //  (double) p.sched_ss_init_budget.tv_nsec/(double)1000000,(double)p.sched_ss_repl_period.tv_nsec/(double)1000000);
  
  unsigned int milliseconds = 1000;
  // sleep(1);
  while(1)
  {
    volatile unsigned int i;
    volatile unsigned int j;
    printf("thread %d\n", (*thread_id));
    for (i = 0; i < milliseconds; i++)
      {
        
        for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++)
          {
          }
          if ((*thread_id)== 1)
          {
            thread_1_cnt++; // count progress in milliseconds
          }
          if ((*thread_id)== 2)
          {
            thread_2_cnt++; // count progress in milliseconds
          }
      }
  }
}

void test_sporadic_scheduling(int thr1_prio_high, int thr1_prio_low, int thr1_budget_ns, int thr1_period_ns, 
                              int thr2_prio_high, int thr2_prio_low, int thr2_budget_ns, int thr2_period_ns 
) 
{
  struct sched_param sparam_high, sparam_low;
  int ret;

  thread_1_cnt = 0;
  thread_2_cnt = 0;

  sparam_high.sched_priority               = thr1_prio_high;
  sparam_high.sched_ss_low_priority        = thr1_prio_low;
  sparam_high.sched_ss_repl_period.tv_sec  = 0;
  sparam_high.sched_ss_repl_period.tv_nsec = thr1_period_ns;
  sparam_high.sched_ss_init_budget.tv_sec  = 0;
  sparam_high.sched_ss_init_budget.tv_nsec = thr1_budget_ns;
  sparam_high.sched_ss_max_repl            = CONFIG_SCHED_SPORADIC_MAXREPL;
  
  sparam_low.sched_priority               = thr2_prio_high;
  sparam_low.sched_ss_low_priority        = thr2_prio_low;
  sparam_low.sched_ss_repl_period.tv_sec  = 0;
  sparam_low.sched_ss_repl_period.tv_nsec = thr2_period_ns;
  sparam_low.sched_ss_init_budget.tv_sec  = 0;
  sparam_low.sched_ss_init_budget.tv_nsec = thr2_budget_ns;
  sparam_low.sched_ss_max_repl            = CONFIG_SCHED_SPORADIC_MAXREPL;

  printf("thread 1: high prio %d low prio %d budget %d ms  period %d ms \n", 
    sparam_high.sched_priority, sparam_high.sched_ss_low_priority, 
    sparam_high.sched_ss_init_budget.tv_nsec/1000000, sparam_high.sched_ss_repl_period.tv_nsec / 1000000);
  printf("thread 2: high prio %d low prio %d budget %d ms  period %d ms \n", 
  sparam_low.sched_priority, sparam_low.sched_ss_low_priority, 
  sparam_low.sched_ss_init_budget.tv_nsec/1000000, sparam_low.sched_ss_repl_period.tv_nsec / 1000000);

 // first worker thread
  pthread_attr_t attr_high, attr_low;
  pthread_t worker_thread_high, worker_thread_low;
  int id_high=1, id_low=2;

  (void)pthread_attr_init(&attr_high);
  #ifdef _POSIX_SPORADIC_SERVER
  printf("var _POSIX_SPORADIC_SERVER defined\n");
  #endif
  printf("SCHED_SPORADIC = %d\n",SCHED_SPORADIC);
  ret = pthread_attr_setschedpolicy(&attr_high, SCHED_SPORADIC);
  if (ret != OK)
  {
    printf("sporadic_test: ERROR: pthread_attr_setschedpolicy failed, ret=%d\n",
            ret);
  }
  (void)pthread_attr_setschedparam(&attr_high, &sparam_high);

  printf("param policy %d low_prio %d budget %d period %d\n", 
      attr_high.policy,
      attr_high.low_priority,
      attr_high.budget.tv_nsec,
      attr_high.repl_period.tv_nsec);

  ret = pthread_create(&worker_thread_high, &attr_high, &worker_thread_run, &id_high);
  if (ret != 0)
  {
    printf("uros_rbs: pthread_create worker_thread_high failed: %d\n", ret);
    return 1;
    } else {
      printf("uros_rbs: started worker_thread_high\n");
    }

   // second worker thread 
  (void)pthread_attr_init(&attr_low);
  (void)pthread_attr_setschedparam(&attr_low, &sparam_low);
  ret = pthread_attr_setschedpolicy(&attr_low, SCHED_SPORADIC);
  if (ret != OK)
    {
      printf("sporadic_test: ERROR: pthread_attr_setschedpolicy failed, ret=%d\n",
             ret);
    }
  ret = pthread_create(&worker_thread_low, &attr_low, &worker_thread_run, &id_low);
  if (ret != 0)
    {
      printf("uros_rbs: pthread_create worker_thread_low failed: %d\n", ret);
      return 1;
    } else {
      printf("uros_rbs: started worker_thread_low\n");
    }


// debugging parameters:
  struct sched_param p;

  ret = nxsched_getparam(worker_thread_high, &p);
  if (ret != 0)
  {
    printf("worker thread %d: nxsched_getparam failed: %d\n" , worker_thread_high, ret);
    return;
  }
  printf("running worker-thread-high: pid %d prio %d budget %d ns period %d ns\n",worker_thread_high, p.sched_priority, 
  p.sched_ss_init_budget.tv_nsec, p.sched_ss_repl_period.tv_nsec);

  ret = nxsched_getparam(worker_thread_low, &p);
  if (ret != 0)
  {
    printf("worker thread %d: nxsched_getparam failed: %d\n" , worker_thread_low, ret);
    return;
  }
  printf("running worker-thread-high: pid %d prio %d budget %d ns period %d ns\n",worker_thread_low, p.sched_priority, 
  p.sched_ss_init_budget.tv_nsec, p.sched_ss_repl_period.tv_nsec);

  for(unsigned int i=0;i<10;i++)
  {
    // printf("main sleeping\n");
    sleep(1);
  }

  ret = pthread_cancel(worker_thread_high);
  if (ret != 0)
    {
      fprintf(stderr, "uros_rbs: pthread_cancel worker_thread_high failed\n");
    }
  else
    {
      printf("uros_rbs: worker_thread_high returned.\n");
    }

  ret = pthread_cancel(worker_thread_low);
  if (ret != 0)
    {
      fprintf(stderr, "uros_rbs: pthread_cancel worker_thread_low failed\n");
    }
  else
    {
      printf("uros_rbs: worker_thread_low returned.\n");
    }

    printf("progress high %d ms progress low %d ms\n", thread_1_cnt, thread_2_cnt);
}
*/

#if defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int uros_rbs_main(int argc, char* argv[])
#endif
{
  // rcl_allocator_t allocator = rcl_get_default_allocator();
  // rclc_support_t support;
  int ret;

  sporadic_test();
  // test_sporadic_scheduling(40, 4, 4000000, 10000000,
  //                         50, 5, 2000000, 10000000);



/*

  // Set the executor thread priority 
  struct sched_param exe_param;
  exe_param.sched_priority = 100;
  ret = sched_setparam(0, &exe_param);
  if (ret < 0)
  {
    printf("uros_rbs: sched_setparam failed: %d\n" , ret);
    return 1;
  }


  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rclc_node_init_default(&node, "pong_rclc_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(&high_pong_publisher_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "high_pong"));
  RCCHECK(rclc_subscription_init_best_effort(&high_ping_subscription_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "high_ping"));

  RCCHECK(rclc_publisher_init_best_effort(&low_pong_publisher_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "low_pong"));
  RCCHECK(rclc_subscription_init_best_effort(&low_ping_subscription_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "low_ping"));

  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  struct sched_param sparam_high, sparam_low;
  //sparam_high.priority = sched_get_priority_max(SCHED_FIFO);
  //sparam_low.priority = sched_get_priority_min(SCHED_FIFO);
  printf("max prio %d min prio %d\n", SCHED_PRIORITY_MAX, SCHED_PRIORITY_MIN);

  sparam_high.sched_priority               = 60;
  sparam_high.sched_ss_low_priority        = 3;
  sparam_high.sched_ss_repl_period.tv_sec  = 0;
  sparam_high.sched_ss_repl_period.tv_nsec = 10000000;  // 10ms
  sparam_high.sched_ss_init_budget.tv_sec  = 1;
  sparam_high.sched_ss_init_budget.tv_nsec = 6000000;   // 6ms
  sparam_high.sched_ss_max_repl            = CONFIG_SCHED_SPORADIC_MAXREPL;
  
  sparam_low.sched_priority               = 50;
  sparam_low.sched_ss_low_priority        = 2;
  sparam_low.sched_ss_repl_period.tv_sec  = 0;
  sparam_low.sched_ss_repl_period.tv_nsec = 10000000;   // 10ms
  sparam_low.sched_ss_init_budget.tv_sec  = 1;
  sparam_low.sched_ss_init_budget.tv_nsec = 4000000;    // 4ms
  sparam_low.sched_ss_max_repl            = CONFIG_SCHED_SPORADIC_MAXREPL;
  printf("high prio %d low prio %d\n", sparam_high.sched_priority, sparam_low.sched_priority);

  printf("subscription high ping \n");
  RCCHECK(rclc_executor_add_subscription_sched(&executor, &high_ping_subscription_, &high_ping_msg_, &high_ping_received, ON_NEW_DATA, &sparam_high));

  printf("subscription low ping \n");
  RCCHECK(rclc_executor_add_subscription_sched(&executor, &low_ping_subscription_, &low_ping_msg_, &low_ping_received, ON_NEW_DATA, &sparam_low));
 
  // int sched_policy = SCHED_SPORADIC;
  int sched_policy = SCHED_FIFO;

  RCCHECK(rclc_executor_start_multi_threading_for_nuttx(&executor, sched_policy));

  printf("clean up \n");
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_subscription_fini(&high_ping_subscription_, &node));
  RCCHECK(rcl_publisher_fini(&high_pong_publisher_, &node));
  RCCHECK(rcl_subscription_fini(&low_ping_subscription_, &node));  
  RCCHECK(rcl_publisher_fini(&low_pong_publisher_, &node));

  RCCHECK(rcl_node_fini(&node));
  */
 return 0;
}
/*
Error message missing library rt for clock_getcpuclockid(pthread_self(), &clockId);
but this function is not implemented in NuttX! (01-02-2021)

arm-none-eabi-ld --entry=__start -nostartfiles -nodefaultlibs -g -T/home/jst3si/olimex_ws/firmware/NuttX/configs/olimex-stm32-e407/scripts/ld.script -L"/home/jst3si/olimex_ws/firmware/NuttX/staging" -L"/home/jst3si/olimex_ws/firmware/NuttX/arch/arm/src/board" -L "/usr/lib/gcc/arm-none-eabi/6.3.1/../../../arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard" \
	-o "/home/jst3si/olimex_ws/firmware/NuttX/nuttx"   \
	--start-group -lsched -ldrivers -lconfigs -lc -lmm -larch -lxx -lapps -lnet -lfs -lbinfmt -lxx -lboard -lsupc++ "/usr/lib/gcc/arm-none-eabi/6.3.1/thumb/v7e-m/fpv4-sp/hard/libgcc.a" --end-group
/home/jst3si/olimex_ws/firmware/NuttX/staging/libapps.a(main_rbs.o): In function `burn_cpu_cycles':
/home/jst3si/olimex_ws/firmware/apps/examples/uros_rbs/main_rbs.c:49: undefined reference to `clock_getcpuclockid'
Makefile:184: recipe for target 'nuttx' failed
make[1]: *** [nuttx] Error 1
make[1]: Verzeichnis „/home/jst3si/olimex_ws/firmware/NuttX/arch/arm/src“ wird verlassen
tools/Makefile.unix:422: recipe for target 'pass2' failed
make: *** [pass2] Error 2


*/
