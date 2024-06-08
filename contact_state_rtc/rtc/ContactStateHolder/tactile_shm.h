#ifndef __TACTILE_SHM_H__
#define __TACTILE_SHM_H__
#include <pthread.h>

#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <stdio.h>
#include <errno.h>

#define MAX_SENSOR_NUM 1000000

struct tactile_shm {
   double contact_force[MAX_SENSOR_NUM][3];

#ifdef USE_PTHREAD_MUTEX
  pthread_mutex_t cmd_lock;
  pthread_mutex_t info_lock;
#else
  int cmd_lock;
  int info_lock;
#endif
};

void *set_shared_memory(key_t _key, size_t _size);

#endif
