//written with some GPT help

#ifndef GPS_BUFFER_H_INCLUDED
#define GPS_BUFFER_H_INCLUDED

#include "gps.h"
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdatomic.h>
#include <unistd.h>

#define GPS_RING_BUFFER_LEN 16

typedef struct {
  raw_gnss_t data_buffer[GPS_RING_BUFFER_LEN];
  int serial_port;
  unsigned int write_loc;
  unsigned int read_loc;
  pthread_mutex_t buffer_mutex;
  atomic_int stop_thread;
  pthread_t thread_id;
} gps_buffer_t;

static gps_buffer_t gps_buffer;

void march_gps_buffer() {
  raw_gnss_t new_data;
  if (read_nav_data(gps_buffer.serial_port, &new_data) > 0) {
    pthread_mutex_lock(&gps_buffer.buffer_mutex);
    gps_buffer.data_buffer[gps_buffer.write_loc] = new_data;
    gps_buffer.write_loc = (gps_buffer.write_loc + 1) % GPS_RING_BUFFER_LEN;

    // Overwrite oldest data if buffer is full
    if (gps_buffer.write_loc == gps_buffer.read_loc) {
      gps_buffer.read_loc = (gps_buffer.read_loc + 1) % GPS_RING_BUFFER_LEN;
    }
    pthread_mutex_unlock(&gps_buffer.buffer_mutex);
  }
}

int read_gps_buffer(raw_gnss_t* out_data) {
  pthread_mutex_lock(&gps_buffer.buffer_mutex);
  if (gps_buffer.read_loc == gps_buffer.write_loc) {
    pthread_mutex_unlock(&gps_buffer.buffer_mutex);
    return -1;
  }

  *out_data = gps_buffer.data_buffer[gps_buffer.read_loc];
  gps_buffer.read_loc = (gps_buffer.read_loc + 1) % GPS_RING_BUFFER_LEN;
  pthread_mutex_unlock(&gps_buffer.buffer_mutex);
  return 1;
}

void* gps_buffer_update_thread(void* arg) {
  (void)arg; // Unused
  while (!atomic_load(&gps_buffer.stop_thread)) {
	  march_gps_buffer();
    usleep(1 * 1000); // Sleep for 10ms
  }
  return NULL;
}

int start_gps_thread(int port) {
  gps_buffer.serial_port = port;
  
  gps_buffer.write_loc = 0;
  gps_buffer.read_loc=0;
  gps_buffer.stop_thread = 0;
  pthread_mutex_init(&gps_buffer.buffer_mutex, NULL);


  int ret = pthread_create(&gps_buffer.thread_id, NULL, &gps_buffer_update_thread, NULL);
  if (ret != 0) {
    fprintf(stderr, "Failed to create GPS buffer thread: %s\n", strerror(ret));
    return -1;
  }

  printf("Launched GPS Thread\n");
  return 1;
}

int create_gps_thread() {
  char* device = getenv("UBLOX_PATH");
  if(device == NULL) {
    device = "/dev/ttyS1";
  }

  printf("Device: %s\n", device); 
  int serial_port = open_serial_port(device);
  printf("Got Serial Port\n");

  if(serial_port < 0) {
    printf("Failed to open port\n");
    exit(1);
  }
  return start_gps_thread(serial_port);
}

void stop_gps_thread() {
  atomic_store(&gps_buffer.stop_thread, 1);
  pthread_join(gps_buffer.thread_id, NULL);
  pthread_mutex_destroy(&gps_buffer.buffer_mutex);
}

#endif

