#include <Arduino.h>
#include <time.h>

#define PI_INTERRUPT  B12
#define SEND_SIZE 28 // 4 bytes of data and a check sum
#define REC_SIZE 5*4

// char send[SEND_SIZE];  // 4 floats, 16 bytes
// byte received[REC_SIZE];
// float f = 0;
// float sum = 0;
// unsigned char * p;
// unsigned long start;
// unsigned long read_time;
// unsigned long write_time = 0;

void coms_init();
bool read_from_pi(uint8_t *buffer, float *float_values);
bool read_from_pi_pid(uint8_t *buffer, float *float_values);
void write_to_pi(uint8_t *buffer);
void read_shorts_from_pi(uint8_t *buffer, float *float_vals, const size_t num_vals);
int buffer_to_int(uint8_t *buffer);
void read_floats_from_pi(uint8_t *buffer, float *float_vals, const size_t num_vals);