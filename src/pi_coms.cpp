#include <pi_coms.h>

// void coms_init() {
//   Serial.begin(1000000);
//   Serial.setTimeout(1);
//   delay(100);
// }

//stores as a float
void read_shorts_from_pi(uint8_t *buffer, float *float_vals, const size_t num_vals){
    for (int i = 0; i < num_vals; i++) {
      short s = 0;
      memcpy(&s, &(buffer[__SIZEOF_SHORT__*i]), __SIZEOF_SHORT__);
      float_vals[i] = (float) (s % 256)*256 + (s/256);
  }
}

int buffer_to_int(uint8_t *buffer){
  int ret_val;
  memcpy(&ret_val, &buffer[0], __SIZEOF_INT__);
  return ret_val;
}

void read_floats_from_pi(uint8_t *buffer, float *float_vals, const size_t num_vals){
  float f;
  for (int i = 0; i < num_vals; i++) {
    memcpy(&f, &(buffer[__SIZEOF_FLOAT__*i]), __SIZEOF_FLOAT__);
    float_vals[i] = f;
  }
}


bool read_from_pi(uint8_t *buffer, float *float_values) {

  float f;
  float sum = 0;
  Serial.readBytes(buffer, 20);  
  for (int i = 0; i < 4; i++) {
    memcpy(&f, &(buffer[4*i]), 4);
    float_values[i] = f;
    sum = sum + f;
  }
  memcpy(&f, &(buffer[16]), 4);
  if (abs(sum-f) < 0.1) {
    return true;
  }
  return false;
  
}

bool read_from_pi_pid(uint8_t *buffer, float *float_values) {
  // start = micros();
  float f;
  float sum = 0;
  Serial.readBytes(buffer, 16);  
  for (int i = 0; i < 3; i++) {
    memcpy(&f, &(buffer[4*i]), 4);
    float_values[i] = f;
    sum = sum + f;
  }
  memcpy(&f, &(buffer[12]), 4);
  if (abs(sum-f) < 0.1) {
    return true;
  }
  return false;
  // read_time = micros() - start;
}

void write_to_pi(uint8_t *buffer) {
  // start = micros();
  for (int i = 0; i<SEND_SIZE; i++) {
    Serial.write(buffer[i]);
  }
  // write_time = micros() - start;
}