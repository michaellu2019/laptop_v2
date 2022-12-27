#include "_constants.h";

// serial output
int speedX = 0;
int speedY = 0;
char output_buffer[32];

void setup() {
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(rc_channel1_reader.pin), read_rc_channel<CHANNEL_ONE - 1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rc_channel2_reader.pin), read_rc_channel<CHANNEL_TWO - 1>, CHANGE);
}

void loop() {
  rc_channel1_reader.clip_pulse_duration();
  rc_channel2_reader.clip_pulse_duration();

  speedX = map(rc_channel1_reader.clipped_pulse_duration, RC_PWM_MIN, RC_PWM_MAX, -255, 255);
  speedY = map(rc_channel2_reader.clipped_pulse_duration, RC_PWM_MIN, RC_PWM_MAX, -255, 255);
  if (abs(speedX) < 20) speedX = 0;
  if (abs(speedY) < 20) speedY = 0;

  sprintf(output_buffer, "%dx,%dy,", speedX, speedY);
  Serial.println(output_buffer);
}

template <int j>
void read_rc_channel() {
  rc_channel_readers[j]->current_read_time = micros();
  if (rc_channel_readers[j]->current_read_time > rc_channel_readers[j]->last_read_time) {
    rc_channel_readers[j]->pulse_duration = rc_channel_readers[j]->current_read_time - rc_channel_readers[j]->last_read_time;
    rc_channel_readers[j]->last_read_time = rc_channel_readers[j]->current_read_time;
  }
}
