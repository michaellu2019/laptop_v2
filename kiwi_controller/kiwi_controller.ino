#include "util/atomic.h"
#include "_constants.h"

char input_buffer[40];

void setup() {
  // serial
  Serial.begin(115200);
  Serial.println("INIT");

  Serial3.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  // radio
  // radio.begin();
  // radio.openReadingPipe(0, address);
  // radio.setPALevel(RF24_PA_MAX);
  // radio.startListening();
  
  // motors
  attachInterrupt(digitalPinToInterrupt(left_drive_motor.ENCA), read_drive_motor_encoder<LEFT_DRIVE_MOTOR>, RISING);
  attachInterrupt(digitalPinToInterrupt(right_drive_motor.ENCA), read_drive_motor_encoder<RIGHT_DRIVE_MOTOR>, RISING);
  attachInterrupt(digitalPinToInterrupt(back_drive_motor.ENCA), read_drive_motor_encoder<BACK_DRIVE_MOTOR>, RISING);
}

float test_duration = 0; 
int input_x;
int input_y;
float drive_x;
float drive_y;

void loop() {
  while (Serial3.available() > 0) {
    char c;
    c = Serial3.read();
    if (c == ',') {
      if (strchr(input_buffer, 'x')) input_x = atoi(input_buffer);
      if (strchr(input_buffer, 'y')) input_y = atoi(input_buffer);

      memset(input_buffer, 0, strlen(input_buffer));
    } else {     
      strncat(input_buffer, &c, 1);
    }
  }

  // char output_buffer[40];
  // sprintf(output_buffer, "%d, %d --> ", input_x, input_y);
  // Serial.print(output_buffer);

  current_time_micros = micros();
  float dt = ((float) (current_time_micros - prev_time_micros))/(1.0e6);
  prev_time_micros = current_time_micros;
//  left_drive_motor.set_drive_speed(0.2, dt);
//  right_drive_motor.set_drive_speed(0.2, dt);
//  back_drive_motor.set_drive_speed(0.2, dt);

  test_duration += dt;

  drive_x = input_x/255.0;
  drive_y = input_y/255.0;

  drive_robot(drive_x, drive_y, 0, dt);

  // if (test_duration < 4.0) {
  //   drive_robot(0, 1, 0, dt);
  // } else if (test_duration < 8.0) {
  //   drive_robot(0, -1, 0, dt);
  // } else if (test_duration < 12.0) {
  //   drive_robot(1, 0, 0, dt);
  // } else if (test_duration < 16.0) {
  //   drive_robot(-1, 0, 0, dt);
  // } else {
  //   drive_robot(0, 0, 0, dt);
  // }
}