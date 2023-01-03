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

void loop() {
  // read from Serial3 to get input velocity values
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

  // calculate global time values
  curr_time_micros = micros();
  float dt = ((float) (curr_time_micros - prev_time_micros))/(1.0e6);
  prev_time_micros = curr_time_micros;

  drive_x = input_x/255.0;
  drive_y = input_y/255.0;

  drive_xf = af * drive_xf + bf * drive_x + bf * prev_drive_x;
  prev_drive_x = drive_x;
  drive_yf = af * drive_yf + bf * drive_y + bf * prev_drive_y;
  prev_drive_y = drive_y;

  drive_robot(drive_xf, drive_yf, 0, dt);
}