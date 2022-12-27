#define NUM_INTERRUPT_PINS 2

#define CHANNEL_ONE 1
#define CHANNEL_TWO 2

#define RC_PWM_MIN 1000
#define RC_PWM_MAX 1800

class RCChannelReader {
  public:
    int channel;
    int id;
    int clipped_pulse_duration;
    int pin;
    volatile long pulse_duration;
    volatile long current_read_time;
    volatile long last_read_time;

  public:
    RCChannelReader(int id, int channel, int pin) {
      this->id = id;
      this->channel = channel;
      this->pin = pin;
      this->pulse_duration = 0;
      this->clipped_pulse_duration = 0;
      this->current_read_time = 0;
      this->last_read_time = 0;

      pinMode(pin, INPUT_PULLUP);
    }

    void clip_pulse_duration() {
      if (this->pulse_duration < 2000) {
        this->clipped_pulse_duration = this->pulse_duration;
      }
      
      this->clipped_pulse_duration = constrain(this->clipped_pulse_duration, RC_PWM_MIN, RC_PWM_MAX);
    }
};

RCChannelReader rc_channel1_reader = RCChannelReader(CHANNEL_ONE - 1, CHANNEL_ONE, 2);
RCChannelReader rc_channel2_reader = RCChannelReader(CHANNEL_TWO - 1, CHANNEL_TWO, 3);
RCChannelReader* rc_channel_readers[NUM_INTERRUPT_PINS] = { &rc_channel1_reader, &rc_channel2_reader };
