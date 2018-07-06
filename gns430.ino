#include <movingAvg.h>

#include <Wire.h>
#include <SparkFunSX1509.h>

#define EXPANDER_COUNT 2

#define MAX_INPUT_PINS 2
#define MAX_INPUTS 16
#define SERIAL_BUF_SIZE 80

class Input {
 public:
  Input(int pin_0, int pin_1) :
    pin_0_mask_(1 << pin_0),
    pin_1_mask_(pin_1 > 0 ? 1 << pin_1 : 0),
    pin_mask_(pin_0_mask_ | pin_1_mask_) {
    pins_[0] = pin_0;
    pins_[1] = pin_1;
    for (int i = 0; i < MAX_INPUT_PINS; ++i) {
      last_[i] = HIGH;
    }
  }

  virtual void initialize(SX1509 *io) {
    io_ = io;
    for (int i = 0; i < MAX_INPUT_PINS; ++i) {
      if (pins_[i] != -1) {
        io_->pinMode(pins_[i], INPUT_PULLUP);
        io_->debouncePin(pins_[i]);
        io_->enableInterrupt(pins_[i], edge_trigger_mode());
      }
    }
  }

  virtual void edge(unsigned int changed_pins) = 0;

  virtual byte edge_trigger_mode() {return RISING;}

  const unsigned int pin_0_mask_;
  const unsigned int pin_1_mask_;
  const unsigned int pin_mask_;
  int pins_[MAX_INPUT_PINS];

 protected:
  SX1509 * io_;
  int last_[MAX_INPUT_PINS];
};

class Button : public Input {
 public:
  Button(int pin, const char *command) :
      Input(pin, -1),
      command_(command) {     
  }

  virtual void edge(unsigned int changed_pins) {
    Serial.println(command_);
  }
 private:
  const char *command_;
};

class Toggle : public Input {
 public:
  Toggle(int pin, const char *command, int active) :
      Input(pin, -1),
      command_(command),
      active_(active) {     
  }

  virtual void edge(unsigned int changed_pins) {
    Serial.print(command_);
    if (io_->digitalRead(pins_[0]) == active_) {
      Serial.println("_ON");
    } else {
      Serial.println("_OFF");
    }
  }

  virtual byte edge_trigger_mode() { return CHANGE; }
 private:
  const char *command_;
  const int active_;
};

class HoldButton : public Input {
 public:
  HoldButton(int pin, int hold_time_ms, const char *edge_command, const char *hold_command) :
      Input(pin, -1),
      hold_time_ms_(hold_time_ms),
      hold_triggered_(false),
      low_edge_time_ms_(-1),
      edge_command_(edge_command),
      hold_command_(hold_command) {     
  }

  virtual void edge(unsigned int changed_pins) {
    if (io_->digitalRead(pins_[0]) == HIGH) {
      if (!hold_triggered_) {
        Serial.println(edge_command_);
      }
      low_edge_time_ms_ = -1;
      hold_triggered_ = false;
    } else {
      low_edge_time_ms_ = millis();
      hold_triggered_ = false;
    }
  }

  virtual byte edge_trigger_mode() { return CHANGE; }

  void scan(int milliseconds) {
    if ( low_edge_time_ms_ == -1 || hold_triggered_) {
      return;
    }
    if (milliseconds - low_edge_time_ms_ >= hold_time_ms_) {
      hold_triggered_ = true;
      Serial.println(hold_command_);
    }
  }

 private:
  int hold_time_ms_;
  int low_edge_time_ms_;
  bool hold_triggered_;
  const char *edge_command_;
  const char *hold_command_;
};

class Encoder : public Input {
 public:
  Encoder(int pin_a, int pin_b, const char *left_command, const char *right_command) :
      Input(pin_a, pin_b),
      left_command_(left_command),
      right_command_(right_command),
      a_val_(0),
      b_val_(0) {}

  virtual void initialize(SX1509 *io) {
    Input::initialize(io);
    scan();
  }

  virtual void edge(unsigned int changed_pins) {
    bool a_changed = (changed_pins & pin_0_mask_) >> pins_[0];
    bool b_changed = (changed_pins & pin_1_mask_) >> pins_[1];
    a_val_ = a_val_ != a_changed;
    b_val_ = b_val_ != b_changed;

    if (a_val_ == b_val_) return;

    if (changed_pins & pin_0_mask_) {
      Serial.println(left_command_);
    } else {
      Serial.println(right_command_);
    }
  }

  virtual byte edge_trigger_mode() { return CHANGE; }

  void scan() {
    a_val_ = io_->digitalRead(pins_[0]);
    b_val_ = io_->digitalRead(pins_[1]);
  }
 private:
  const char *left_command_;
  const char *right_command_;
  bool a_val_;
  bool b_val_;
};

#define PIN_CASE(pin) \
    case 1 << pin:\
      pin_to_input_[pin]->edge(changed_pins);\
      break;



class Expander {
 public:
  Expander(const byte i2c_address, int interrupt_pin, void (*isr)(), Input* inputs[], int num_inputs) :
      i2c_address_(i2c_address),
      interrupt_pin_(interrupt_pin),
      isr_(isr),
      num_inputs_(num_inputs < MAX_INPUTS ? num_inputs : MAX_INPUTS) {
    pinMode(interrupt_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin_), 
                  isr_, FALLING);
    if (!io_.begin(i2c_address_))
    {
      Serial.print("Failed to init SX1509 at ");
      Serial.print(i2c_address_);
    }
    io_.debounceTime(1);

    memset(pin_to_input_, NULL, sizeof(pin_to_input_[0]) * MAX_INPUTS);
    for (int i = 0; i < num_inputs_; ++i) {
      inputs_[i] = inputs[i];
      for (int j = 0; j < MAX_INPUT_PINS; ++j) {
        int pin = inputs_[i]->pins_[j];
        if (pin == -1) continue;
        pin_to_input_[pin] = inputs[i];
      }
      inputs_[i]->initialize(&io_);
    }
  }

  void scan() {
    if (interrupt_pending_) {
      interrupt_pending_ = false;
      unsigned int changed_pins = io_.interruptSource();
      switch(changed_pins) {
        PIN_CASE(0)
        PIN_CASE(1)
        PIN_CASE(2)
        PIN_CASE(3)
        PIN_CASE(4)
        PIN_CASE(5)
        PIN_CASE(6)
        PIN_CASE(7)
        PIN_CASE(8)
        PIN_CASE(9)
        PIN_CASE(10)
        PIN_CASE(11)
        PIN_CASE(12)
        PIN_CASE(13)
        PIN_CASE(14)
        PIN_CASE(15)
      }
    }
  }

  volatile bool interrupt_pending_;

 private:
  const byte i2c_address_;
  int interrupt_pin_;
  void (*isr_)();
  SX1509 io_;
  Input* inputs_[MAX_INPUTS];
  Input* pin_to_input_[MAX_INPUTS];
  int num_inputs_;
};


Expander* expanders[EXPANDER_COUNT];

void isr_0() {
  expanders[0]->interrupt_pending_ = true;
}

void isr_1() {
  expanders[1]->interrupt_pending_ = true;
}

class Potentiometer {
 public:
  Potentiometer(int pin, int dead_zone, char* command) :
      pin_(pin),
      dead_zone_(dead_zone),
      last_value_(-1),
      command_(command),
      avg_value_(10) {
    avg_value_.begin();
  }

  void scan(bool heartbeat) {
    int value = avg_value_.reading(analogRead(pin_));

    int difference = last_value_ - value;
    bool non_zero = abs(difference) > dead_zone_;
    if (non_zero || heartbeat) {
      last_value_ = value;
      Serial.print(command_);
      Serial.println(last_value_);
    }
  }

 private:
  int pin_;
  int dead_zone_;
  movingAvg avg_value_;
  int last_value_;
  char *command_;
};

Encoder* freq_small;
Encoder* freq_large;
Encoder* fms_small;
Encoder* fms_large;
HoldButton* clear_button;
Potentiometer* com_volume;
Potentiometer* nav_volume;
Toggle* id_toggle;
Toggle* power_toggle;

void setup() 
{
  ADCSRA = (ADCSRA & B11111000) | 4;
  Serial.begin(9600);

  freq_small = new Encoder(0, 1, "FREQ_SMALL_INCR", "FREQ_SMALL_DECR");
  freq_large = new Encoder(2, 3, "FREQ_LARGE_DECR", "FREQ_LARGE_INCR");
  fms_small = new Encoder(4, 5, "FMS_SMALL_INCR", "FMS_SMALL_DECR");
  fms_large = new Encoder(6, 7, "FMS_LARGE_DECR", "FMS_LARGE_INCR");
  id_toggle = new Toggle(4, "ID", HIGH);
  power_toggle = new Toggle(12, "POWER", LOW);
  Input* inputs_0[] = {
     freq_small,
     freq_large,
     id_toggle,
     new Button(5, "MSG"),
     new Button(6, "FPL"),
     new Button(7, "PROC"),
     new Button(10, "OBS"),
     new Button(11, "CDI"),
     power_toggle,
     new Button(13, "COM_TRANS"),
     new Button(14, "NAV_TRANS"),
     new Button(15, "FREQ_PUSH"),
  };
  expanders[0] = new Expander(0x3E, 2, isr_0, inputs_0, 12);

  clear_button = new HoldButton(15, 1000, "CLR", "CLR_HOLD");
  Input* inputs_1[] = {
     new Button(0, "FMS_PUSH"),
     fms_small,
     fms_large,
     new Button(10, "RNG_DN"),
     new Button(11, "RNG_UP"),
     new Button(12, "MENU"),
     new Button(13, "ENT"),
     new Button(14, "DIRECT"),
     clear_button,
  };
  expanders[1] = new Expander(0x3F, 3, isr_1, inputs_1, 9);

  com_volume = new Potentiometer(1, 2, "COM_VOL:");
  nav_volume = new Potentiometer(0, 2, "NAV_VOL:");
}

int last_slow_scan = 0;
int last_heartbeat = 0;
char serial_buffer[SERIAL_BUF_SIZE];
int serial_pos = 0;
void loop() 
{
  expanders[0]->scan();
  expanders[1]->scan();
  int milliseconds = millis();
  if (milliseconds - last_slow_scan > 50) {
    last_slow_scan = milliseconds;
    bool heartbeat = milliseconds - last_heartbeat > 30000;
    com_volume->scan(heartbeat);
    nav_volume->scan(heartbeat);
    if (heartbeat) {
      id_toggle->edge(0);
      power_toggle->edge(0);
      last_heartbeat = milliseconds;
    }
    
    clear_button->scan(milliseconds);
    freq_small->scan();
    freq_large->scan();
    fms_small->scan();
    fms_large->scan();
  }
  if (Serial.available() > 0) {
    if (serial_pos >= SERIAL_BUF_SIZE) {
      serial_pos = 0;
    }
    serial_buffer[serial_pos] = Serial.read();

    switch(serial_buffer[serial_pos]) {
      case '\n':
        serial_buffer[serial_pos] = '\0';
        if (strncmp(serial_buffer, "ID", 2) == 0) {
          Serial.println("GNS430:1");
        }
        break;
      default:
        break;
    }
    serial_pos++;
  }
}
