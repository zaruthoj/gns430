#include <movingAvg.h>

#include <Wire.h>
#include <SparkFunSX1509.h>

#define EXPANDER_COUNT 2

#define MAX_INPUT_PINS 2
#define MAX_INPUTS 16
#define SERIAL_BUF_SIZE 80

class SerialControl {
 public:
  virtual void send_control(uint8_t data=1) = 0;
};

class Control : public SerialControl {
 public:
  Control(const char *offset) : offset_(offset) {}

  void send_control(uint8_t data=1) {
    Serial.print("C|");
    Serial.print(offset_);
    Serial.print("|N|");
    Serial.println(data);
  }
 private:
  const char *offset_;
};

class Keypress : public SerialControl {
 public:
  Keypress(const char* key, const char* keyshift) : key_(key), keyshift_(keyshift) {}

  void send_control(uint8_t data=1) {
    Serial.print("K|");
    Serial.print(key_);
    Serial.print("|N|");
    Serial.println(keyshift_);
  }
 private:
  const char* key_;
  const char* keyshift_;
};

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


class Encoder : public Input {
 public:
  Encoder(int pin_a, int pin_b, SerialControl *left_command, SerialControl *right_command) :
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
      left_command_->send_control();
    } else {
      right_command_->send_control();
    }
  }

  virtual byte edge_trigger_mode() { return CHANGE; }

  void scan() {
    a_val_ = io_->digitalRead(pins_[0]);
    b_val_ = io_->digitalRead(pins_[1]);
  }
 private:
  SerialControl *left_command_;
  SerialControl *right_command_;
  bool a_val_;
  bool b_val_;
};

Encoder* freq_small;
Encoder* freq_large;
Encoder* fms_small;
Encoder* fms_large;

void scan_encoders() {
  freq_small->scan();
  freq_large->scan();
  fms_small->scan();
  fms_large->scan();
}

class Button : public Input {
 public:
  Button(int pin, SerialControl *command) :
      Input(pin, -1),
      command_(command) {     
  }

  virtual void edge(unsigned int changed_pins) {
    command_->send_control();
    scan_encoders();
  }
 private:
  SerialControl *command_;
};

class Toggle : public Input {
 public:
  Toggle(int pin, SerialControl *command, int active) :
      Input(pin, -1),
      command_(command),
      active_(active) {     
  }

  virtual void edge(unsigned int changed_pins) {
    command_->send_control(io_->digitalRead(pins_[0]) == active_);
    scan_encoders();
  }

  virtual byte edge_trigger_mode() { return CHANGE; }
 private:
  SerialControl *command_;
  const int active_;
};

class HoldButton : public Input {
 public:
  HoldButton(int pin, int down_command, int up_command) :
      Input(pin, -1),
      down_command_(down_command),
      up_command_(up_command) {     
  }

  virtual void edge(unsigned int changed_pins) {
    if (io_->digitalRead(pins_[0]) == LOW) {
      down_command_->send_control();
    } else {
      up_command_->send_control();
    }
    scan_encoders();
  }

  virtual byte edge_trigger_mode() { return CHANGE; }

 private:
  SerialControl *up_command_;
  SerialControl *down_command_;
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
    io_.debounceTime(2);

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

Potentiometer* com_volume;
Potentiometer* nav_volume;
Toggle* id_toggle;
Toggle* power_toggle;

void setup() 
{
  ADCSRA = (ADCSRA & B11111000) | 4;
  Serial.begin(115200);

  freq_small = new Encoder(0, 1, new Control("66632"), new Control("66633"));//"FSI", "FSD");
  freq_large = new Encoder(2, 3, new Control("66631"), new Control("66630"));//"FLD", "FLI");
  fms_small = new Encoder(4, 5, new Control("66627"), new Control("66628"));//"GSI", "GSD");
  fms_large = new Encoder(6, 7, new Control("66626"), new Control("66625"));//"GLD", "GLI");
  id_toggle = new Toggle(4, new Control("66614"), HIGH);
  power_toggle = new Toggle(12, new Control("66602"), LOW);
  Input* inputs_0[] = {
     freq_small,
     freq_large,
     id_toggle,
     new Button(5, new Control("66606")), // "MSG"),
     new Button(6, new Control("66609")), // "FPL"),
     new Button(7, new Control("66612")), // "PROC"),
     new Button(10, new Control("66605")), // "OBS"),
     new Button(11, new Control("66604")), // "CDI"),
     power_toggle,
     new Button(13, new Keypress("67", "43")), // "COM_TRANS"),
     new Button(14, new Keypress("86", "43")), // "NAV_TRANS"),
     new Button(15, new Control("66629")), // "FREQ_TOGGLE"),
  };
  expanders[0] = new Expander(0x3E, 2, isr_0, inputs_0, 12);

  Input* inputs_1[] = {
     new Button(0, new Control("66624")), // "CURSOR"),
     fms_small,
     fms_large,
     new Button(10, new Control("66615")), // "RNG_DN"),
     new Button(11, new Control("66616")), // "RNG_UP"),
     new Button(12, new Control("66618")), // "MENU"),
     new Button(13, new Control("66623")), // "ENT"),
     new Button(14, new Control("66617")), // "DIRECT"),
     new HoldButton(15, new Control("66621"), new Control("66622")), // "CLR_DN", "CLR_UP"),
  };
  expanders[1] = new Expander(0x3F, 3, isr_1, inputs_1, 9);

  //com_volume = new Potentiometer(1, 2, "COM_VOL:");
  //nav_volume = new Potentiometer(0, 2, "NAV_VOL:");
}

uint32_t last_slow_scan = 0;
uint32_t last_heartbeat = 0;
uint32_t last_watchdog = 0;
void loop() 
{
  expanders[0]->scan();
  expanders[1]->scan();

  uint32_t now = millis();
  if (now > last_watchdog + 1000) {
    last_watchdog = now;
    Serial.println("P3DGNS");  
  } 
  #if 0
  int milliseconds = millis();
  if (milliseconds - last_slow_scan > 50) {
    last_slow_scan = milliseconds;
    bool heartbeat = milliseconds - last_heartbeat > 30000;
    //com_volume->scan(heartbeat);
    //nav_volume->scan(heartbeat);
    if (heartbeat) {
      id_toggle->edge(0);
      power_toggle->edge(0);
      last_heartbeat = milliseconds;
    }
    
    freq_small->scan();
    freq_large->scan();
    fms_small->scan();
    fms_large->scan();
  }
  #endif
}
