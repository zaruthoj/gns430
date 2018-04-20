#include <Wire.h>
#include <SparkFunSX1509.h>

#define EXPANDER_COUNT 1

#define MAX_INPUT_PINS 2
#define MAX_INPUTS 16


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

  void initialize(SX1509 *io) {
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

  //virtual void repeat(int pin, int val) {}

  virtual byte edge_trigger_mode() {return FALLING;}

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

#if 0
class HoldButton : public Input {
 public:
  HoldButton(int pin, int hold_time_ms, const char *edge_command, const char *hold_command) :
      Input(pin, -1),
      hold_time_ms_(hold_time_ms),
      low_edge_time_ms_(-1),
      edge_command_(edge_command),
      hold_command_(hold_command) {     
  }

  virtual void edge(int pin, int val) {
    if (val == HIGH) {
      low_edge_time_ms_ = -1;
      Serial.println(edge_command_);
    } else {
      low_edge_time_ms_ = millis();
    }
  }

  virtual void repeat(int pin, int val) {
    if (val == LOW && low_edge_time_ms_ > 0 && millis() - low_edge_time_ms_ >= hold_time_ms_) {
      low_edge_time_ms_ = -1;
      Serial.println(hold_command_);
    }
  }

 private:
  int hold_time_ms_;
  int low_edge_time_ms_;
  const char *edge_command_;
  const char *hold_command_;
};
#endif

class Encoder : public Input {
 public:
  Encoder(int pin_a, int pin_b, const char *left_command, const char *right_command) :
      Input(pin_a, pin_b),
      left_command_(left_command),
      right_command_(right_command) {}

  virtual void edge(unsigned int changed_pins) {
    unsigned int pin_states = io_->digitalReadPins();
    bool a_val = pin_states & pin_0_mask_;
    bool b_val = pin_states & pin_1_mask_;

    if (a_val == b_val) return;
    

    if (changed_pins & pin_0_mask_) {
      Serial.println(left_command_);
    } else {
      Serial.println(right_command_);
    }
  }

  virtual byte edge_trigger_mode() { return CHANGE; }
 private:
  const char *left_command_;
  const char *right_command_;
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
    io_.debounceTime(1);
    
    
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

  void set_interrupt() {
    interrupt_pending_ = true;
  }

 private:
  const byte i2c_address_;
  int interrupt_pin_;
  void (*isr_)();
  SX1509 io_;
  Input* inputs_[MAX_INPUTS];
  Input* pin_to_input_[MAX_INPUTS];
  int num_inputs_;
  bool interrupt_pending_;
};


Expander* expanders[EXPANDER_COUNT];

void isr_0() {
  expanders[0]->set_interrupt();
}

void isr_1() {
  expanders[1]->set_interrupt();
}

void setup() 
{
  Serial.begin(9600);

  Input* inputs_0[] = {
     new Button(0, "ENT"),
     new Button(1, "MENU"),
     new Button(2, "RNG_UP"),
     new Button(3, "RNG_DN"),
     new Button(4, "DIRECT"),
     //new HoldButton(5, 1000, "CLR", "CLR_HOLD"),
     new Button(5, "CLR"),
     new Button(6, "COM_TRANS"),
     new Button(7, "COM_TRANS"),
     new Button(8, "CDI"),
     new Button(9, "OBS"),
     new Button(10, "MSG"),
     new Button(11, "FPL"),
     new Button(12, "PROC"),
     new Encoder(14, 15, "FREQ_LARGE_DECR", "FREQ_LARGE_INCR"),
  };
  expanders[0] = new Expander(0x3E, 2, isr_0, inputs_0, 14);
#if 0
  Input* inputs_1[] = {
     new Encoder(0, 1, "FREQ_LARGE_DECR", "FREQ_LARGE_INCR"),
     new Encoder(2, 3, "FREQ_SMALL_DECR", "FREQ_SMALL_INCR"),
     new Button(4, "FREQ_PUSH"),
     new Encoder(5, 6, "FMS_LARGE_DECR", "FMS_LARGE_INCR"),
     new Encoder(7, 8, "FMS_SMALL_DECR", "FMS_SMALL_INCR"),
     new Button(9, "FMS_PUSH"),
  };
  expanders[1] = new Expander(0x3F, 3, isr_1, inputs_1, 6);
#endif
}

void loop() 
{
  for(int i = 0; i < EXPANDER_COUNT; ++i) {
    expanders[i]->scan();
  }
}
