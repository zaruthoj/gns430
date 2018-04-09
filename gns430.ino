#include <Wire.h>
#include <SparkFunSX1509.h>

#define EXPANDER_COUNT 1

#define MAX_INPUT_PINS 2
#define MAX_INPUTS 16

class Input {
 public:
  Input(int pin_0, int pin_1) {
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
        io_->debounceConfig(011);
      }
    }
  }

  void scan() {
    for(int i = 0; i < MAX_INPUT_PINS && pins_[i] != -1; ++i) {
      int val = io_->digitalRead(pins_[i]); 
      if (val != last_[i]) {
        edge(i, val);
        last_[i] = val;
      }
    }
  }

  virtual void edge(int pin, int val) = 0;
 private:
  SX1509 * io_;
  int pins_[MAX_INPUT_PINS];
  int last_[MAX_INPUT_PINS];
};

class Button : public Input {
 public:
  Button(int pin, const char *command) :
      Input(pin, -1),
      command_(command) {     
  }

  virtual void edge(int pin, int val) {
    if (val == LOW) {
      Serial.println(command_);
    }
  }
 private:
  const char *command_;
};

class Expander {
 public:
  Expander(const byte i2c_address, Input* inputs[], int num_inputs) :
      i2c_address_(i2c_address),
      num_inputs_(num_inputs < MAX_INPUTS ? num_inputs : MAX_INPUTS) {
    if (!io_.begin(i2c_address_))
    {
      Serial.print("Failed to init SX1509 at ");
      Serial.print(i2c_address_);
    }

    for (int i = 0; i < num_inputs_; ++i) {
      inputs_[i] = inputs[i];
      inputs_[i]->initialize(&io_);
    }
  }

  void scan() {
    for (int i = 0; i < num_inputs_; ++i) {
      inputs_[i]->scan();
    }
  }
 private:
  const byte i2c_address_;
  SX1509 io_;
  Input* inputs_[MAX_INPUTS];
  int num_inputs_;
};

Expander* expanders[EXPANDER_COUNT];
void setup() 
{
  Serial.begin(9600);
  Input* inputs_0[] = {
     new Button(0, "ENT"),
     new Button(1, "MENU"),
     new Button(2, "RNG_UP"),
     new Button(3, "RNG_DN"),
     new Button(4, "DIRECT"),
     new Button(5, "CLR"),
  };
  expanders[0] = new Expander(0x3E, inputs_0, 6);
}

void loop() 
{
  for(int i = 0; i < EXPANDER_COUNT; ++i) {
    expanders[i]->scan();
  }
}
