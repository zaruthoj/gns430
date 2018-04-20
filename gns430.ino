#include <Wire.h>
#include <SparkFunSX1509.h>

#define EXPANDER_COUNT 2

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
        //io_->debouncePin(pins_[i]);
        //io_->debounceConfig(0);
      }
    }
  }

  void scan() {
    bool changed_pins[] = {false, false};
    int val;
    if (pins_[0] > 0) {
      val = io_->digitalRead(pins_[0]);
      changed_pins[0] = val != last_[0];
      last_[0] = val;
    }

    if (pins_[1] > 0) {
      val = io_->digitalRead(pins_[1]);
      changed_pins[1] = val != last_[1];
      last_[1] = val;
    }

    changed_pins[0] ? edge(0, last_[0]) : repeat(0, last_[0]);
    changed_pins[1] ? edge(1, last_[1]) : repeat(1, last_[1]);
  }

  virtual void edge(int pin, int val) {};

  virtual void repeat(int pin, int val) {}
 protected:
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
    if (val == HIGH) {
      Serial.println(command_);
    }
  }
 private:
  const char *command_;
};

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

class Encoder : public Input {
 public:
  Encoder(int pin_a, int pin_b, const char *left_command, const char *right_command) :
      Input(pin_a, pin_b),
      left_command_(left_command),
      right_command_(right_command) {}

  virtual void edge(int pin, int val) {
    Serial.println(left_command_);
    Serial.println(pins_[pin]);
    Serial.println(last_[0]);
    Serial.println(last_[1]);
    Serial.println();
    if (last_[0] == last_[1]) return;
    if (pin == 0) {
      //Serial.println(left_command_);
    } else {
      //Serial.println(right_command_);
    }
  }

 private:
  const char *left_command_;
  const char *right_command_;
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

SX1509 io;
class TestEncoder {
public:
  TestEncoder(int pin_a, int pin_b) :
      pin_a_(pin_a),
      pin_b_(pin_b),
      last_a_(HIGH),
      last_b_(HIGH) {
  }

  void scan() {
    int val_a = io.digitalRead(pin_a_);
    int val_b = io.digitalRead(pin_b_);
    
    
    int pin = -1;
    int val = HIGH;
    if (val_a != last_a_) {
      //Serial.print("A");
      //Serial.println(val_a);
      pin = pin_a_;
      val = val_a;
    } else if (val_b != last_b_) {
      //Serial.print("B");
      //Serial.println(val_b);  
      pin = pin_b_;
      val = val_b;
    }
    
    last_a_ = val_a;
    last_b_ = val_b;

    if (pin == -1) {
      return;
    }
    //Serial.println();
    if (val_a == val_b) return;
    if (pin == pin_a_) {
      Serial.println("left");
    } else {
      Serial.println("right");
    }
  }

protected:

  int pin_a_;
  int pin_b_;
  int last_a_;
  int last_b_;
};


TestEncoder encoder(0, 1);
void setup() 
{
  Serial.begin(9600);
#if 0
  Input* inputs_0[] = {
     new Button(0, "ENT"),
     new Button(1, "MENU"),
     new Button(2, "RNG_UP"),
     new Button(3, "RNG_DN"),
     new Button(4, "DIRECT"),
     new HoldButton(5, 1000, "CLR", "CLR_HOLD"),
     new Button(6, "COM_TRANS"),
     new Button(7, "COM_TRANS"),
     new Button(8, "CDI"),
     new Button(9, "OBS"),
     new Button(10, "MSG"),
     new Button(11, "FPL"),
     new Button(12, "PROC"),
  };
  expanders[0] = new Expander(0x3E, inputs_0, 13);

  Input* inputs_1[] = {
     new Encoder(0, 1, "FREQ_LARGE_DECR", "FREQ_LARGE_INCR"),
     new Encoder(2, 3, "FREQ_SMALL_DECR", "FREQ_SMALL_INCR"),
     new Button(4, "FREQ_PUSH"),
     new Encoder(5, 6, "FMS_LARGE_DECR", "FMS_LARGE_INCR"),
     new Encoder(7, 8, "FMS_SMALL_DECR", "FMS_SMALL_INCR"),
     new Button(9, "FMS_PUSH"),
  };
  expanders[1] = new Expander(0x3F, inputs_1, 6);
#endif
  
  io.begin(0x3F);
  io.pinMode(0, INPUT_PULLUP);
  io.pinMode(1, INPUT_PULLUP);
}

void loop() 
{
#if 0
  for(int i = 0; i < EXPANDER_COUNT; ++i) {
    expanders[i]->scan();
  }
#endif
  encoder.scan();
}
