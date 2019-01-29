//#define AM

#include <movingAvg.h>

#include <Wire.h>
#include <SparkFunSX1509.h>

#ifdef AM
#include <si_message_port.hpp>
SiMessagePort* messagePort;
#endif

#define EXPANDER_COUNT 2

#define MAX_INPUT_PINS 2
#define MAX_INPUTS 16
#define SERIAL_BUF_SIZE 80

enum Events {
  GPS_POWER_BUTTON = 1,
  GPS_CDI_BUTTON, // GPS_NEAREST_BUTTON
  GPS_OBS_BUTTON,
  GPS_MSG_BUTTON,
  GPS_MSG_BUTTON_DOWN,
  GPS_MSG_BUTTON_UP,
  GPS_FLIGHTPLAN_BUTTON,
  GPS_PROCEDURE_BUTTON,
  GPS_ZOOMIN_BUTTON,
  GPS_ZOOMOUT_BUTTON,
  GPS_DIRECTTO_BUTTON,
  GPS_MENU_BUTTON,
  GPS_CLEAR_BUTTON_DOWN,
  GPS_CLEAR_BUTTON_UP,
  GPS_ENTER_BUTTON,
  GPS_NAV_ID_TOGGLE, // GPS_ACTIVATE_BUTTON
  GPS_CURSOR_BUTTON,
  GPS_GROUP_KNOB_INC,
  GPS_GROUP_KNOB_DEC,
  GPS_PAGE_KNOB_INC,
  GPS_PAGE_KNOB_DEC,
  FREQ_TOGGLE,    // GPS_BUTTON1
  FREQ_WHOLE_INC, // GPS_BUTTON2
  FREQ_WHOLE_DEC, // GPS_BUTTON3
  FREQ_FRAC_INC,  // GPS_BUTTON4
  FREQ_FRAC_DEC,  // GPS_BUTTON5
  RADIO_VOR1_IDENT_TOGGLE,
  COM_RADIO_SWAP,
  NAV1_RADIO_SWAP,
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

inline void send_command(const char *command, uint16_t message, int8_t data) {
#ifdef AM
  messagePort->SendMessage(message, (uint8_t)data);
#else
  Serial.println(command);
#endif
}

class Encoder : public Input {
 public:
  Encoder(int pin_a, int pin_b, const char *left_command, const char *right_command, uint16_t left_message, uint16_t right_message) :
      Input(pin_a, pin_b),
      left_command_(left_command),
      right_command_(right_command),
      left_message_(left_message),
      right_message_(right_message),
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
      send_command(left_command_, left_message_, 0);
    } else {
      send_command(right_command_, right_message_, 0);
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
  uint16_t left_message_;
  uint16_t right_message_;
  
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
  Button(int pin, const char *command, uint16_t message) :
      Input(pin, -1),
      command_(command),
      message_(message) {     
  }

  virtual void edge(unsigned int changed_pins) {
    send_command(command_, message_, 0);
    scan_encoders();
  }
 private:
  const char *command_;
  uint16_t message_;
};

class Toggle : public Input {
 public:
  Toggle(int pin, const char *command, uint16_t message, int active) :
      Input(pin, -1),
      command_(command),
      message_(message),
      active_(active) {     
  }

  virtual void edge(unsigned int changed_pins) {
    if (io_->digitalRead(pins_[0]) == active_) {
      send_command(command_, message_, 1);
    } else {
      send_command(command_, message_, 0);
    }
    scan_encoders();
  }

  virtual byte edge_trigger_mode() { return CHANGE; }
 private:
  const char *command_;
  uint16_t message_;
  const int active_;
};

class HoldButton : public Input {
 public:
  HoldButton(int pin, const char* down_command, const char* up_command, uint16_t down_message, uint16_t up_message) :
      Input(pin, -1),
      down_command_(down_command),
      up_command_(up_command),
      up_message_(up_message),
      down_message_(down_message) {     
  }

  virtual void edge(unsigned int changed_pins) {
    if (io_->digitalRead(pins_[0]) == LOW) {
      send_command(down_command_, down_message_, 0);
    } else {
      send_command(up_command_, up_message_, 0);
    }
    scan_encoders();
  }

  virtual byte edge_trigger_mode() { return CHANGE; }

 private:
  const char *up_command_;
  const char *down_command_;
  uint16_t up_message_;
  uint16_t down_message_;
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

#ifdef AM
static void new_message_callback(uint16_t message_id, struct SiMessagePortPayload* payload) {
  // noop
}
#endif

Potentiometer* com_volume;
Potentiometer* nav_volume;
Toggle* id_toggle;
Toggle* power_toggle;

void setup() 
{
  ADCSRA = (ADCSRA & B11111000) | 4;
#ifdef AM
  messagePort = new SiMessagePort(SI_MESSAGE_PORT_DEVICE_ARDUINO_NANO, SI_MESSAGE_PORT_CHANNEL_C, new_message_callback);
#else
  Serial.begin(115200);
#endif

  freq_small = new Encoder(0, 1, "FSI", "FSD", FREQ_FRAC_INC, FREQ_FRAC_DEC);
  freq_large = new Encoder(2, 3, "FLD", "FLI", FREQ_WHOLE_DEC, FREQ_WHOLE_INC);
  fms_small = new Encoder(4, 5, "GSI", "GSD", GPS_PAGE_KNOB_INC, GPS_PAGE_KNOB_DEC);
  fms_large = new Encoder(6, 7, "GLD", "GLI", GPS_GROUP_KNOB_DEC, GPS_GROUP_KNOB_INC);
  id_toggle = new Toggle(4, "ID", GPS_NAV_ID_TOGGLE, HIGH);
  //power_toggle = new Toggle(12, "POWER", LOW);
  Input* inputs_0[] = {
     freq_small,
     freq_large,
     id_toggle,
     new Button(5, "MSG", GPS_MSG_BUTTON),
     new Button(6, "FPL", GPS_FLIGHTPLAN_BUTTON),
     new Button(7, "PROC", GPS_PROCEDURE_BUTTON),
     new Button(10, "OBS", GPS_OBS_BUTTON),
     new Button(11, "CDI", GPS_CDI_BUTTON),
     power_toggle,
     new Button(13, "COM_TRANS", COM_RADIO_SWAP),
     new Button(14, "NAV_TRANS", NAV1_RADIO_SWAP),
     new Button(15, "FREQ_TOGGLE", FREQ_TOGGLE),
  };
  expanders[0] = new Expander(0x3E, 2, isr_0, inputs_0, 12);

  Input* inputs_1[] = {
     new Button(0, "CURSOR", GPS_CURSOR_BUTTON),
     fms_small,
     fms_large,
     new Button(10, "RNG_DN", GPS_ZOOMIN_BUTTON),
     new Button(11, "RNG_UP", GPS_ZOOMOUT_BUTTON),
     new Button(12, "MENU", GPS_MENU_BUTTON),
     new Button(13, "ENT", GPS_ENTER_BUTTON),
     new Button(14, "DIRECT", GPS_DIRECTTO_BUTTON),
     new HoldButton(15, "CLR_DN", "CLR_UP", GPS_CLEAR_BUTTON_DOWN, GPS_CLEAR_BUTTON_UP),
  };
  expanders[1] = new Expander(0x3F, 3, isr_1, inputs_1, 9);

  //com_volume = new Potentiometer(1, 2, "COM_VOL:");
  //nav_volume = new Potentiometer(0, 2, "NAV_VOL:");
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
#ifdef AM
  messagePort->Tick();
#endif

#if 0
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
#endif
}
