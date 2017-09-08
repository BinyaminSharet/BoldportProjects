#include <Wire.h>

#define REG_IODIR       0x00
#define REG_IPOL        0x02
#define REG_GPINTEN     0x04
#define REG_DEFVAL      0x06
#define REG_INTCON      0x08
#define REG_IOCO        0x0A
#define REG_GPPU        0x0C
#define REG_INTF        0x0E
#define REG_INTCAP      0x10
#define REG_GPIO        0x12
#define REG_OLAT        0x14

#define PA_MASK     0x00
#define PB_MASK     0x01

#define IXO_PORT_A      0
#define IXO_PORT_B      1

static const unsigned char port_mask[] = { PA_MASK, PB_MASK };

class IXO {
public:
    IXO(uint8_t address, uint8_t reset_pin) {
        address_ = address;
        reset_pin_ = reset_pin;
        dir_[IXO_PORT_A] = 0b11111111;
        dir_[IXO_PORT_B] = 0b11111111;
        gpio_[IXO_PORT_A] = 0b00000000;
        gpio_[IXO_PORT_B] = 0b00000000;
    }

    void begin() {
        Wire.begin();
    }

    void reset() {
        pinMode(A0, OUTPUT);
        delay(10);
        digitalWrite(A0, LOW);
        delay(200);
        digitalWrite(A0, HIGH);
        delay(30);
    }

    void reg_write(uint8_t port, uint8_t reg, uint8_t value) {
        Wire.beginTransmission(address_);
        Wire.write(port_mask[port] | reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    void reg_read(uint8_t port, uint8_t reg, uint8_t * value) {
        Wire.beginTransmission(address_);
        Wire.write(port_mask[port] | reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)address_, (uint8_t)1);
        *value = Wire.read();
    }

    void set_dir(uint8_t port, uint8_t pins) {
        reg_write(port, REG_IODIR, pins);
        dir_[port] = pins;
    }

    void set_gpio(uint8_t port, uint8_t pins) {
        reg_write(port, REG_GPIO, pins);
    }

    void latch(uint8_t port, uint8_t pins) {
        reg_write(port, REG_OLAT, pins);
    }

    void read_gpio(uint8_t port, uint8_t * pins) {
        reg_read(port, REG_GPIO, pins);
    }

private:
    uint8_t reset_pin_;
    uint8_t address_;
    uint8_t dir_[2];
    uint8_t gpio_[2];
};

const uint8_t IXO_ADDRESS = 0x20;

const uint8_t INTB_PIN = 3;

IXO ixo(IXO_ADDRESS, 0);

void setup()
{
    Serial.begin(9600);
    ixo.begin();
    // reset IxpandO
    ixo.reset();
    // set each pin in port A as output
    ixo.reg_write(IXO_PORT_A, REG_IODIR, 0b00000000);
    // set each pin in port B as input
    ixo.reg_write(IXO_PORT_B, REG_IODIR, 0b11111111);
    // set pull-ups for each pin in port B
    ixo.reg_write(IXO_PORT_B, REG_GPPU, 0b11111111);
    // revert the polarity of each pin in port B because we use pull up,
    // so when the switch is open it has logical 1 and when the switch is
    // closed it is connected to ground and has logical 0
    ixo.reg_write(IXO_PORT_B, REG_IPOL, 0b11111111);
    // enable interrupt (over INT B) for each pin in port B
    // by default, the interrupt is triggered when a pin's value change.
    // if we want to trigger INT B when the value is not DEFVAL,
    // we need to configure DEFVAL and INTCON as well.
    ixo.reg_write(IXO_PORT_B, REG_GPINTEN, 0b11111111);
    // INT B toggles when it is triggered
    // so our ISR is triggers on both rise and fall.
    attachInterrupt(digitalPinToInterrupt(INTB_PIN), ixpando_intb_isr, CHANGE);
}

static uint8_t changed = 1;
void ixpando_intb_isr() {
    changed = 1;
}

void loop()
{  
  if (changed) {
    uint8_t pins = 0;
    Serial.println("change!");
    changed = 0;
    ixo.read_gpio(IXO_PORT_B, &pins);
    // reverse the pins ..
    pins = (pins & 0xF0) >> 4 | (pins & 0x0F) << 4;
    pins = (pins & 0xCC) >> 2 | (pins & 0x33) << 2;
    pins = (pins & 0xAA) >> 1 | (pins & 0x55) << 1;
    ixo.reg_write(IXO_PORT_A, REG_GPIO, pins);
  }
}
