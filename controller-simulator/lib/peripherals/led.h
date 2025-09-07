#ifndef LED_H
#define LED_H

namespace peripherals {
class Led {

private:
  unsigned int pin;

  unsigned int getPin() const;

public:
  Led(unsigned int pin);

  ~Led();

  void Off();

  void On();
};

} // namespace peripherals

#endif // LED_H