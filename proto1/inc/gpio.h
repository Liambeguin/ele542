#ifndef GPIO_H_
#define GPIO_H_

typedef enum {
	GPIO_PORT_A,
	GPIO_PORT_B,
	GPIO_PORT_C,
	GPIO_PORT_D,
} gpio_port_t;

enum {
	GPIO_PIN_0 = 0x01,
	GPIO_PIN_1 = 0x02,
	GPIO_PIN_2 = 0x04,
	GPIO_PIN_3 = 0x08,
	GPIO_PIN_4 = 0x10,
	GPIO_PIN_5 = 0x20,
	GPIO_PIN_6 = 0x40,
	GPIO_PIN_7 = 0x80,
};

enum {
	GPIO_INPUT = 0x00,
	GPIO_OUTPUT = 0xff,
};


uint8_t gpio_set_input(gpio_port_t port, uint8_t gpio_mask);
uint8_t gpio_set_output(gpio_port_t port, uint8_t gpio_mask);
void gpio_pin_set(gpio_port_t port, uint8_t gpio_mask);
void gpio_pin_clear(gpio_port_t port, uint8_t gpio_mask);
void gpio_pin_toggle(gpio_port_t port, uint8_t gpio_mask);
uint8_t gpio_port_read(gpio_port_t port);
uint8_t gpio_pin_read(gpio_port_t port, uint8_t pin);


#endif /* GPIO_H_ */
