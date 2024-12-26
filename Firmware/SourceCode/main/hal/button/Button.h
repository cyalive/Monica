/*
	Button - a small library for Arduino to handle button debouncing
	
	MIT licensed.
*/

#ifndef Button_h
#define Button_h
#include <driver/gpio.h>


class Button
{
	public:
		Button(gpio_num_t pin, uint16_t debounce_ms = 100, uint32_t long_press_threshold = 1000);
		void begin();
		bool read();
		bool toggled();
		bool pressed();
		bool released();
		bool has_changed();
		bool isLongPressed();
		
		const static bool PRESSED = 0;
		const static bool RELEASED = 1;
	
	private:
		gpio_num_t  _pin;
		uint16_t _delay;
		bool     _state;
		uint32_t _ignore_until;
		bool     _has_changed;
		uint32_t _press_start_time;
		bool     _is_long_press;
		uint32_t _long_press_threshold;
};

#endif
