#include <stddef.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "crc8.h"

#define EEPROM_DATA_LENGTH 130
// CN03XYY8CH20003GC44KA01

//'D', 'E', 'L', 'L', '0', '0', 'A', 'C', '1', '8', '0', '1', '9', '5', '0', '9',
//'2', 'C', 'N', '0', 'X', 'Y', 'Y', '8', 'C', 'H', '2', '0', '0', '0', '3', 'G',
//'C', '4', '4', 'K', 'A', '0', '1', '5', '=', 0x94, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,

static const uint8_t ow_address[8] PROGMEM = { 0x09, 0x52, 0x8D, 0xED, 0x65, 0x00, 0x00, 0xEF };

static const uint8_t default_eeprom_data[EEPROM_DATA_LENGTH] PROGMEM = {
    0x44, 0x45, 0x4c, 0x4c, 0x30, 0x30, 0x41, 0x43, 0x31, 0x38, 0x30, 0x31, 0x39, 0x35, 0x30, 0x39,
    0x32, 0x43, 0x4e, 0x30, 0x33, 0x58, 0x59, 0x59, 0x38, 0x43, 0x48, 0x32, 0x30, 0x30, 0x30, 0x33,
    0x47, 0x43, 0x34, 0x34, 0x4b, 0x41, 0x30, 0x31, 0xd2, 0x6e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xff, 0x17
};

enum {
	OW_STATE_IDLE,
	OW_STATE_RESET,
	OW_STATE_PRESENCE,
	OW_STATE_RX,
	OW_STATE_TX
};

enum {
	OW_DATA_SOURCE_RAM,
	OW_DATA_SOURCE_ROM,
	OW_DATA_SOURCE_EEPROM
};

static struct {
	uint8_t state;
	uint8_t bit_state;
	uint8_t current_byte;
	uint8_t current_bit;
	uint8_t current_value;
	union {
		const uint8_t *tx_buffer;
		uint8_t *rx_buffer;
	};
	uint8_t buffer_size;
	uint8_t buffer_source;
	void (*callback)(void);
	uint8_t command;
	uint8_t selected;
	uint8_t pull_low_next;
	uint8_t arg_buffer[8];
} ow = {
	.state = OW_STATE_IDLE,
	.bit_state = 1,
    .pull_low_next = 0
};


#if 0
#define OW_RELEASE() do { DDRC &= ~_BV(PC3); PORTC |= _BV(PC3);  PORTB |= _BV(PB5);} while (0)
#define OW_PULL_LOW() do { PORTC &= ~_BV(PC3); DDRC |= _BV(PC3); PORTB &= ~_BV(PB5);} while (0)
#else
#define OW_RELEASE() do { DDRD &= ~_BV(PD2); PORTD |= _BV(PD2);  PORTB |= _BV(PB5);} while (0)
#define OW_PULL_LOW() do { PORTD &= ~_BV(PD2); DDRD |= _BV(PD2); PORTB &= ~_BV(PB5);} while (0)
#endif

#define RESET_LEN 55
#define PRESENT_LEN 15

static inline void ow_start_timer(void) {
	TCNT0 = 256-RESET_LEN;
#if F_CPU == 1000000
	TCCR0B = _BV(CS00) ;
#elif (F_CPU >= 8000000) && (F_CPU <= 9000000)
    TCCR0B =  _BV(CS01) | _BV(CS00);
#elif (F_CPU >= 16000000)
    TCCR0B = _BV(CS02) ;
#else
#error F_CPU should be 1 MHz or 8..9 MHz
#endif
}

static inline void ow_rx(uint8_t *buffer, uint8_t count, void (*callback)(void)) {
	ow.state = OW_STATE_RX;
	ow.rx_buffer = buffer;
	ow.buffer_size = count;
	ow.callback = callback;
	ow.current_byte = 0;
	ow.current_bit = 0;
	ow.current_value = 0;
}

static inline void ow_fetch_current_byte_from_buffer() {
	switch (ow.buffer_source) {
		case OW_DATA_SOURCE_RAM:
			ow.current_value = ow.tx_buffer[ow.current_byte];
			break;
		case OW_DATA_SOURCE_ROM:
			ow.current_value = pgm_read_byte(ow.tx_buffer + ow.current_byte);
			break;
		case OW_DATA_SOURCE_EEPROM:
			ow.current_value = eeprom_read_byte(ow.tx_buffer + ow.current_byte);
			break;
	}
#if 1
    UDR0 = ow.current_value;
#endif
}

static inline void ow_tx(const uint8_t *buffer, uint8_t count, uint8_t source, void (*callback)(void)) {
	ow.state = OW_STATE_TX;
	ow.tx_buffer = buffer;
	ow.buffer_size = count;
	ow.buffer_source = source;
	ow.callback = callback;
	ow.current_byte = 0;
	ow.current_bit = 0;
	ow_fetch_current_byte_from_buffer();
	ow.pull_low_next = !(ow.current_value & 1);
}

static void ow_read_real_mem(void) {
	uint16_t offset = (((uint16_t) ow.arg_buffer[1]) << 8) | ow.arg_buffer[0];
	uint8_t max_len = EEPROM_DATA_LENGTH - offset;
    const uint8_t *base = default_eeprom_data + offset; //  (const uint8_t*)
    ow_tx(base, max_len, OW_DATA_SOURCE_ROM, NULL); //OW_DATA_SOURCE_EEPROM
}

static void ow_read_mem(void) {
	uint8_t tmp[] = { ow.command, ow.arg_buffer[0], ow.arg_buffer[1] };
	ow.arg_buffer[2] = Crc8(tmp, sizeof(tmp));
    ow_tx(ow.arg_buffer + 2, 1, OW_DATA_SOURCE_RAM, ow_read_real_mem);
}

static void ow_write_real_mem(void) {
	uint16_t offset = (((uint16_t) ow.arg_buffer[1]) << 8) | ow.arg_buffer[0];
	uint8_t data = ow.arg_buffer[2];
	uint8_t *base = (uint8_t*) offset;
	eeprom_write_byte(base, data);
}

static void ow_write_mem(void) {
	uint8_t tmp[] = { ow.command, ow.arg_buffer[0], ow.arg_buffer[1], ow.arg_buffer[2] };
	ow.arg_buffer[3] = Crc8(tmp, sizeof(tmp));
	ow_tx(ow.arg_buffer + 2, 1, OW_DATA_SOURCE_RAM, ow_write_real_mem);
}

static void ow_command_received(void) {
	switch (ow.command) {
		case 0x33: // READ ROM
			ow_tx(ow_address, 8, OW_DATA_SOURCE_ROM, NULL);
			break;
		case 0xCC: // SKIP ROM
			ow.selected = 1;
			ow_rx(&(ow.command), sizeof(ow.command), ow_command_received);
			break;
		case 0xF0: // READ MEM
			if (ow.selected) {
				ow_rx(ow.arg_buffer, 2, ow_read_mem);
			}
			break;
		case 0x0F: // WRITE MEM
			if (ow.selected) {
				ow_rx(ow.arg_buffer, 3, ow_write_mem);
			}
	}
}

static void ow_bit_change(uint8_t bit)
{
    if(bit)
    {
        PORTB |= _BV(PB4);
    }
    else
    {
        PORTB &= ~_BV(PB4);
    }
	switch (ow.state) {
		case OW_STATE_RESET:
			if (bit) {
				ow.state = OW_STATE_PRESENCE;
                _delay_us(25);
				OW_PULL_LOW();
                bit = 0;
				OCR0A = 256-(RESET_LEN-PRESENT_LEN);
			}
			break;
		case OW_STATE_RX:
            if (!bit)
            {
                _delay_us(40);
				uint8_t cur_bit = ow.current_bit;
                if (PIND & _BV(PD2)) {
					ow.current_value |= _BV(cur_bit);
				}
				cur_bit++;
				if (cur_bit == 8) {
					uint8_t cur_byte = ow.current_byte;
					ow.rx_buffer[cur_byte++] = ow.current_value;
#if 1
                    UDR0 = ow.current_value;
#endif
					ow.current_value = 0;
					cur_bit = 0;
					if (cur_byte == ow.buffer_size) {
						ow.state = OW_STATE_IDLE;
					}
					ow.current_byte = cur_byte;
				}
				ow.current_bit = cur_bit;
				if (ow.state == OW_STATE_IDLE) {
					if (ow.callback) {
						ow.callback();
					}
				}
			}
			break;
		case OW_STATE_TX:
			if (!bit) {
                _delay_us(40);
				uint8_t cur_bit = ow.current_bit;
				cur_bit++;
				if (cur_bit == 8) {
					uint8_t cur_byte = ow.current_byte;
					cur_byte++;
					cur_bit = 0;
					if (cur_byte == ow.buffer_size) {
						ow.state = OW_STATE_IDLE;
					} else {
						ow.current_byte = cur_byte;
						ow_fetch_current_byte_from_buffer();
					}
				}
				if (ow.state != OW_STATE_IDLE) {
					ow.pull_low_next = !(ow.current_value & _BV(cur_bit));
				}
				ow.current_bit = cur_bit;
				OW_RELEASE();
				if (ow.state == OW_STATE_IDLE) {
					if (ow.callback) {
						ow.callback();
					}
				}
			}
			break;
	}
    if (!bit)
    {
		ow_start_timer();
    }
#if 0
    else
    {
        _delay_us(1);
	}
#endif


}

//ISR(PCINT1_vect)
ISR(INT0_vect)
{
	uint8_t bit = ow.bit_state;
	do {
        PORTB &= ~_BV(PB3);
#if 0
		PCIFR = _BV(PCIF1);
#else
        EIFR = _BV(INTF0);
#endif
#if 1
        bit = (bit==0?1:0);
		if (!bit && ow.pull_low_next) {
			OW_PULL_LOW();
			ow.pull_low_next = 0;
		}
#endif
		ow_bit_change(bit);
        PORTB |= _BV(PB3);
    }
    while
#if 0
            (PCIFR & _BV(PCIF1));
#else
            (EIFR & _BV(INTF0));
#endif
    ow.bit_state = (PIND & _BV(PD2)) ? 1 : 0;
}

ISR(TIMER0_OVF_vect)
{
//    PORTB |= _BV(PB4);
	TCCR0B = 0;    
    TIFR0 = _BV(OCF0A);
	if (!ow.bit_state) {
		ow.state = OW_STATE_RESET;
		ow.selected = 0;
	}
}

ISR(TIMER0_COMPA_vect)
{
    switch (ow.state) {
    case OW_STATE_PRESENCE:
        TCNT0 = 256-RESET_LEN;
        OCR0A = 0;
        OW_RELEASE();
        ow_rx(&ow.command, sizeof(ow.command), ow_command_received);
        break;
    }
}

int main(void) {
    CLKPR = 0x80 ; //_BV(CLKPCE);
    CLKPR = 0;

    // Disable analog comparator
    ACSR |= _BV(ACD);
    // Disable ADC, TIM1 and USI
    PRR = _BV(PRADC) /*| _BV(PRUSART0)*/ |  _BV(PRSPI) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRTWI);

#if 0
	
	// Check EEPROM and load default values if needed
    // OW_PULL_LOW();
    if (eeprom_read_byte((const uint8_t*) 0) == 0xFF) {
		for (uint8_t i = 0; i < EEPROM_DATA_LENGTH; i++) {
            eeprom_write_byte(((uint8_t*) 1) + i - 1, pgm_read_byte(default_eeprom_data + i));
		}
	} else {
		_delay_us(10);
    }
#endif

	OW_RELEASE();
	// TIM0: overflow and compare A interrupts
	TIMSK0 |= _BV(TOIE0) | _BV(OCIE0A);
	OCR0A = 0;
    
	// PCINT11: Any change
#if 0
	PCICR = _BV(PCIE1);    
	PCMSK1 |= _BV(PCINT11);
	// Read current line state
	PCIFR = _BV(PCIF1);
#else
    // INT0: Any change
    EICRA = (EICRA | _BV(ISC00)) & ~_BV(ISC01);
    EIMSK |= _BV(INT0);
    // Read current line state
    EIFR = _BV(INTF0);
#endif
#if 0
    ow.bit_state = (PINC & _BV(PC3)) ? 1 : 0;
#else
    ow.bit_state = (PIND & _BV(PD2)) ? 1 : 0;
#endif
    DDRB |= _BV(PB5) | _BV(PB4) | _BV(PB3);
    PORTB |= _BV(PB5);
    PORTB |= _BV(PB4);
    PORTB |= _BV(PB3);

#if 1
    UBRR0 = F_CPU/(16*115200)-1;
    UCSR0B = _BV(TXEN0);
#endif

	// Enable interrupts
    sei();
	// Main loop
//    set_sleep_mode(SLEEP_MODE_IDLE);
//    sleep_enable();
    while (1) {
//        sleep_cpu();
        asm volatile ("nop");

	}
}

