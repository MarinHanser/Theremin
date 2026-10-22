#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define LCD_ADDR 0x27
#define RS 0x01
#define EN 0x04
#define BL 0x08
#define MAX_CM 65

volatile uint8_t measure = 0;
volatile uint8_t done = 0;
volatile uint16_t pulse = 0;

void i2c_wait() {
    while (!(TWCR & (1 << TWINT)));
}

void i2c_start() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    i2c_wait();
    TWDR = LCD_ADDR << 1;
    TWCR = (1 << TWINT) | (1 << TWEN);
    i2c_wait();
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    i2c_wait();
}

void i2c_stop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}

void lcd_pulse(uint8_t data) {
    i2c_start();
    i2c_write(data | BL | EN);
    i2c_write(data | BL);
    i2c_stop();
}

void lcd_write(uint8_t value, uint8_t mode) {
    lcd_pulse((value & 0xF0) | mode);
    lcd_pulse((value << 4) | mode);
    _delay_us(50);
}

void lcd_init() {
    TWSR = 0;
    TWBR = 72;
    TWCR = (1 << TWEN);
    _delay_ms(50);

    lcd_pulse(0x30);
    _delay_ms(5);
    lcd_pulse(0x30);
    _delay_us(150);
    lcd_pulse(0x30);
    _delay_us(150);
    lcd_pulse(0x20);

    lcd_write(0x28, 0);
    lcd_write(0x0C, 0);
    lcd_write(0x06, 0);
    lcd_write(0x01, 0);
    _delay_ms(2);
}

void lcd_print(uint8_t row, const char *text) {
    lcd_write(row ? 0xC0 : 0x80, 0);
    while (*text) {
        lcd_write(*text++, RS);
    }
}

void timer0_init() {
    TCCR0A = (1 << WGM01);
    OCR0A = 249;
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
    static uint8_t ms = 0;
    if (++ms >= 60) {
        ms = 0;
        measure = 1;
    }
}

void timer1_init() {
    DDRB &= ~(1 << PB0);
    DDRB |= (1 << PB1);
    TCCR1A = 0;
    TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10);
    TIMSK1 = (1 << ICIE1);
}

ISR(TIMER1_CAPT_vect) {
    static uint16_t start = 0;
    if (TCCR1B & (1 << ICES1)) {
        start = ICR1;
        TCCR1B &= ~(1 << ICES1);
    } else {
        pulse = ICR1 - start;
        done = 1;
        TCCR1B |= (1 << ICES1);
    }
    TIFR1 = (1 << ICF1);
}

void start_measure() {
    cli();
    TCCR1B |= (1 << ICES1);
    TIFR1 = (1 << ICF1);
    done = 0;
    sei();

    PORTB |= (1 << PB1);
    _delay_us(10);
    PORTB &= ~(1 << PB1);
}

int main() {
    char line[17];
    uint16_t last = 0xFFFF;

    timer0_init();
    timer1_init();
    lcd_init();
    sei();

    while (1) {
        if (measure) {
            measure = 0;

            uint16_t cm = 999;
            if (done) {
                cm = (uint32_t)pulse * 2 / 29;
            }
            start_measure();

            if (cm >= MAX_CM) {
                cm = 999;
            }
            if (cm != last) {
                last = cm;
                if (cm < MAX_CM) {
                    sprintf(line, "Distance: %3u cm", cm);
                    lcd_print(0, line);
                    sprintf(line, "Tone:  %4u Hz  ", 1400 - cm * 18);
                    lcd_print(1, line);
                } else {
                    lcd_print(0, "Distance: --- cm");
                    lcd_print(1, "Tone:   --- Hz  ");
                }
            }
        }
    }
}