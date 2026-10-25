#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define LCD_ADDR 0x27
#define RS 0x01
#define EN 0x04
#define BL 0x08
#define MAX_CM 65
#define NO_ECHO 999
#define BTN_DOWN PD4
#define BTN_UP PD7

typedef struct {
    uint16_t *values;
    uint16_t *sorted;
    uint8_t size;
    uint8_t count;
    uint8_t index;
} Filter;

volatile uint8_t measure = 0;
volatile uint8_t done = 0;
volatile uint16_t pulse = 0;

uint8_t seg_addr = 0;
const uint8_t digits[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

void i2c_wait() {
    while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_start(uint8_t addr) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    i2c_wait();
    TWDR = addr << 1;
    TWCR = (1 << TWINT) | (1 << TWEN);
    i2c_wait();
    return (TWSR & 0xF8) == 0x18;
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
    i2c_start(LCD_ADDR);
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

uint8_t seg_find() {
    for (uint8_t addr = 0x20; addr < 0x3F; addr++) {
        if (addr == LCD_ADDR) {
            addr = 0x38;
        }
        uint8_t found = i2c_start(addr);
        i2c_stop();
        if (found) {
            return addr;
        }
    }
    return 0;
}

void seg_show(uint8_t number) {
    if (seg_addr == 0) {
        return;
    }
    i2c_start(seg_addr);
    i2c_write(~digits[number % 10]);
    i2c_stop();
}

void filter_init(Filter *f, uint8_t size) {
    free(f->values);
    free(f->sorted);
    f->values = (uint16_t *)malloc(size * sizeof(uint16_t));
    f->sorted = (uint16_t *)malloc(size * sizeof(uint16_t));
    f->size = (f->values != NULL && f->sorted != NULL) ? size : 0;
    f->count = 0;
    f->index = 0;
}

void filter_add(Filter *f, uint16_t value) {
    if (f->size == 0) {
        return;
    }
    f->values[f->index] = value;
    f->index = (f->index + 1) % f->size;
    if (f->count < f->size) {
        f->count++;
    }
}

void sort(uint16_t *start, uint16_t *end) {
    for (uint16_t *p = start + 1; p < end; p++) {
        uint16_t value = *p;
        uint16_t *q = p;
        while (q > start && *(q - 1) > value) {
            *q = *(q - 1);
            q--;
        }
        *q = value;
    }
}

uint16_t filter_median(Filter *f) {
    if (f->count == 0) {
        return NO_ECHO;
    }
    uint16_t *src = f->values;
    uint16_t *dst = f->sorted;
    for (uint8_t i = 0; i < f->count; i++) {
        *dst++ = *src++;
    }
    sort(f->sorted, f->sorted + f->count);
    return f->sorted[f->count / 2];
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

void buzzer_init() {
    DDRD |= (1 << PD3);
    TCCR2A = (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS21);
}

void buzzer_set(uint16_t freq, uint16_t volume) {
    uint8_t top = 0;
    uint8_t duty = 0;

    if (freq > 0) {
        top = 62500UL / freq - 1;
        duty = (uint32_t)(top + 1) * volume / 2048;
    }

    if (duty == 0) {
        TCCR2A &= ~(1 << COM2B1);
        PORTD &= ~(1 << PD3);
    } else {
        OCR2A = top;
        OCR2B = duty;
        TCCR2A |= (1 << COM2B1);
    }
}

void adc_init() {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void buttons_init() {
    DDRD &= ~((1 << BTN_DOWN) | (1 << BTN_UP));
    PORTD |= (1 << BTN_DOWN) | (1 << BTN_UP);
}

int main() {
    char line[17];
    uint16_t last = 0xFFFF;
    Filter filter = {NULL, NULL, 0, 0, 0};
    uint8_t size = 5;

    timer0_init();
    timer1_init();
    buzzer_init();
    adc_init();
    buttons_init();
    lcd_init();

    filter_init(&filter, size);
    seg_addr = seg_find();
    seg_show(size);

    uint8_t old_buttons = PIND;
    sei();

    while (1) {
        if (measure) {
            measure = 0;

            uint16_t cm = NO_ECHO;
            if (done) {
                cm = (uint32_t)pulse * 2 / 29;
            }
            start_measure();

            if (cm >= MAX_CM) {
                cm = NO_ECHO;
            }
            filter_add(&filter, cm);
            cm = filter_median(&filter);

            uint8_t buttons = PIND;
            uint8_t pressed = old_buttons & ~buttons;
            old_buttons = buttons;

            if ((pressed & (1 << BTN_UP)) && size < 9) {
                size += 2;
                filter_init(&filter, size);
                seg_show(size);
            }
            if ((pressed & (1 << BTN_DOWN)) && size > 1) {
                size -= 2;
                filter_init(&filter, size);
                seg_show(size);
            }

            uint16_t freq = 0;
            if (cm < MAX_CM) {
                freq = 1400 - cm * 18;
            }
            buzzer_set(freq, adc_read());

            if (cm != last) {
                last = cm;
                if (cm < MAX_CM) {
                    sprintf(line, "Distance: %3u cm", cm);
                    lcd_print(0, line);
                    sprintf(line, "Tone:  %4u Hz  ", freq);
                    lcd_print(1, line);
                } else {
                    lcd_print(0, "Distance: --- cm");
                    lcd_print(1, "Tone:   --- Hz  ");
                }
            }
        }
    }
}