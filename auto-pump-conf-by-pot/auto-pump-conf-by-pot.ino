#include <avr/wdt.h>
#include <avr/sleep.h>

#define INVERSE_POT

/*
  Как устроен таймер: используем такты процессора (деленные на множитель) вместо секунд.

  Почему не подходят секунды:
  - их просто неоткуда взять (никакого millis в режиме глубокого энергосбережения — нет)
  - интервал срабатывания watchdog в "1 секунду" имеет заметную погрешность (30%)

  Зачем делить на множитель: чтобы поместиться в 32- или 16-битные числа для таймеров.
  64-битными числами на 8-битном микроконтроллере очень "дорого" оперировать
  (требуется много инструкций на одно действие, а памяти мало).
*/

// 1024 — 16 дней, 4_800_000 * 60 * 60 * 24 * 16 тактов
// 1024 — (3240000000 тактов)<< 11
// 1 бит - (3164062.5 тактов) << 11    0x30479Eu
// 1024 ~ (0xc11e тактов) << 27; погрешность 14 секунд суммарно)

// '0x{:02X}u'.format( 4_800_000 * 60 * 5 >> (11 + 10) )
#define STEP_FOR_TOTAL_5MIN 0x2AEu
// '0x{:02X}u'.format( 4_800_000 * 60 * 8 >> (11 + 10) )
#define STEP_FOR_TOTAL_8MIN 0x44Au
// '0x{:02X}u'.format( 4_800_000 * 60 * 60 * 24 * 16 >> (11 + 10) )
#define STEP_FOR_TOTAL_16DAYS 0x30479Eu
#define INTERVAL_STEP STEP_FOR_TOTAL_16DAYS


//-- WDT_vect
// признак срабатывания watchdog
volatile bool watchdog_flag = false;
volatile uint32_t main_timer = 0;
volatile uint16_t pump_timer = 0;
// число тактов процессора (деленное на 2048), проходящее за одно срабатывание watchdog
volatile uint16_t watchdog_ticks = 0;

// INT0_vect
volatile bool button_pressed = true;

// TIM0_OVF_vect
volatile uint8_t timer_overflow;

uint16_t interval, duration;
uint32_t interval_time = 0xFFFFFFFF;
uint8_t count = 0;

void disablePins() {
  DDRB = 0;
  PORTB = 0;
}

void turnOnInterval() {
  PORTB = 0b00000010;
  DDRB = 0b00000110;
}

void turnOnDuration() {
  PORTB = 0b00000100;
  DDRB = 0b00000110;
}

void blinkInterval() {
  turnOnInterval();
  sleepOnTimer(88); // ~300ms
  disablePins();
  sleepOnTimer(88); // ~300ms
}

void blinkDuration() {
  turnOnDuration();
  sleepOnTimer(97); // ~331 ms
  disablePins();
  sleepOnTimer(97); // ~331 ms
}

void setup() {
  ADCSRA = _BV(ADPS2) | _BV(ADPS0);

  // Enable overflow interrupt on Timer0
  // used in sleepOnTimer and measureWatchDogTick
  TIMSK0 = _BV(TOIE0);

  wdt_enable(WDTO_1S);  // разрешаем ватчдог
  bitSet(WDTCR, WDTIE); // разрешаем прерывания по ватчдогу. Иначе будет резет.
  sei();                // разрешаем прерывания

  measureWatchDogTick();

  // включаем прерывания по низкому уровню PB1 (кнопка)
  bitSet(GIMSK, INT0);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // максимальный сон
}

void loop() {
  if (bit_is_set(PORTB, PB0)) {
    // полив активен
    // 1024 - 16 секунд - 37500 степов;    (4_800_000 * 16 ) >>11
    // 2 — примерно 73. 73 * 512 = 37376
    // можно использовать и меньшее значение, т.к. остановка полива идет на следующий такт watchdog
    // где целые секунды уже получились
    if (pump_timer >= (duration >> 1) * 73u) {
      // выключаем всё
      disablePins();
      main_timer = 0;
      count++;
    }
  } else {
    // полив не активен

    if (button_pressed) {
      // кнопку нажимали
      readPotentiometers();

      uint8_t blinks = (interval + 1) >> 5;
      while (blinks--) {
        blinkInterval();
      }

      blinks = (duration + 1) >> 6;
      while (blinks--) {
        blinkDuration();
      }

      sleepOnTimer(58); // ~198ms

      showCount();

      button_pressed = false;
      // прерывание включается обратно из watchdog
    }

    if (main_timer > interval_time) {
      // пора включать полив

      // выключаем прерывание (ему мешает светодиод длительности)
      bitClear(GIMSK, INT0);

      // заодно включаем светодиод длительности
      PORTB = 0b00000101;
      DDRB = 0b00000111;

      // начнем отсчитывать с нуля
      pump_timer = 0;
    }
  }

  sleep_enable();  // разрешаем сон
  sleep_cpu();     // спать!
}

void measureWatchDogTick() {
  // enter synchronization mode
  GTCCR = _BV(TSM) | _BV(PSR10);

  // Set a suited prescaler based on F_CPU
  TCCR0B = _BV(CS02); // F_CPU/256
  // Set timer0 couter to zero
  TCNT0 = 0;
  timer_overflow = 0;

  // ждем "первое" срабатывание watchdog
  while (!watchdog_flag) {
  }
  // запускаем таймер
  bitClear(GTCCR, TSM);
  watchdog_flag = false;

  // ждем "второе" срабатывание watchdog
  while (!watchdog_flag) {
  }
  // выключаем таймер
  TCCR0B = 0;

  // считываем, сколько 256*тактов CPU натикало и сдвигаем на 3 (чтобы попасть в 32 бита для 16ти дней)
  watchdog_ticks = ((timer_overflow << 8) + TCNT0) >> 3;
}

void readPotentiometers() {
  // переключаем PB1 (MISO) в output low
  PORTB = 0; // хотя рассчитываем, что оно уже
  DDRB = 0b00000010;
  // даем устаканиться
  sleepOnTimer(23); // ~78 ms

  ADMUX = 0b00000010;  // ADC2 (PB4)
  ADCSRA |= _BV(ADEN) | _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)); // Wait for conversion

#ifdef INVERSE_POT
  interval = 0x3ff - ADCW;
#else
  interval = ADCW;
#endif
  interval_time = INTERVAL_STEP * (uint32_t) interval;

  ADMUX = 0b00000011;  // ADC3 (PB3)
  bitSet(ADCSRA, ADSC);
  while (bit_is_set(ADCSRA, ADSC)); // Wait for conversion

#ifdef INVERSE_POT
  duration = 0x3ff - ADCW;
#else
  duration = ADCW;
#endif

  // выключаем подачу напряжения на потенциометры
  disablePins();
  // выключаем ADC
  bitClear(ADCSRA, ADEN);
}

void showCount() {
  uint8_t blinks = count;
  while (blinks--) {
    turnOnInterval();
    sleepOnTimer(29); // ~99ms
    turnOnDuration();
    sleepOnTimer(88); // ~300ms
  }
  disablePins();
}

ISR(WDT_vect) {
  bitSet(WDTCR, WDTIE); // разрешаем прерывания по ватчдогу. Иначе будет резет.

  if (watchdog_flag) {
    // только если флаг никто не сбросил (т.е. в основном цикле)
    main_timer += watchdog_ticks;
    pump_timer += watchdog_ticks;

    if (bit_is_clear(PORTB, PB0) && !button_pressed && digitalRead(PB1)) {
      // не во время полива: если нажатие обработано и кнопка не нажата (высокий уровень)
      // включаем прерывание обратно
      bitSet(GIMSK, INT0);
    }
  }

  watchdog_flag = true;
}

ISR(INT0_vect) {
  // отключаем прерывание, чтобы оно не продолжало срабатывать по нажатой кнопке
  bitClear(GIMSK, INT0);
  button_pressed = true;
}

ISR(TIM0_OVF_vect) {
  timer_overflow++; // Increment counter by one
}


void sleepOnTimer(uint8_t value) {
  TCNT0 = 0;
  timer_overflow = 0xFF - value;

  TCCR0B = _BV(CS01) | _BV(CS00); // F_CPU/64

  while(timer_overflow) {
  }

  // выключаем таймер
  TCCR0B = 0;
}