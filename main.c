#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/pio.h>
#include <hardware/adc.h>
#include "pio_midi_uart_lib.h"
#include "segment.pio.h"
#include "pgm.pio.h"
#include "config.h"

static int sws_sm;
static bool mode;
static bool ab;
static bool alt;
static uint8_t bank;
static bool has_pgm;

static void *midi_port;
static mutex_t mutex;

static void program_change(const uint_fast8_t n) {
  mutex_enter_blocking(&mutex);
  uint8_t buf[2] = { 0xC0, n };
  pio_midi_out_write_tx_buffer(midi_port, buf, 2);
  mutex_exit(&mutex);
}

static void control_change(const uint_fast8_t ctl, const uint_fast16_t n) {
  mutex_enter_blocking(&mutex);
  uint8_t buf[3] = { 176, ctl, n };
  pio_midi_out_write_tx_buffer(midi_port, buf, 3);
  mutex_exit(&mutex);
}

static bool mute;
static bool mute_change;
static void alt_callback(uint32_t events) {
  static uint64_t old_time;
  const uint64_t time = time_us_64();
  if(events == GPIO_IRQ_EDGE_RISE) {
    old_time = time;
  } else {
    if ((time - old_time) < timeout) {
      alt = !alt;
      if(alt)
        set_dot(AB);
      else
         unset_dot(AB);
      if(!mode) set_dot(PGM);
    } else {
      mute_change =  true;
    }
  }
}

static void fn_callback(uint32_t events) {
  static uint64_t old_time;
  const uint64_t time = time_us_64();
  if(events == GPIO_IRQ_EDGE_RISE) {
    old_time = time;
  } else {
    if ((time - old_time) < timeout) {
      ab = !ab;
      set_digit(AB, USER + ab);
      set_dot(PGM);
    } else {
      mode = true;
      set_dot(BANK);
    }
  }
}

static void gpio_callback(uint gpio, uint32_t events) {
  if(gpio == ALT_PIN) alt_callback(events);
  else fn_callback(events);
}

static void on_pgm(void) {
  pio_interrupt_clear(pio1, 0);
  irq_clear(PIO1_IRQ_0);
  has_pgm = true;
}

static inline uint8_t sw_get(void) {
  const uint32_t n = pio_sm_get(pio1, sws_sm);
  if(n==8) return 3;
  return n/2;
}

static void handle_pgm(void) {
  has_pgm = false;
  const uint32_t n = sw_get() + 4*alt;

  if(!mode) {
    unset_dot(PGM);
    program_change(64*ab + 8*bank + n);
    set_digit(PGM, n + 1);
  } else {
    mode = false;
    bank = n;
    unset_dot(BANK);
    set_digit(BANK, bank + 1);
    set_dot(PGM);
  }
}

static void setup_mute(void) {
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, true);
}

static void setup_irq(void) {
  gpio_init(ALT_PIN);
  gpio_set_dir(ALT_PIN, false);
  gpio_set_irq_enabled_with_callback(ALT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  gpio_init(AB_PIN);
  gpio_set_dir(AB_PIN, false);
  gpio_set_irq_enabled(AB_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

static void setup_midi(void) {
  midi_port = pio_midi_out_create(MIDI_PIN);
  mutex_init(&mutex);
}

static uint16_t adc_val[N_ADC];
static uint16_t adc_acc[N_ADC][SAMPLES];

static inline uint16_t adc_mean(const uint n) {
  uint16_t mean = 0;
  for(uint i = 0; i < SAMPLES; i++) {
    mean += adc_acc[n][i];
  }
  mean /= SAMPLES;
  return mean >>= 4;
}
static uint sample_count[N_ADC];
static inline void adc_get(const uint n, const uint ctl) {
  adc_select_input(n);
  adc_acc[n][sample_count[n]++] = adc_read();
  if(sample_count[n] == 10) {
    const uint16_t mean = adc_mean(n);
    if(mean != adc_val[n]) {
      control_change(ctl, mean);
      adc_val[n] = mean;
    }
    sample_count[n] = 0;
  }
}

static void adc(void) {
  adc_init();
  adc_gpio_init(27);
  adc_gpio_init(28);
  while(true) {
    adc_get(0, volume);
    adc_get(1, expression);
  }
}

int main() {
  setup_mute();
  setup_irq();
  setup_segment(first_segment_pin, first_digit_pin);
  sws_sm = setup_pgm(SW_PIN, on_pgm);
  setup_midi();

  multicore_launch_core1(adc);

  set_digit(PGM, 1);
  set_digit(AB, USER);
  set_digit(BANK, 1);
  program_change(0);

  while(true) {
    if(has_pgm) handle_pgm();
    if(mute_change) {
      mute_change = false;
      gpio_put(LED_PIN, mute = !mute);
      control_change(0x50, mute*127);
    }
    mutex_enter_blocking(&mutex);
    pio_midi_out_drain_tx_buffer(midi_port);
    mutex_exit(&mutex);
  }
}
