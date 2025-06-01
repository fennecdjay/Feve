#include <hardware/address_mapped.h>
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
  // buf[1] = ctl + 32;
  // buf[2] = (uint8_t)n;
  // pio_midi_out_write_tx_buffer(midi_port, buf, 3);
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
  if(gpio == 4) alt_callback(events);
  else fn_callback(events);
}

static void on_pgm(void) {
  pio_interrupt_clear(pio1, 0);
  irq_clear(PIO1_IRQ_0);
  has_pgm = true;
}

static uint8_t get(void) {
  const uint32_t n = pio_sm_get(pio1, sws_sm);
  if(n==8) return 3;
  return n/2;
}


static void handle_pgm(void) {
  has_pgm = false;
  const uint32_t n = get() + 4*alt;
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

static void adc(void) {
  uint8_t count0 = 0;
  uint16_t adc0[SAMPLES];
  uint16_t old0 = 0;
  uint8_t count1 = 0;
   uint16_t adc1[SAMPLES];
  uint16_t old1 = 0;
  adc_init();
  adc_gpio_init(27);
  adc_gpio_init(28);
  while(true) {
    adc_select_input(0);
    adc0[count0++] = adc_read();
    if(count0 == SAMPLES) {
      uint64_t mean = 0;
      for(uint8_t i = 0; i < SAMPLES; i++)
        mean += adc0[i];
      mean /= SAMPLES;
      mean >>= 5;
      if(mean != old0) {
        control_change(0x07, mean);
        old0 = mean;
      }
    }
    adc_select_input(1);
    adc1[count1++] = adc_read();
    if(count1 == SAMPLES) {
      uint64_t mean = 0;
      for(uint8_t i = 0; i < SAMPLES; i++)
        mean += adc1[i];
      mean /= SAMPLES;
      mean >>= 5;
      if(mean != old1) {
        control_change(0x10, mean);
        old1 = mean;
      }
    }
  }
}

int main() {
  setup_mute();
  setup_irq();
  setup_segment(first_segment_pin, first_digit_pin);
  sws_sm = setup_pgm(on_pgm);
  setup_midi();

  set_digit(PGM, 1);
  set_digit(AB, USER);
  set_digit(BANK, 1);
  program_change(1);

  multicore_launch_core1(adc);
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
