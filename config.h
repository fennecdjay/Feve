#pragma once

const uint8_t first_segment_pin = 18;
const uint8_t first_digit_pin = 15;

const uint32_t timeout = 500000;

#define PGM  2
#define AB   0
#define BANK 1

static const uint SW_PIN = 9;
static const uint LED_PIN = 7;
static const uint ALT_PIN = 5;
static const uint AB_PIN  = 6;
static const uint MIDI_PIN = 0;

#define N_ADC 2
#define SAMPLES 10
static const uint volume = 0x07;
static const uint expression = 0x10;
