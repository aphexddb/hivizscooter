#include <FastLED.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <avr/wdt.h>

// For LED display hardware SPI is a little faster, but must be wired to specific pins:
// Arduino Uno = pin 11 for data, 13 for clock (other boards are different)
#define DATA_PIN 11             // LED Data Pin on Arduino
#define CLOCK_PIN 13            // LED Clock Pin on Arduino
#define BRAKE_SWITCH_PIN 7      // Digital Pin connected to brake switch
#define NUM_STRIPS 2            // Number of LED strips
#define NUM_LEDS_PER_STRIP 36   // Number of LEDs in strip
#define COLOR_ORDER BGR         // Adafruit DotStar pixels have RGB order of: Blue, Green, Red
#define MAX_INT_VALUE 65535     // Max value of an int
#define ANIMATION_FRAMERATE 100 // Framerate for animations
#define MAX_BRIGHTNESS 10       // Maximum LED brightness, watch the power!
#define DEBUG                   // Uncomment to run in debug mode

// debugging print to serial
#ifdef DEBUG
#define DPRINT(...) Serial.print(__VA_ARGS__)     // DPRINT is a macro, debug print
#define DPRINTLN(...) Serial.println(__VA_ARGS__) // DPRINTLN is a macro, debug print with new line
#else
#define DPRINT(...)   // defines a blank line
#define DPRINTLN(...) // defines a blank line
#endif

uint8_t GetState();
void SetScooterState(uint8_t sensors);
void SetAnimateBraking();
void SetAnimateIdle();
void AnimateFlashColor(CRGB strip[], uint16_t animation_frame, CRGB flash_color);
void AnimateDrawFractionalBar(CRGB strip[], int pos16, int width, uint8_t hue, bool wrap);
void AnimateWave(CRGB strip[], uint16_t animation_frame, uint8_t hue);
void Bounce(CRGB strip[], uint8_t start_position, uint16_t animationFrame, uint8_t hue);
void AnimateBounce(CRGB strip[], uint16_t frame);

// animation states
enum AnimationStates
{
  ANIMATION_IDLE,
  ANIMATION_BRAKING
};

// sensor states
enum SensorStates
{
  SENSOR_NONE,
  SENSOR_BRAKE
};

// scooter state machine values
enum ScooterStates
{
  STATE_IDLE,
  STATE_BRAKING
};

// state machine
uint8_t state = STATE_IDLE;

// Colors
const CRGB color_amber = CRGB(255, 126, 0); // SAE/ECE Amber #ff7e00
const CRGB color_red = CRGB(255, 0, 0);     // Red #ff0000
uint8_t brightness = MAX_BRIGHTNESS;

// setup our pixel matrix
const int NUM_LEDS = NUM_STRIPS * NUM_LEDS_PER_STRIP;
CRGB leds[NUM_LEDS];

// animation config
uint16_t animate_speed = ANIMATION_FRAMERATE; // Number of frames to increment per loop
uint16_t frame = 0;                           // Current frame

void setup()
{
  // treat the LED matrix as one giant strip
  FastLED.addLeds<DOTSTAR, DATA_PIN, CLOCK_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(brightness);
  FastLED.clear();

  // FastLED 2.1 Power management set at 5V, 500mA
  set_max_power_in_volts_and_milliamps(5, 500);

  // set the brake switch pin to input mode
  pinMode(BRAKE_SWITCH_PIN, INPUT);

  // Enable watchdog in case something gets wierd. Reset device
  // if no watchdog reset recieved in the specified timeframe.
  wdt_enable(WDTO_4S); // 4 sec watchdog

#ifdef DEBUG
  Serial.begin(9600);
#endif
}

void loop()
{
  // The main loop checks for state changes each iteration
  SetScooterState(GetState());

  // Animations are a function of the current animation frame "frame"
  // Once you've applied the animation to the current strip/frame,
  // Its up to the main loop to send the data to the strip(s)
  // using FastLED.show()
  FastLED.show();
  frame += animate_speed;

  // reset watchdog timer
  wdt_reset();
}

// Read the state of any connected sensors
uint8_t GetState()
{
  if (digitalRead(BRAKE_SWITCH_PIN) == HIGH)
  {
    return SENSOR_BRAKE;
  }
  return SENSOR_NONE;
}

// State machine: set state based on current state and sensors
void SetScooterState(uint8_t sensors)
{
  switch (state)
  {
  case STATE_IDLE:
    if (sensors == SENSOR_NONE)
    {
      SetAnimateIdle();
    }
    else if (sensors == SENSOR_BRAKE)
    {
      state = STATE_BRAKING;
      DPRINTLN("Brake sensor on");
      SetAnimateBraking();
    }
    break;

  case STATE_BRAKING:
    if (sensors != SENSOR_BRAKE)
    {
      state = STATE_IDLE;
      DPRINTLN("Brake sensor off");
      SetAnimateIdle();
    }
    else
    {
      SetAnimateBraking();
    }
    break;
  }
}

/**
 * Set different LED animations
 */

void SetAnimateBraking()
{
  AnimateFlashColor(leds, frame, color_red);
}

void SetAnimateIdle()
{
  // AnimateBounce(leds, frame);
  AnimateWave(leds, frame, HUE_ORANGE);
}

/**
 * Supporting animation functions
 * thanks to hsiboy (https://gist.github.com/hsiboy/f9ef711418b40e259b06)
 */

// Flash LED strip with a color
void AnimateFlashColor(CRGB strip[], uint16_t animation_frame, CRGB flash_color)
{
  if (animation_frame > MAX_INT_VALUE / 3)
  {
    fill_solid(leds, NUM_LEDS, flash_color);
  }
  else
  {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
}

// Uses FastLED sin16() and no float math for efficiency.
// Color is determined by "hue"
void AnimateWave(CRGB strip[], uint16_t animation_frame, uint8_t hue)
{
  FastLED.clear();
  uint8_t strip_length = sizeof(leds) / sizeof(CRGB);
  uint8_t value;
  for (uint8_t i = 0; i < strip_length; i++)
  {
    value = (sin16(animation_frame + ((MAX_INT_VALUE / strip_length) * i)) + (MAX_INT_VALUE / 2)) / 256;
    if (value >= 0)
    {
      strip[i] += CHSV(hue, 255, value);
    }
  }
}

// Linear "Larson scanner" (or knight rider effect) with anti-aliasing. Color is determined by "hue"
void Bounce(CRGB strip[], uint8_t start_position, uint16_t animationFrame, uint8_t hue)
{
  uint16_t pos16;
  if (animationFrame < (MAX_INT_VALUE / 2))
  {
    pos16 = animationFrame * 2;
  }
  else
  {
    pos16 = MAX_INT_VALUE - ((animationFrame - (MAX_INT_VALUE / 2)) * 2);
  }

  int position = map(pos16, 0, MAX_INT_VALUE, 0, ((NUM_LEDS_PER_STRIP)*16));
  AnimateDrawFractionalBar(strip, start_position + position, 3, hue, 0);
}

// 3 chaser animations offset by 120 degrees each
void AnimateBounce(CRGB strip[], uint16_t frame)
{
  FastLED.clear(); //Clear previous buffer
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    uint8_t start_position = i * NUM_LEDS_PER_STRIP;

    Bounce(strip, start_position, frame, 0);
    Bounce(strip, start_position, frame + (MAX_INT_VALUE / 3), 100);
    Bounce(strip, start_position, frame + (MAX_INT_VALUE / 3) * 2, 150);
  }
}

// Anti-aliasing code care of Mark Kriegsman Google+: https://plus.google.com/112916219338292742137/posts/2VYNQgD38Pw
void AnimateDrawFractionalBar(CRGB strip[], int pos16, int width, uint8_t hue, bool wrap)
{
  uint8_t stripLength = sizeof(strip) / sizeof(CRGB);
  uint8_t i = pos16 / 16; // convert from pos to raw pixel number

  uint8_t frac = pos16 & 0x0F; // extract the 'factional' part of the position
  uint8_t firstpixelbrightness = 255 - (frac * 16);

  uint8_t lastpixelbrightness = 255 - firstpixelbrightness;

  uint8_t bright;
  for (int n = 0; n <= width; n++)
  {
    if (n == 0)
    {
      // first pixel in the bar
      bright = firstpixelbrightness;
    }
    else if (n == width)
    {
      // last pixel in the bar
      bright = lastpixelbrightness;
    }
    else
    {
      // middle pixels
      bright = 255;
    }

    strip[i] += CHSV(hue, 255, bright);
    i++;
    if (i == stripLength)
    {
      if (wrap == 1)
      {
        i = 0; // wrap around
      }
      else
      {
        return;
      }
    }
  }
}