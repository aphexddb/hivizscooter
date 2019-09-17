#include <FastLED.h>
#include <Arduino.h>
#include <avr/wdt.h>

// For LED display hardware SPI is a little faster, but must be wired to specific pins:
// Arduino Uno = pin 11 for data, 13 for clock (other boards are different)
#define DATA_PIN 11           // LED Data Pin on Arduino
#define CLOCK_PIN 13          // LED Clock Pin on Arduino
#define BRAKE_SWITCH_PIN 7    // Digital Pin connected to brake switch
#define NUM_STRIPS 1          // Number of LED strips
#define NUM_LEDS_PER_STRIP 72 // Number of LEDs in strip
// #define NUM_LEDS_PER_STRIP 36   // Number of LEDs in strip
#define COLOR_ORDER BGR         // Adafruit DotStar pixels have RGB order of: Blue, Green, Red
#define MAX_INT_VALUE 65536     // Max value of an int
#define ANIMATION_FRAMERATE 100 // Framerate for animations
// #define MAX_BRIGHTNESS 255      // Maximum LED brightness, watch the power!
#define MAX_BRIGHTNESS 10 // Maximum LED brightness, watch the power!

uint8_t ReadSensors();
void GetState(uint8_t sensors);
void SetAnimateBraking();
void SetAnimateIdle();
void AnimateDoubleChaser(CRGB strip[], uint16_t animation_frame, uint8_t hue);
void AnimateFlashColor(CRGB strip[], uint16_t animation_frame, CRGB flash_color);
void AnimateRing(CRGB strip[], uint16_t animation_frame, uint8_t hue);
void AnimateDrawFractionalBar(CRGB strip[], int pos16, int width, uint8_t hue, bool wrap);
void AnimateWave(CRGB strip[], uint16_t animation_frame, uint8_t hue);
void AnimateSpark(CRGB strip[], uint16_t animation_frame, uint8_t fade);
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

  // Enable watchdog. We use this to reset device if no
  // watchdog reset recieved in the specified timeframe.
  // wdt_enable(WDTO_4S); // 4 sec watchdog
}

void loop()
{
  GetState(ReadSensors());

  // Each animation adjusts the "strip" specified in its parameter.
  // Animations are a function of the current animation frame "frame"
  // Once you've applied the animation to the current strip/frame,
  // Its up to the main loop to send the data to the strip(s)
  FastLED.show();
  frame += animate_speed;

  // reset watchdog timer
  // wdt_reset();
}

// Read the state of any connected sensors
uint8_t ReadSensors()
{
  if (digitalRead(BRAKE_SWITCH_PIN) == HIGH)
  {
    return SENSOR_BRAKE;
  }
  return SENSOR_NONE;
}

// Get the current scooter state based on the state machine here
void GetState(uint8_t sensors)
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
      SetAnimateBraking();
      state = STATE_BRAKING;
    }
    break;

  case STATE_BRAKING:
    if (sensors != SENSOR_BRAKE)
    {
      SetAnimateIdle();
      state = STATE_IDLE;
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
  animate_speed = ANIMATION_FRAMERATE * 2.5;
  AnimateFlashColor(leds, frame, color_red);
  // AnimateDoubleChaser(leds, frame, HUE_RED);
}

void SetAnimateIdle()
{
  animate_speed = 100;
  AnimateWave(leds, frame, HUE_ORANGE);
  // AnimateSpark(leds, frame, 100);
  // AnimateBounce(leds, frame);
}

/**
 * Supporting animation functions
 */

// 2 chaser animations offset 180 degrees
void AnimateDoubleChaser(CRGB strip[], uint16_t animation_frame, uint8_t hue)
{
  FastLED.clear(); // Clear previous buffer
  animation_frame = animation_frame * 2;
  AnimateRing(strip, animation_frame, hue);
  AnimateRing(strip, animation_frame + (MAX_INT_VALUE / 2), hue);
}

// Flash LED strip color for half of animation length
void AnimateFlashColor(CRGB strip[], uint16_t animation_frame, CRGB flash_color)
{
  if (animation_frame > (MAX_INT_VALUE / 2))
  {
    fill_solid(leds, NUM_LEDS, flash_color);
  }
  else
  {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
}

// Anti-aliased cyclical chaser, 3 pixels wide
// Color is determined by "hue"
void AnimateRing(CRGB strip[], uint16_t animation_frame, uint8_t hue)
{
  uint8_t strip_length = sizeof(leds) / sizeof(CRGB);
  int pos16 = map(animation_frame, 0, MAX_INT_VALUE, 0, ((strip_length)*16));
  AnimateDrawFractionalBar(strip, pos16, 3, hue, 1);
}

// Anti-aliasing code care of Mark Kriegsman Google+: https://plus.google.com/112916219338292742137/posts/2VYNQgD38Pw
void AnimateDrawFractionalBar(CRGB strip[], int pos16, int width, uint8_t hue, bool wrap)
{
  uint8_t strip_length = sizeof(leds) / sizeof(CRGB);
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
    if (i == strip_length)
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

//******************************       Spark       **********************************
// Same as color spark but no hue value, // in HSV white is any hue with 0 saturation
// Frequency of sparks is a percentage mapped to global var "animateSpeed"
// "animateSpeed" var contrained from 1 - 255 (0.4% - 100%)
// "fade" parameter specifies dropoff (next frame brightness = current frame brightness * (x/256)
// fade = 256 means no dropoff, pixels are on or off
// NOTE: this animation doesnt clear the previous buffer because the fade/dropoff is a function of the previous LED state
//***********************************************************************************
void AnimateSpark(CRGB strip[], uint16_t animation_frame, uint8_t fade)
{
  uint8_t stripLength = sizeof(leds) / sizeof(CRGB);
  uint16_t rand = random16();

  for (int i = 0; i < stripLength; i++)
  {
    strip[i].nscale8(fade);
  }

  if (rand < (MAX_INT_VALUE / (256 - (constrain(animate_speed, 1, 55)))))
  {
    strip[rand % stripLength].setHSV(0, 0, 255);
  }
}

//*********************     Bounce      ***************************
// Linear "Larson scanner" (or knight rider effect) with anti-aliasing
// Color is determined by "hue"
//*****************************************************************
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
    // Bounce(strip, start_position, frame + (MAX_INT_VALUE / 3) * 2, 150);
  }
}