#ifndef ledstrip_home_h
#define ledstrip_home_h

typedef struct {
  bool fading;
  uint8_t ledFadeMode;  // color fade mode, 0 for entire strip, 1 for swipe pixels
  unsigned long fadeTimeInterval;
  float ledStripBrightness;
} led_strip_disp_state;
#endif

