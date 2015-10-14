This sketch provides a programmable wireless controller for a LED strip(s) using an Arduino and Bluetooth (or other serial).  Commands can be sent to a BT module
attached to an Arduino to program the mode of the LED strip.  This can be used for home lighting strips, for example.  Programmable feature values are stored in EEPROM 
so the LED strip starts in the last-known-state when it's turned on.

This code supports multiple types of LED strips.  Initially WS2801,WS2811/12 strips are supported, through support graciously provided by the AdafruitWS2801 library.  Support
for PWM-based single-color or RGB strips will be added.

Programmable features include color, brightness, fade mode, animations, and more.

Different features are available for addressable and non-addresable strips.
