# PoV-Spinner
Persistence of Vision Spinner for xLights using teh FSEQLib and FaslLED.

**Note:** The FSEQLib library currently does not support version two compressed FSEQ format. Please read the instructions on the [FSEQLib libraries Github page](https://github.com/ShaunPrice/FSEQLib) for configuring xLights.

## Configureation ##

You need to set up teh following in the code:
```
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// PoV Defines //////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TRIGGER_PIN 34 // The pin to use to indicate the start possition
#define ENCODER_PIN 35 // The pin to use for the encoder input if an encoder is used.
#define ENCODER_PULSES_PER_REVOLUTION 400 // If this was a 400 pulse encoder it should be 400. 0 means no encoder, use timmed revolution from start.
#define LED_SENDS_PER_REVOLUTION 18 // The number of time to send data top the LED's per revolution 
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// End PoV defines //////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
```
