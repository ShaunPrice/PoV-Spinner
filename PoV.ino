/*
Name:		FSEQLib.cpp
Created:	9/18/2018 5:04:31 PM
Author:	Shaun Price
Contact:	Via Github website below
Copyright (C) 2018-2020 Shaun Price
Website:	https://github.com/ShaunPrice/FSEQLib

Version 2.0.0

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/
/*
SD Card VSPI PINS (ESP32 - Wemos Lolin32 Lite)
==============================================
SCLK = 18
MISO = 19
MOSI = 23
SS = 5

SD Card SPI PINS (ESP8266 - Wemos D1 R2 mini)
==============================================
SCLK = D5/GPIO 14
MISO = D6/GPIO 12
MOSI = D7/GPIO 13
SS   = D8/GPIO 15

Also note Card Detect (CD) and Data Pin defines.
*/

#ifdef ESP8266
#include <SDFSFormatter.h>
#include <SDFS.h>
#endif
#include <SPI.h>
#include <SD.h>
#include <FastLED.h>
#include "FSEQLib.h"


#define DEBUG 0 // 0=OFF, 1=Serial

// Serial Debug Info
#if (DEBUG == 1)
#define SERIAL_BEGIN(x) Serial.begin(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define SERIAL_BEGIN
#define DEBUG_PRINT
#define DEBUG_PRINTLN
#endif

#ifdef ESP8266
// ESP8266
#define DATA_PIN_1 4			// Data pin for universe 1. // D2 - Wemos D1 R2 mini
#define CARD_DETECT_PIN  0		// May require 10k pull-up  // D3 - Wemos D1 R2 mini
#else
// ESP32
#define DATA_PIN_1 13			// Data pin for universe 1.
#define CARD_DETECT_PIN  17		// May require 10k pull-up
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// PoV Defines //////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TRIGGER_PIN 34 // The pin to use to indicate the start possition
#define ENCODER_PIN 35 // The pin to use for the encoder input if an encoder is used.
#define ENCODER_PULSES_PER_REVOLUTION 400 // If this was a 400 pulse encoder it should be 400. 0 means no encoder, use timmed revolution from start
#define LED_SENDS_PER_REVOLUTION 18 // The number of time to send data top the LED's per revolution 
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// End PoV defines //////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// This is where the second universe would go if using more than one universe
//#define DATA_PIN_2 14	// Data pin for universe 2. 

#define FSEQ_FILE "/PoV.dat"	// Name of the FSEQ file to play
#define UNIVERSES 4				// Universes aren't really defined here but I use the term to define the 
								//  number of times I want to split up the sequence step channels into.
#define NUM_NODES 20			// Nodes/Pixels
#define NUM_CHANNELS_PER_NODE 3 // Number of channels/LEDs per Node/Pixel
#define LEDS_PER_UNIVERSE NUM_NODES * NUM_CHANNELS_PER_NODE	// LEDs/channels per universe
#define BUFFER_LENGTH (UNIVERSES * LEDS_PER_UNIVERSE) + ((4 - (UNIVERSES * LEDS_PER_UNIVERSE) % 4) % 4) // Buffer is 32 bit (4 byte) padded.
// Adjust the brightness of the display
// For best results leave brightness at 255 (100%)
// and correct the brightness with Xlights.
#define BRIGHTNESS 255

size_t bytesRead = 0;

File dataFile;

// SPI
char stepBuffer[BUFFER_LENGTH];
CRGB leds[UNIVERSES][NUM_NODES];
//uint8_t universeBuffer[LEDS_PER_UNIVERSE]; // If using uint8_t buffer rather than CRGB buffer

uint32_t stepDelay = 50; // This will get set from the spead of the PoV display
int32_t currentStep = 0;
char* pFileBuffer;
HeaderData rawHeader;
FSEQLib header;

bool cardInitialised = false;
bool cardDetected = false;

///////////////////////////
// PoV Variables //////////
///////////////////////////
bool povTrigger = false;
bool povEncoder = false;
bool povSequenceStart = false;
unsigned long lastTriggerTime = 0;
unsigned long lastSendTime = 0;
unsigned int revolutionSendCount = 0;
unsigned int stepNumber = 0;
unsigned int lastSendCount = 0; 
unsigned int encoderCount = 0;
unsigned long startTime = 0;
//////////////////////////
// End PoV Variables /////
//////////////////////////

// Interupt for the PoV start input
void IRAM_ATTR povTriggerInterupt()
{
    // make sure we havn't just sent the LED's before setting high
    if (millis() - lastTriggerTime > 10) // Don't trigger unless greater than 10mS
    {
        // Reset the last trigger time
        lastTriggerTime = millis();
        // Recalculate the stepDelay
        stepDelay = uint32_t((millis() - startTime) / LED_SENDS_PER_REVOLUTION);
        // Set the last start time to current time
        startTime = millis();
        // Set the trigger/start to high
        povTrigger = true;
        // Reset the encoder count to start again
        encoderCount = 0;
        // Reset the Sequence count
        revolutionSendCount = 0;
    }
}

// Interupt for the PoV encoder input
void IRAM_ATTR povEncoderInterupt()
{
    // Increment the encoder count
    encoderCount++;
    // Make sure we don't exceed the encoder pulses per revolution
    if (encoderCount > ENCODER_PULSES_PER_REVOLUTION)
    {
        encoderCount = ENCODER_PULSES_PER_REVOLUTION;
    }
}

void setup()
{
	SERIAL_BEGIN(115200);

    //////////////////////
	// PoV Pin Modes /////
    pinMode(TRIGGER_PIN, INPUT_PULLDOWN);
    pinMode(ENCODER_PIN, INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), povTriggerInterupt, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), povEncoderInterupt, RISING);

    // End PoV Pon Mode //
    //////////////////////
    
    pinMode(SS, OUTPUT); // SD Card VSPI SS
	pinMode(CARD_DETECT_PIN, INPUT_PULLUP); // SD Card Detected CD 

	FastLED.addLeds<NEOPIXEL, DATA_PIN_1>(leds[0], NUM_NODES);
	// The second universe would be:
	//FastLED.addLeds<NEOPIXEL, DATA_PIN_2>(leds[1], NUM_NODES);

	// Adjust the brightness of the display
	// For best results leave brightness at 255 (100%)
	// and correct the brightness with Xlights.
	FastLED.setBrightness(BRIGHTNESS);

    ////////////////////////////////////////////
    // Load all LED data from SD to memory /////
    ////////////////////////////////////////////

    while (digitalRead(CARD_DETECT_PIN) == 0)
    {
        delay(1000);
        DEBUG_PRINTLN("SD card not detected...");
    }
    
	// Initialise SPI Master

	// See if the card is present and can be initialized:
	DEBUG_PRINTLN("Initializing SD card...");
	//SDFS.setConfig(SDFSConfig(SS));
    while (!cardInitialised)
    {
        if (SD.begin(SS))
        {
            DEBUG_PRINTLN("card initialized.");

            // Open the file.
            // Note that only one file can be open at a time,
            // so you have to close this one before opening another.
            // Also, this only supports 8.3 format so we need to 
            // rename the file from xxx.fseq to something else.
            dataFile = SD.open(FSEQ_FILE, "r");

            DEBUG_PRINTLN("File size: " + String(dataFile.size()));
                
            if (dataFile.size() > 0)
                {
                cardInitialised = true;
                    
                dataFile.readBytes(rawHeader.rawData, 28);
                header = FSEQLib(rawHeader);

                // DEBUG code to print out the header details
                if (header.majorVersion() == 1)
                {
                    DEBUG_PRINTLN( "======================");
                    DEBUG_PRINTLN( "== Xlights FSEQ V1.0 Header");
                    DEBUG_PRINTLN( "======================");
                    DEBUG_PRINTLN( "Magic: " + String(header.magic()));
                    DEBUG_PRINTLN( "Data Offset: " + String(header.dataOffset()));
                    DEBUG_PRINTLN( "Version: " + String(header.majorVersion()) + "." + String(header.minorVersion()));
                    DEBUG_PRINTLN( "Header Length: " + String(header.headerLength()));
                    DEBUG_PRINTLN( "Channels per Step: " + String(header.channelsPerStep()));
                    DEBUG_PRINTLN( "Number of Steps: " + String(header.sequenseSteps()));
                    DEBUG_PRINTLN( "Step Time (ms): " + String(header.stepTime()));
                    DEBUG_PRINTLN( "Universes: " + String((header.universes() == 0) ? 0 : header.universes()));
                    DEBUG_PRINTLN( "Size of Universe: " + String((header.sizeofUniverses() == 0) ? 0 : header.sizeofUniverses()));
                    DEBUG_PRINTLN( "Gamma: " + String(header.gamma()));
                    DEBUG_PRINTLN( "Color Order: " + header.colorOrder());
                    DEBUG_PRINTLN( "======================");
                }
                else
                {
                    DEBUG_PRINTLN("======================");
                    DEBUG_PRINTLN("== Xlights FSEQ V2.0 Header");
                    DEBUG_PRINTLN("======================");
                    DEBUG_PRINTLN( "Magic: " + header.magic());
                    DEBUG_PRINTLN( "Data Offset: " + String(header.dataOffset()));
                    DEBUG_PRINTLN( "Version: " + String(header.majorVersion()) + "." + String(header.minorVersion()));
                    DEBUG_PRINTLN( "Header Length: " + String(header.headerLength()));
                    DEBUG_PRINTLN( "Channels per Step: " + String(header.channelsPerStep()));
                    DEBUG_PRINTLN( "Number of Steps: " + String(header.sequenseSteps()));
                    DEBUG_PRINTLN( "Step Time (ms): " + String(header.stepTime()));
                    DEBUG_PRINTLN( "Compression Type: " + header.compressionTypeName());
                    DEBUG_PRINTLN( "Compressed Blocks: " + String(header.compressedBlocks()));
                    DEBUG_PRINTLN( "Sparse Ranges: " + String(header.sparseRanges()));
                    DEBUG_PRINTLN( "UUID: " + String((char)header.uuid()));
                    DEBUG_PRINTLN( "======================");
                }
                DEBUG_PRINTLN("done!");
                // Set the data offset
                dataFile.seek(header.dataOffset());
            }
            else
            {
                cardInitialised = false;
                dataFile.close();
            }
        }
    }

	// Read the channels for all of the steps
    unsigned int fileLength = BUFFER_LENGTH * UNIVERSES * header.sequenseSteps();
    pFileBuffer = (char*)malloc(fileLength * sizeof(char));
	int filelen = dataFile.readBytes(pFileBuffer, fileLength);

	if (filelen != fileLength)
	{
		DEBUG_PRINTLN("File buffer Failed to load. Closing File and SD card.");
		cardInitialised = false;
	}
    // Close the file
	dataFile.close();
    DEBUG_PRINTLN("File buffer loaded.");

    // We need to calculate the time beteen LED output
    // We can caltulate this from the time between encoder input pulses or the 
    // start pulse were there is only one input per revolution

    // Wait for start pulse to occur twice. The stepDelay is calculated in the interupt
    while (!povTrigger)
    {
        yield();
    }
    povTrigger = false;
 
    // Wait for first revolution to finish
    while (!povTrigger)
    {
        yield();
    }
    povTrigger = false;

    DEBUG_PRINTLN("Calculated Step Delay: " + String(stepDelay));
    // End PoV Load Data into memory
    ////////////////////////////////////////////
}

void loop()
{
    // Check that we haven't reached the end
    if (stepNumber >= header.sequenseSteps())
    {
        stepNumber = 0;
    }   

    // Wait until we receive the start trigger for the cycle
	while (revolutionSendCount == 0)
    {
        while (!povTrigger)
        {
            yield();
        }

        povTrigger = false; // Reset the trigger
        revolutionSendCount = 1;
        DEBUG_PRINTLN("Triggered");
        DEBUG_PRINTLN("Calculated Step Delay: " + String(stepDelay));
    }

    // Load the step buffer
    memcpy(&stepBuffer[0], &pFileBuffer[stepNumber*BUFFER_LENGTH], BUFFER_LENGTH);

	// Output data
	for (uint8_t current_universe = 0; current_universe < UNIVERSES; current_universe++)
	{
		// Copy the led values into the universe buffer
		memcpy(&leds[current_universe], &stepBuffer[current_universe * LEDS_PER_UNIVERSE], LEDS_PER_UNIVERSE);
		//memcpy(&universeBuffer[0], &stepBuffer[current_universe * LEDS_PER_UNIVERSE], LEDS_PER_UNIVERSE); // If using uint8_t buffer rather than CRGB buffer

		// Send the data
		FastLED.show();
        DEBUG_PRINTLN("Send data to universe: " + String(current_universe));
	}

    // Increment the revolution send counter
    revolutionSendCount++;

    // Increment the step counter
    stepNumber++;

	// Delay to make sure we send the number of times specified per second.
	if(ENCODER_PULSES_PER_REVOLUTION == 0)
    {
        // Check if we've finished
        if (revolutionSendCount >= LED_SENDS_PER_REVOLUTION)
        {
            revolutionSendCount = 0;
        }
        else
        {
            // Using time from start trigger
            delay(stepDelay);
        }
    }
    else
    {
        // Using the encoder value
        while ((encoderCount - lastSendCount) < (ENCODER_PULSES_PER_REVOLUTION / LED_SENDS_PER_REVOLUTION))
        {
            yield();
        }

        lastSendCount = encoderCount;

        if (revolutionSendCount >= LED_SENDS_PER_REVOLUTION)
        {
            revolutionSendCount = 0;
           	currentStep++;

            // Reset to first step if we have gone past the last step
            if (currentStep == header.sequenseSteps())
            {
                // Restart at first step
                currentStep = 0;
            }
        }
    }
}
