 /*
RACS 40h translation
May 2014
https://github.com/rbnstubbs/trellis-monome

///////////////////////////////////////////////////////////////////////
This program is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License as published by 
the Free Software Foundation; either version 2 of the License, or 
(at your option) any later version.

This program is distributed in the hope that it will be useful, but 
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License 
for more details.
///////////////////////////////////////////////////////////////////////

Designed to work with the Adafruit Trellis (four boards)
Uses Trellis XY Library and monomecereal.maxpat, but it will also work 
with monome's serialOSC if the USB chip is flashed. A 3D printed enclosure 
was adapted from Adafruit's implementation. All files are on the git.

This sketch was largely based on the Arduinome clone originally by Owen 
Vallis & Jordan Hochenbaum: http://flipmu.com/work/arduinome/

http://monome.org/

Robin Stubbs
robin.stubbs@uleth.ca
*/


///Trellis and XY Init
 
    #include <Wire.h>
    #include <Adafruit_Trellis.h>
    #include <Adafruit_Trellis_XY.h>

    Adafruit_Trellis matrix0 = Adafruit_Trellis();
    Adafruit_Trellis matrix1 = Adafruit_Trellis();
    Adafruit_Trellis matrix2 = Adafruit_Trellis();
    Adafruit_Trellis matrix3 = Adafruit_Trellis();

    Adafruit_TrellisSet trellis =  Adafruit_TrellisSet(&matrix0, &matrix1, &matrix2, &matrix3);

    #define NUMTRELLIS 4

    #define numKeys (NUMTRELLIS * 16)

    #define INTPIN A2
    
    // Frequency to run the interrupt at
    #define TIE 2
    #define TEN 1
    #define FREQ 90 // in Hertz
    //-----------------------------------------------------------
    // define offsets for 4X4 Tiles
    struct TileOffsets TileOffsets = {
      { 0, 4, 0, 4, 0, 0, 0, 0 },
      { 0, 0, 4, 4, 0 ,0, 0, 0 }
    };

    uint8_t xVal;
    uint8_t yVal;
    uint8_t xyTrellisID;
    
    
    unsigned long interval = 16;  // the time we need to wait CAN BE ALTERED (used in debounce and LED tests) 
    unsigned long previousMillis=0; // millis() returns an unsigned long.

void timer_setup() {
  // Teensy 3.0 version
  SIM_SCGC6 |= SIM_SCGC6_PIT; // Activates the clock for PIT
  // Turn on PIT
  PIT_MCR = 0x00;
  // Set the period of the timer.  The µC runs at 50MHz
  // So interrupt length can be determined by 50Mhz/FREQ.
  PIT_LDVAL1 = 50000000/FREQ;
  // Enable interrupts on timer1
  PIT_TCTRL1 = TIE;
  // Start the timer
  PIT_TCTRL1 |= TEN;
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1); // Another step to enable PIT channel 1 interrupts
}
        

    /// SETUP    
void setup() {
      Serial.begin(115200);
      Serial.println("racs arduinome");
     
      // INT pin requires a pullup
      pinMode(INTPIN, INPUT);
      digitalWrite(INTPIN, HIGH);
      trellis.begin(0x71, 0x70, 0x73, 0x72);
      // LED Brightness 
	  trellis.setBrightness(3); 
     
      // order is important here!! Start with tile address you want to
      // use as the first one, etc.

      timer_setup();

    
  /// XY init seq SET
   setRows();   
   cli();
   sei();
   
    clearRows();
}


///////////////////////////////////////////////////////////////////////
// Global variables 

byte byte0, byte1, row = 0;                      // storage for incoming serial data

boolean WaitingCommand = true;          // true when we expect the next serial byte to be an address value, false otherwise
int command =  0;                       // garbage int to hold address of function

byte state = 0x00;                      // garbage byte to hold state value
byte x = 0x00;                          // garbage byte to hold x position
byte y = 0x00;                          // garbage byte to hold y position
byte z = 0x00;                          // garbage byte to iterate over buttons




// The following variables are used to store flags that indicate whether we have received a given message,
// the value of the data in that message.  e.g. IntensityChange = 0 == no intensity change message received,
// IntensityChange = 1 == an intensity change message has been received, and it's value will be in IntensityVal
// The code that eventually acts upon the incoming message will reset the 'Change' variable to 0 once the 
// message has been handled.            

int ShutdownModeChange = 0;            
boolean ShutdownModeVal= false;

int i = 0;                              // temporary variable for looping etc.
int id = 0;                             // temporary storage for incoming message ID

// These variables are used to handle the ADC messages, and store which ports are currently enabled,
// and which are not.
byte ADCnum;
byte ADCstate;
byte ADCEnableState[4];


////////////////////////////////////////////////
/// RACS - button states

    void checkButtons () {      /// RACS - WORKING
      if ((unsigned long)(millis() - previousMillis) >= interval) {
                
        if (trellis.readSwitches()) {

          for (uint8_t i=0; i<numKeys; i++) {

            if (trellis.justPressed(i)) {
               TrellisToXY(i, TileOffsets, &xVal, &yVal);      
               Serial.write((0 << 4) | (1 & 15));      // OG           Serial.write((0 << 4) | (b & 15));
               Serial.write((yVal << 4) | (xVal & 15));      // OG          Serial.write((id << 4) | (i & 15));              
            }
            if (trellis.justReleased(i)) {
               TrellisToXY(i, TileOffsets, &xVal, &yVal);
               Serial.write((0 << 4) | (0 & 15)); 
               Serial.write((yVal << 4) | (xVal & 15));
       	   }
          }
        }
         previousMillis = millis();
      }
    }
                          


///////////////////////////////////////////////
/// LED Functions

void clearRows() {
      for (uint8_t xVal=0; xVal<8; xVal++) {
        yVal=0;
          while (yVal<8) {
          if ((unsigned long)(millis() - previousMillis) >= interval) {
          int LED = XYToTrellis(numKeys, TileOffsets, xVal, yVal);
          trellis.clrLED(LED);
          trellis.writeDisplay();
          yVal++;
          previousMillis = millis();
          }
          }
      }
  }
// ////////////////////////////////////////////
  // Brian's XY init seq  
	  void setRows() {
      for (uint8_t yVal=0; yVal<8; yVal++) {
          xVal=0;
          while (xVal<8) {
            int LED = XYToTrellis(numKeys, TileOffsets, xVal, yVal);
            if ((unsigned long)(millis() - previousMillis) >= interval) {
			trellis.setLED(LED);
			trellis.writeDisplay();
			trellis.setBrightness(xVal);	//Brightness xVal 0-7 brightness  
			xVal++;
			previousMillis = millis();
        }
     }
  }
}



///////////////////////////////////////////////
/// RACS JUST Check Serial now (pretty much straight out of the Arduinome)
/// This should eventually be re-implemented as an interrupt, it seems to lag a little

void pit1_isr() {
    PIT_TFLG1 = 1; 
  // first up: enable interrupts to keep the serial
  // class responsive
  sei(); 
    
    do {
      if (Serial.available()) {
        if (WaitingCommand) {          // command is the first byte of the two byte command
          byte0 = Serial.read();
          command = byte0 >> 4;
          WaitingCommand = false;
        }

           if(Serial.available()) { // command outside range so must be an error
              WaitingCommand = true; // the next byte is the first byte of the next command
              byte1 = Serial.read(); // read the second byte of this command
 
              switch(command) {
 
                case 2: // LED command
                state = byte0 & 0x0f; // is the command for on or off
                x = ((byte1 >> 4) & 0x07); // mask so we don't go over the 8 by 8 grid
                y = ((byte1 & 15) & 0x07);
 
                  if (state == 0) {
                      trellis.clrLED(XYToTrellis(numKeys, TileOffsets, y, x)); //offLED(x, y);
                 }
                   else {
                   trellis.setLED(XYToTrellis(numKeys, TileOffsets, y, x)); //onLED(x, y);
                   }
                  break;
 
                case 3: // led intensity command
                break;
                 
                case 4: // led test command
                  if( (byte1 & 1) == 0) {
                    clearRows();
                 }
                  else {
                    setRows();
                 }
                break;
                
                case 5: // enable ADC command - but we don't do this
                  state = byte1 & 0x0F;
                  ADCEnableState[(byte1 >> 4)&0x03] = state;
                break;
                
                case 6: // shutdown command - not sure what the monome is expected to do here
                  ShutdownModeChange = true;
                  ShutdownModeVal = byte1 & 15;
                break;
                
                case 7: // led row command
                  y = byte0 & 0x07; // mask this value so we don't write to an invalid address.
                  x = 0;
                  for(byte i = 0x1; i != 0x0; i <<= 1 ) {
                    if( (i & byte1) != 0) {
                    trellis.setLED(XYToTrellis(numKeys, TileOffsets, y, x)); //onLED(x, y);
                   }
                  else {
                       trellis.clrLED(XYToTrellis(numKeys, TileOffsets, y, x)); // offLED(x, y);
                  }
                  x++;
                }
                break;
 
                case 8: // colomn command
                  x = byte0 & 0x07; // mask this value so we don't write to an invalid address.
                  y = 0;
                  for(byte i = 0x1; i != 0x0; i <<= 1 ) {
                    if( (i & byte1) != 0) {
                      trellis.setLED(XYToTrellis(numKeys, TileOffsets, y, x)); //onLED(x, y);
                    }
                    else {
                      trellis.clrLED(XYToTrellis(numKeys, TileOffsets, y, x)); //offLED(x, y);
                    }
                  y++;
                }
                break;
               } // end switch(command)
              } // end of else in if(WaitingCommand)
             } // end of if(Serial.available();
            } // end of do
           while (Serial.available() > 8); // keep on going if we have a lot of commands stacked up
          }


void sendADC(int port, int value) {
  Serial.write((1 << 4) | ((port << 2) & 0x0C) | ((value >> 8) & 0x03));
  Serial.write(value & 0xFF);
}

int current[4]; 
int previous[4]; 
int tolerance = 7;

void checkADCs() {

  for (int adcCounter = 0; adcCounter < 4; adcCounter++)
  {

    if (ADCEnableState[adcCounter] != 0)
    {
      current[adcCounter] = analogRead(adcCounter);

      if (abs(previous[adcCounter]-current[adcCounter]) > tolerance)
      {
        previous[adcCounter] = current[adcCounter];
        sendADC(adcCounter,current[adcCounter]);
      }
    }
  }
}


/// RACS - Send it 
void loop() {
  checkButtons();
  trellis.writeDisplay();
  checkADCs();  
 }

// eof

