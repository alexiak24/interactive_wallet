/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
  THIS CODE IS STILL IN PROGRESS!

  Open up the serial console on the Arduino at 115200 baud to interact with FONA


  This code will receive an SMS, identify the sender's phone number, and automatically send a response

  For use with FONA 800 & 808, not 3G
*/

#include "Adafruit_FONA.h"

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

// this is a large buffer for replies
char replybuffer[255];
int LastAccess;
int count = 0;
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
const int buttonPin = 5;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

String phoneNumber = "9292156383";

// variables will change:
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

void enableGPRS() {
  fona.enableGPRS(false);
  Serial.println(F("Enabling GPRS"));
  if (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to turn GPRS on. Trying again."));
    enableGPRS();
  }
}

void setup() {
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("FONA SMS caller ID test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  // make it slow so its easy to read!
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }

  enableGPRS();

  Serial.println("FONA Ready");
}



String msg = "";
char fonaInBuffer[64];          //for notifications from the FONA

void loop() {

  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter++;
      if (buttonPushCounter > 7)
      {
        buttonPushCounter = 0;
        char callerIDbuffer[32];
        msg = "You're spending too much!";
        msg.toCharArray(replybuffer, 144);
        phoneNumber.toCharArray(callerIDbuffer, 30);
        if (!fona.sendSMS(callerIDbuffer, replybuffer)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
      }

      Serial.println("on");
      Serial.print("number of button pushes:  ");
      Serial.println(buttonPushCounter);
      digitalWrite(ledPin, HIGH);
    } else {
      // if the current state is LOW then the button
      // wend from on to off:
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
    // Delay a little bit to avoid bouncing
    delay(600);
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;




  char* bufPtr = fonaInBuffer;    //handy buffer pointer

  if (fona.available())      //any data available from the FONA?
  {
    int slot = 0;            //this will be the slot number of the SMS
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaInBuffer) - 1)));

    //Add a terminal NULL to the notification string
    *bufPtr = 0;

    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaInBuffer, "+CMTI: \"SM\",%d", &slot)) {
      Serial.print("slot: "); Serial.println(slot);

      char callerIDbuffer[32];  //we'll store the SMS sender number in here

      // Retrieve SMS sender address/phone number.
      if (! fona.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);


      //TODO

      float latitude, longitude;

      if ((fona.type() == FONA3G_A) || (fona.type() == FONA3G_E))
        return;
      if (fona.getNetworkStatus() == 1) {
        // network & GPRS? Great! Print out the GSM location to compare
        boolean gsmloc_success = fona.getGSMLoc(&latitude, &longitude);

        if (gsmloc_success) {

          msg = "http://maps.google.com/maps?q=" + String(latitude, 8) + "," + String(longitude, 8);
          msg.toCharArray(replybuffer, 144);
          Serial.println(msg);
          Serial.print("GSMLoc lat:");
          Serial.println(latitude, 6);
          Serial.print("GSMLoc long:");
          Serial.println(longitude, 6);
        }
        // delete the original msg after it is processed
        //   otherwise, we will fill up all the slots
        //   and then we won't be able to receive SMS anymore
        if (fona.deleteSMS(slot)) {
          Serial.println(F("OK!"));
        } else {
          Serial.println(F("Couldn't delete"));
        }
      } else {
        Serial.println("GSM location failed...");
        Serial.println(F("Disabling GPRS"));
        fona.enableGPRS(false);
        Serial.println(F("Enabling GPRS"));
        if (!fona.enableGPRS(true)) {
          Serial.println(F("Failed to turn GPRS on"));
        }
      }

      //Send back an automatic response
      Serial.println("Sending reponse...");

      if (!fona.sendSMS(callerIDbuffer, replybuffer)) {
        Serial.println(F("Failed"));
      } else {

        Serial.println(F("Sent!"));
      }



    }




  }
}


// 3 hours= 10,800,000 miliseconds









