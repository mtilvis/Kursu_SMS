/**
 * @brief This example demonstrates basic processing of the incoming SMS message in the 
 * Adeon format. The target is to set the values of defined parameters via SMS in case that
 * the SMS has been sent by authorized user (from authorized phone number).
 * 
 *  Copyright (c) 2020 JSC electronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Arduino.h>
#include <AdeonGSM.h>
#include <utility/SIMlib.h>

#if defined(ESP8266) //setup for ESP8266 boards
    #define LED         2   //D4
    #define RELAY       5   //D1
    #define LED_ON      LOW
    #define LED_OFF     HIGH       
#elif defined (ESP32) //setup for ESP32 boards
    #define LED         2  //D2
    #define RELAY       5  //D5
    #define LED_ON      HIGH
    #define LED_OFF     LOW   
#else //setup for Arduino AVR boards
    #define PUDOTUS        13               // IO, johon lämpöpumpun reletulo kytketään (rele)
    #define LP_ALARM       6                // Lämpöpumpun hälytyslähtö
    #define PUDOTUS_ON      LOW
    #define PUDOTUS_OFF     HIGH   
    #define GSM_POWER   9                   // GSM modeemin käynnistys virtakatkosta
    #define DEBUGOUT    4                   // Vilautetaan lediä kun käynnissä
#endif

// For SendStatus()
#define SS_NORMAL   0
#define SS_ALARM    1
// *** Temperature measurement
// resistance at 25 degrees C
#define THERMISTORNOMINAL 4800      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 24   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 5600    
// What pin to connect the sensor to
#define THERMISTORPIN A0 
// *** Temperature measurement

Adeon adeon = Adeon();
/*GSM Class serial setup
SoftwareSerial default setting for Arduino AVR ATmega328p boards – RX 10, TX 11, BAUD 9600
SoftwareSerial default setting for ESP8266 boards – RX 14, TX 12, BAUD 9600
HardwareSerial default setting for Arduino AVR ATmega2560 boards – Serial2, BAUD 9600
HardwareSerial default setting for ESP32 boards – Serial2, RX 16, TX 17, BAUD 9600
*/
GSM gsm = GSM();

char* msgBuf; 
char* pnBuf; 

static constexpr const char* sendSms = "AT+CMGS=\"+";
static constexpr const char* txt_kotona = "Kotona ";
static constexpr const char* txt_poissa = "Poissa ";
static constexpr const char* txt_LPOK = "Lampopumppu OK!";
static constexpr const char* txt_LPAlarm = "Lampopumppu vikatilassa!";
static constexpr const char* txt_lampo = "Lampotila on : ";
static constexpr const char* txt_lamporaja = "Lamporaja = ";
static constexpr const char* CRLF = "\r\n";

#define SIZEOF_SENDSMS          10
#define SIZEOF_PHONENUMBER      12
#define SIZEOF_TXT_LAMPO        15
#define SIZEOF_TXT_POISSA        7
#define SIZEOF_TXT_KOTONA        7
#define SIZEOF_TXT_LPOK         15
#define SIZEOF_TXT_LPALARM      24
#define SIZEOF_TXT_LAMPORAJA    12
//#define DEBUG_MEMORY

boolean LPAlarm = LOW;
boolean prev_LPAlarm = LPAlarm;
boolean AlaLampoAlarm = LOW;
boolean prev_AlaLampoAlarm = LOW;


void numOfItems();
void IOControl();
void callbackRel(uint16_t val);
void userInit();
void paramInit();
void processMsg();

#ifdef DEBUG_MEMORY
extern char *__brkval;

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
#endif


void numOfItems()
{
  Serial.print(F("NUM OF USERS: "));
  Serial.println(adeon.getNumOfUsers()); //number of saved users

  Serial.print(F("NUM OF PARAMS: "));
  Serial.println(adeon.getNumOfParams()); //number of saved parameters
  Serial.println();
}


void callbackRel(uint16_t val)
{
    (val == 0) ? digitalWrite(LP_ALARM, HIGH) : digitalWrite(LP_ALARM, LOW); 
}

void userInit()
{
    //add users with ADMIN, USER or HOST rights
    adeon.addUser("358405144229", ADEON_ADMIN);     // Marko
//    adeon.addUser("358402513317", ADEON_ADMIN);      // Modeemi ite
    adeon.addUser("358405948061", ADEON_ADMIN);      // Teija
    adeon.printUsers();
}

void paramInit()
{
    //add parameters
    adeon.addParam("Kotona", 1);
    adeon.addParamWithCallback(callbackRel, "LP_ALARM", 0);
    adeon.setParamAccess("LP_ALARM", ADEON_USER);
    adeon.addParam("Lamporaja", 12);
    adeon.setParamAccess("Lamporaja", ADEON_ADMIN);
    adeon.printParams();
}

void IOControl()
{
    // Lämpöpumppuhälytys
    if(digitalRead(LP_ALARM))
    {
        LPAlarm = HIGH;
    }   else {
        LPAlarm = LOW;
    }
    if (LPAlarm != prev_LPAlarm)
    {
        Serial.println();
        Serial.println(F("Lämpöpumppu Hälytys!"));
        adeon.editParamValue("LP_ALARM", (uint16_t)LPAlarm);
        sendStatus(SS_ALARM);
    }
    prev_LPAlarm = LPAlarm;

    // Mitatun lämpötilan hälytys
    if ( (uint16_t)measureTemp() < (uint16_t)adeon.getParamValue("Lamporaja") )
    {
       AlaLampoAlarm = HIGH; 
    } else {
       AlaLampoAlarm = LOW; 
    }

    if (prev_AlaLampoAlarm != AlaLampoAlarm)
    {
        Serial.println(F("Lämpöalaraja Hälytys!"));
        Serial.println();
        sendStatus(SS_ALARM);
    }
    prev_AlaLampoAlarm = AlaLampoAlarm;

    (adeon.getParamValue("Kotona") == 0) ? digitalWrite(PUDOTUS, PUDOTUS_OFF) : digitalWrite(PUDOTUS, PUDOTUS_ON);
}

void processMsg(){
    Serial.println("\nPROCESSING INCOMMING MSG.");
    //simulation of incomming message 
    if(adeon.isAdeonReady()){
        if(adeon.isUserInAdeon(pnBuf)){
            Serial.println(F("PHONE NUMBER IS AUTHORIZED"));
            //parameters are parsed and their values are saved into list
            adeon.parseBuf(msgBuf, adeon.getUserRightsLevel(pnBuf));
            adeon.printParams();
            sendStatus(SS_NORMAL);
            Serial.print(F("Temperature ")); 
            Serial.print(measureTemp());
            Serial.println(F(" *C"));
        }
        else {
            Serial.println(F("PHONE NUMBER IS NOT AUTHORIZED\n"));
        }
    }
    else{
        Serial.println(F("ADEON IS NOT READY TO PROCCESS NEW MESSAGE."));
    }
}

// Send status message SMS.
// If mode = ALARM, send message to all ADMIN
// IF mode = NORMAL, send status to initiator (last received sms number)
void sendStatus(uint8_t mode)
{
    char* sms = (char*)malloc(SIZEOF_TXT_LPALARM+3);        // Tähän pisin lähetettävä tekstipätkä (viesti kasataan modeemissa)
    uint8_t len = 0, i = 1;

    if (mode == SS_ALARM) i = GetAlarmPhoneNumber(&sms[SIZEOF_SENDSMS], 0);
    // Build up command and phonenumber
    while (i > 0)
    {
        memcpy( &sms[0], sendSms, SIZEOF_SENDSMS );             // AT command begin

        if (mode == SS_ALARM) 
        {
            GetAlarmPhoneNumber(&sms[SIZEOF_SENDSMS], i-1);
        } else {
            memcpy( &sms[SIZEOF_SENDSMS], pnBuf, SIZEOF_PHONENUMBER );
        }

        memcpy( &sms[SIZEOF_SENDSMS+SIZEOF_PHONENUMBER], "\"", 1);
        memcpy( &sms[SIZEOF_SENDSMS+SIZEOF_PHONENUMBER+1], "\0", 1);

        Serial.println(sms);
        gsm.sendSMSNumber(sms);

        // Build up message
        memcpy( &sms[0], txt_lampo, SIZEOF_TXT_LAMPO);
        len = SIZEOF_TXT_LAMPO;
        dtostrf(measureTemp(), 4, 2, &sms[len]);
        len += 4;
        sms[len++] = '*';
        sms[len++] = 'C';
        memcpy(&sms[len], CRLF, 2);
        len += 2;
        sms[len] = '\0';
        gsm.sendSMSMessage(sms);

        // Pudotus
        len = 0;
        if (adeon.getParamValue("Kotona") == 0)
        {
            memcpy(&sms[len], txt_poissa, SIZEOF_TXT_POISSA);
        } else {
            memcpy(&sms[len], txt_kotona, SIZEOF_TXT_KOTONA);
        }
        len += 7;
        memcpy(&sms[len], CRLF, 2);
        len += 2;
        sms[len] = '\0';
        gsm.sendSMSMessage(sms);

        // Hälytyksen tila
        len = 0;
        if (adeon.getParamValue("LP_ALARM") == 0)
        {
            memcpy(&sms[len], txt_LPOK, SIZEOF_TXT_LPOK);
            len += SIZEOF_TXT_LPOK;
        } else {
            memcpy(&sms[len], txt_LPAlarm, SIZEOF_TXT_LPALARM);
            len += SIZEOF_TXT_LPALARM;
        }    
        memcpy(&sms[len], CRLF, 2);
        len += 2;
        sms[len] = '\0';
        gsm.sendSMSMessage(sms);

        // Lämpötilahälytyksen raja-arvo
        len = 0;
        memcpy(&sms[len], txt_lamporaja, SIZEOF_TXT_LAMPORAJA);
        len += SIZEOF_TXT_LAMPORAJA;
        sprintf(&sms[len], "%d", adeon.getParamValue("Lamporaja"));
        gsm.sendLastSMSMessage(sms);
        if (mode == SS_ALARM) delay(20000);
        i--;
    }
    free(sms);
}

// Get Alarm phone number indexed numberIndex to given string, 
// returns 0 if indexed number is not available, else amount of remaining numbers

uint8_t GetAlarmPhoneNumber(char* number, uint8_t numberIndex)
{
    // Reserve room for max 4 users
    char* userlist = (char*)malloc(4*LIST_ITEM_LENGTH+1);
    // make sure memory is empty
    memset(userlist, "\0", LIST_ITEM_LENGTH*4+1);
    uint8_t messages = adeon.GetUsers(userlist,  ADEON_ADMIN);
    
    if (numberIndex < messages)
    {
        memcpy(number, &userlist[numberIndex*LIST_ITEM_LENGTH], SIZEOF_PHONENUMBER);
        free(userlist);
        return messages - numberIndex;
    } else {
        free(userlist);
        return 0;
    }
}

#if 0
// Gives number of
void GetAlarmPhone(char* number, uint8_T last_number)
{
    // Reserve room for max 4 users
    char* userlist = (char*)malloc(4*16+1);
    // make sure memory is empty
    memset(userlist, "\0", 16*4+1);
    uint8_t messages = adeon.GetUsers(userlist,  ADEON_ADMIN);
    char i=0;
    char* sms = (char*)malloc(60);
    
    while (!(i==messages))
    {
        memcpy( &sms[0], sendSms, SIZEOF_SENDSMS );
        memcpy( &sms[SIZEOF_SENDSMS], &userlist[i*16], SIZEOF_PHONENUMBER );
        memcpy( &sms[SIZEOF_SENDSMS+SIZEOF_PHONENUMBER], "\"", 1);
        memcpy( &sms[SIZEOF_SENDSMS+SIZEOF_PHONENUMBER+1], txt_Status, SIZEOF_TXT_STATUS);
        sms[SIZEOF_SENDSMS+SIZEOF_PHONENUMBER+SIZEOF_TXT_STATUS+2] = 26;
        sms[SIZEOF_SENDSMS+SIZEOF_PHONENUMBER+SIZEOF_TXT_STATUS+3] = '\0';
        DebugOut();
        Serial.println(sms);
      //  Serial.println(gsm.sendSMS(sms));

        i++;
        delay(500);
    }
    free(userlist);
    free(sms);
}
#endif

float measureTemp() 
{
    uint16_t SampleSum = 0;
    uint8_t i;
    float average;
    float steinhart;

    // take N samples in a row, with a slight delay
    for (i=0 ; i < NUMSAMPLES ; i++)
    {
        SampleSum += analogRead(THERMISTORPIN);
        delay(10);
    }
    
    // average all the samples out
    average = SampleSum / NUMSAMPLES;
    
    //Serial.print(F("Average analog reading ")); 
    //Serial.println(average);
    
    // convert the value to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;
    //Serial.print(F("Thermistor resistance ")); 
    //Serial.println(average);
    
    steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert absolute temp to C
    
//    Serial.print(F("Temperature ")); 
//    Serial.print(steinhart);
//    Serial.println(F(" *C"));
    return (steinhart);
}

/*  Muuttaa C:n stringin skandit GSM merkistön skandeiksi
    palauttaa uuden merkkijonon pituuden
*/
#if 0
char str2SMS(char* str, int8_t len)
{
    int8_t i=0, j=0;
    while (i < len+1)
    {
        if (str[i] != 303) 
        {
            str[j] = str[i];
        } else {
            ++i;
            if (str[i] == 244) str[j] = 0x7B;           // ä
            if (str[i] == 266) str[j] = 0x7C;           // ö
            if (str[i] == 204) str[j] = 0x5B;           // Ä
            if (str[i] == 226) str[j] = 0x5C;           // Ö
        }
        i++;
        j++;
    }
    return j-1;
}
#endif

void SIM900power()
{
  pinMode(GSM_POWER, OUTPUT); 
  digitalWrite(GSM_POWER, LOW);
  delay(500);
  digitalWrite(GSM_POWER, HIGH);
  delay(2000);
  digitalWrite(GSM_POWER, LOW);
  delay(2000);
}

void DebugOut(void)
{
    digitalWrite(DEBUGOUT, LOW);
    delay(10);
    digitalWrite(DEBUGOUT, HIGH);
}

void setup()
{
    SIM900power();

    // Setup the Serial port. See http://arduino.cc/en/Serial/IfSerial
    Serial.begin(DEFAULT_BAUD_RATE);
    delay(200);

    pinMode(PUDOTUS, OUTPUT);
    pinMode(LP_ALARM, INPUT_PULLUP);
    digitalWrite(PUDOTUS, PUDOTUS_ON);
    
    pinMode(DEBUGOUT, OUTPUT);
    digitalWrite(DEBUGOUT, LOW);

    analogReference(EXTERNAL);

        
    gsm.begin();

    userInit();
    paramInit();
    numOfItems();  
    measureTemp();  
}

void loop()
{
    gsm.checkGsmOutput();
    if(gsm.isNewMsgAvailable()){
        pnBuf = gsm.getPhoneNum();
        Serial.print(F("Phone number: "));
        Serial.println(pnBuf);
        msgBuf = gsm.getMsg();
        Serial.print(F("Message: : "));
        Serial.println(msgBuf);
        processMsg();
    }
    IOControl();
    delay(1000);
    DebugOut();
//    Serial.print(F("Temperature ")); 
 //   Serial.print(measureTemp());
 //   Serial.println(F(" *C"));
#ifdef DEBUG_MEMORY
    Serial.print(F("  "));
    Serial.print(freeMemory());
#endif
}
