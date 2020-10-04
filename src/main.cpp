#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <EEPROM.h>

//Asetukset
const bool debug = false;
const int loopInterval = 1000;
const char *ssid = "AndroidAP100";
const char *password = "100100100";
const char *server = "broker.hivemq.com";
String topic = "topic_esp8266_katkotesti";
const uint8_t bufferLength = 50;
const int wirelessAmplitudeLimit = 4; //Mittausalue ilman A0 ylösvetoa 0-10, ylösvedon kanssa 0-20. Sopivat arvot ~ 4 ja 8

WiFiUDP ntpUDP;
WiFiClient wifiClient;

NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 86400000);

unsigned long lastPublish = millis();
unsigned long bootTime = millis();

unsigned char inputState[10];
unsigned char inputStatePrevious[10];

uint8_t eepromIndex = 1;

void callback(char *topic, byte *payload, unsigned int length)
{
    // handle message arrived
}

PubSubClient client(server, 1883, callback, wifiClient);

void EEPROMWritelong(int address, unsigned long value);
unsigned long EEPROMReadlong(long address);
bool updateInputState(int input);
bool updateWirelessInputState(int input);
void connectAndPublish(String payload);
uint8_t nextIndex(uint8_t index);
void printEEPROMdata();
void writeEvent(uint8_t index, unsigned long epoch, byte input, byte state);
void clearEEPROM();
String getEEPROMdata();

long average = 0;
float c, s;

/*
EEPROM data
0-10 xxxx a b 0000
510 = latest index

xxxx epoch timestamp
a channel 0=device, 1=d1, 2=d2...
b event 0=input low, 1=input high, 2=startup
*/
void setup()
{
    EEPROM.begin(512); //Initialize EEPROM
    client.setBufferSize(5000);
    Serial.begin(115200);
    delay(2000);
    pinMode(D1, INPUT_PULLUP);
    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, INPUT_PULLUP);
    pinMode(D4, INPUT_PULLUP);
    pinMode(D5, INPUT_PULLUP);
    pinMode(D6, INPUT_PULLUP);
    pinMode(D7, INPUT_PULLUP);
    pinMode(D8, INPUT_PULLUP);

    printEEPROMdata();

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    timeClient.begin();
    timeClient.update();
    topic.concat("/");
    topic.concat(WiFi.macAddress());

    EEPROM.get(510, eepromIndex);
    if (eepromIndex >= bufferLength)
        eepromIndex = 0;

    //update index
    eepromIndex = nextIndex(eepromIndex);
    EEPROM.put(510, eepromIndex);
    EEPROM.commit();
    //Write startup event to eeprom
    writeEvent(eepromIndex, timeClient.getEpochTime(), 0, 1);
    connectAndPublish(getEEPROMdata());
}

void loop()
{
    updateInputState(1);
    updateInputState(2);
    updateInputState(4);
    updateInputState(5);
    updateInputState(6);
    updateWirelessInputState(10);

    if (~digitalRead(D3) & 1)
    {
        connectAndPublish(getEEPROMdata());
        lastPublish = millis();
    }
    if (~digitalRead(D7) & 1)
    {
        clearEEPROM();
    }
    if (debug)
    {
        if (millis() - lastPublish > 10000)
        {
            connectAndPublish(getEEPROMdata());
            lastPublish = millis();
        }
    }
    delay(loopInterval);
}

void connectAndPublish(String payload)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.forceSleepWake();
        WiFi.begin(ssid, password);
    }
    unsigned long timeoutMillis = millis();
    while (WiFi.status() != WL_CONNECTED && timeoutMillis + 5000 > millis())
    {
        delay(500);
        Serial.print(".");
    }
    if (client.connect("Testi_esp_katko"))
    {
        Serial.println("Connected to MQTT broker");

        if (client.publish(topic.c_str(), getEEPROMdata().c_str()))
        {
            Serial.println("Publish ok");
        }
        else
        {
            Serial.println("Publish failed");
        }
    }
    delay(500);
    if (!debug)
    {
        WiFi.disconnect();
        WiFi.forceSleepBegin(500000000);
    }
}

bool updateInputState(int input)
{
    switch (input)
    {
    case 1:
        inputState[input] = ~digitalRead(D1) & 1;
        break;
    case 2:
        inputState[input] = ~digitalRead(D2) & 1;
        break;
    case 3:
        inputState[input] = ~digitalRead(D3) & 1;
        break;
    case 4:
        inputState[input] = ~digitalRead(D4) & 1;
        break;
    case 5:
        inputState[input] = ~digitalRead(D5) & 1;
        break;
    case 6:
        inputState[input] = ~digitalRead(D6) & 1;
        break;
    case 7:
        inputState[input] = ~digitalRead(D7) & 1;
        if (inputState[7] == 1)
            clearEEPROM();
        break;
    }

    if (inputState[input] != inputStatePrevious[input])
    {
        //Serial.print("Input changed:");
        //Serial.print(inputState[input]);
        inputStatePrevious[input] = inputState[input];

        eepromIndex = nextIndex(eepromIndex);
        EEPROM.write(510, eepromIndex);
        writeEvent(eepromIndex, timeClient.getEpochTime(), input, inputState[input]);
        return true;
    }
    return false;
}

bool updateWirelessInputState(int input)
{
    const int sample = 1000;

    int16_t adc0;
    long sum, t, top;
    double amplitude, cp, sp, phi;
    int i;
    sum = 0;
    i = 0;
    c = 0;
    s = 0;
    while (i < sample)
    {
        adc0 = analogRead(A0) - 1023 * 3 / 5;
        t = micros();
        phi = 6.2831853 * t / 1.0e6 * 50.0; //Replace 50.0 by 60.0 if your mains is 60 Hz
        c += (adc0 - average) * cos(phi);
        s += (adc0 - average) * sin(phi);
        sum += adc0;
        i++;
    }
    average = (sum / sample);
    i = 0;
    cp = c / sample;
    sp = s / sample;
    amplitude = 2 * sqrt(cp * cp + sp * sp);
    amplitude = constrain(amplitude, 0, 20); //tweak it from experience, 10 works for me
    if (debug)
    {
        Serial.printf("Amplitude: %f DC-component:%ld CP:%f SP:%f\n", amplitude, average, cp, sp);
    }

    if (amplitude > wirelessAmplitudeLimit)
    {
        inputState[input] = 1;
    }
    else
    {
        inputState[input] = 0;
    }
    if (inputState[input] != inputStatePrevious[input])
    {
        inputStatePrevious[input] = inputState[input];

        eepromIndex = nextIndex(eepromIndex);
        EEPROM.write(510, eepromIndex);
        writeEvent(eepromIndex, timeClient.getEpochTime(), input, inputState[input]);
        return true;
    }
    return false;
}

void EEPROMWritelong(int address, unsigned long value)
{
    //Decomposition from a long to 4 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte four = (value & 0xFF);
    byte three = ((value >> 8) & 0xFF);
    byte two = ((value >> 16) & 0xFF);
    byte one = ((value >> 24) & 0xFF);

    //Write the 4 bytes into the eeprom memory.
    EEPROM.write(address, four);
    EEPROM.write(address + 1, three);
    EEPROM.write(address + 2, two);
    EEPROM.write(address + 3, one);
}
unsigned long EEPROMReadlong(long address)
{
    //Read the 4 bytes from the eeprom memory.
    unsigned long four = EEPROM.read(address);
    unsigned long three = EEPROM.read(address + 1);
    unsigned long two = EEPROM.read(address + 2);
    unsigned long one = EEPROM.read(address + 3);
    //Return the recomposed long by using bitshift.
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

uint8_t nextIndex(uint8_t index)
{
    Serial.printf("nextIndex: Original index:%d\n", index);
    if (index < bufferLength - 1)
    {
        Serial.print("nextIndex: Index updated: ");
        Serial.println(index + 1);
        return index + 1;
    }
    return 0;
}

void printEEPROMdata()
{
    unsigned long epoch;
    byte input;
    byte state;
    uint8_t currentIndex;
    EEPROM.get(510, currentIndex);

    Serial.printf("printEEPROMdata: Current index: %d\n", currentIndex);

    for (int i = 0; i < bufferLength; i++)
    {
        epoch = EEPROMReadlong(i * 10) * 1L;
        input = EEPROM.read(i * 10 + 4);
        state = EEPROM.read(i * 10 + 5);
        if (epoch > 100000 && epoch < 4000000000)
        {
            Serial.printf("printEEPROMdata: EEPROM index: %d\t Epoch timestamp:%lu \tInput: %d \tState: %d\n", i, epoch, input, state);
        }
        yield();
    }
}

void writeEvent(uint8_t index, unsigned long epoch, byte input, byte state)
{
    EEPROMWritelong(index * 10, epoch);
    EEPROM.write(eepromIndex * 10 + 4, input);
    EEPROM.write(eepromIndex * 10 + 5, state);
    EEPROM.commit();
    //printEEPROMdata();
}

void clearEEPROM()
{
    Serial.printf("EEPROM cleared\n");
    for (int i = 0; i < 512; i++)
    {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
    eepromIndex = 0;
}

String getEEPROMdata()
{
    unsigned long epoch;
    byte input;
    byte state;
    uint8_t currentIndex;
    String msg = "{ \"data\":[";
    char row[50];

    for (int i = 0; i < bufferLength; i++)
    {
        epoch = EEPROMReadlong(i * 10) * 1L;
        input = EEPROM.read(i * 10 + 4);
        state = EEPROM.read(i * 10 + 5);
        if (epoch > 100000 && epoch < 4000000000)
        {
            sprintf(row, "{\"time\":%lu, \"input\":%d, \"state\":%d},", epoch, input, state);
            msg.concat(row);
            if (debug)
            {
                Serial.printf("printEEPROMdata: EEPROM index: %d\t Epoch timestamp:%lu \tInput: %d \tState: %d\n", i, epoch, input, state);
            }
        }
    }
    msg.remove(msg.lastIndexOf(","));
    msg.concat("]}");
    return msg;
}