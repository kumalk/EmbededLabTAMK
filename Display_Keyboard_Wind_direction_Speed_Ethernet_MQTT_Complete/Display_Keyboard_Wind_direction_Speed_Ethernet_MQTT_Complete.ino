
//---------------Improting libraries----------------------------------------
// These are external libraries we have used to build this project
#include <LiquidCrystal.h>
#include <Ethernet.h>
#include <TimerOne.h>
#include <PubSubClient.h>

//-------------------Verialbles : Start ---------------------------------------------
//All the global veriables are here 

//Pins to take the measuremnts for wind speed and direction
int windSpeedPin = 3;
int windDirectionPin = A0;

//  veriable to store time values to calculate frequency
volatile long timeStart = 0;
volatile long time1 = 0;
volatile long time2 = 0;
volatile int count = 0;
volatile long DD = 0;
long frequency = 0;

//To store the wind direction voltage value
volatile int ADval = 0;

//To store local IP address


//To calculate average values of speed and direction
int counter = 0;
int ADvalSum = 0;
long freqSum = 0;

int ADvalAvg = 0;
long freqAvg = 0;

int windSpeedVal = 0;
int windDirectionVal = 0;


//To store connected local IP and store sending JSON string via MQTT
IPAddress IP;
char sendData[100];



// MQTT Setup veriables and objects

byte server[] = {10, 6, 0, 23};               // MQTT Server IP Address
unsigned int Port = 1883;                     // MQTT Server Port
EthernetClient ethClient;                     // Ethernet library client object
PubSubClient client(server, Port, ethClient); // Creat PubSubClient Client object

#define outTopic "ICT4_out_2020" //Topic to send message 

static uint8_t mymac[6] = {0x44, 0x76, 0x58, 0x10, 0x00, 0x62}; // MAC Address Ethernet-interface

char *clientId = "a731fsd4"; // MQTT client indentifier


//  Veriables for LCD display
const int rs = 6, en = 7, d4 = A2, d5 = A3, d6 = A4, d7 = A5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);



// Veriables for keypad setup

const int rows[] = {8, 9, 5};  // Use only the first three rows
const int cols[] = {4, A1, 2}; // Use only the first three columns
const char keys[3][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'}};



//==========================Setup================================================

void setup()
{
  Serial.begin(9600); // Starting Serial Monitor
  fetch_IP();         // Fetching IP for local connection
  Timer1.initialize(500000); //Initializing timer for taking samples for 
  Timer1.attachInterrupt(takeMeasures);

  pinMode(windSpeedPin, INPUT); //defining pin

  //LCD screen starting
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);

  //Setting up interupt to measure windspeed using falling signal , attaching pin_ISR_down function
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), pin_ISR_Down, FALLING);

  // Initialize row pins as OUTPUT and set them HIGH
  for (int i = 0; i < 3; i++)
  {
    pinMode(rows[i], OUTPUT);
    digitalWrite(rows[i], HIGH);
  }

  // Initialize column pins as INPUT_PULLUP
  for (int i = 0; i < 3; i++)
  {
    pinMode(cols[i], INPUT_PULLUP);
  }
}

//===========================Loop===============================================
void loop()
{
  
  ADval = analogRead(windDirectionPin);   //Reading voltage value for caluclating wind direction 

 //This part is for calculating frequency by removing noise effect (maybe debounce)
  if (time1 != time2)
  {
    if (time1 - time2 > 0)
    {
      DD = time1 - time2; //calculating time difference between 2 fallings 
      if (DD > 10) //if time difference above 10ms only record the signal unless it avoids by considering as a noise signal
      {
        frequency = 1000 / (time1 - time2); //calculating instananious frequency 
      }
    }
    else
    {
      frequency = 0;
    }

    //This part is updating the display for each 5 seconds
    if ((time1 - timeStart) >= 5000)
    {
      lcd.clear();

      //Calculate average values
      ADvalAvg = ADvalSum / counter; 
      freqAvg = freqSum / counter;

      //calling print functions to print data on LCD display
      printWindSpeed(freqAvg);
      printWindDirection(ADvalAvg);

      //Preparing JSON string with calculated values to send to MQTT server
      sprintf(sendData, "IOTJS={\"S_name1\":\"Embedx_WindSpeed\",\"S_value1\":%d,\"S_name2\":\"Embedx_WindDirection\",\"S_value2\":%d}", windSpeedVal, windDirectionVal);

      send_MQTT_message();                    //Call the send MQTT message function

      //Reseting values of veriables after updating display
      ADvalSum = 0;
      freqSum = 0;
      counter = 0;
      timeStart = time1;
    }

  
    delay(500);//delay for make display consistancy
  }

  
  
  //Reading keypad values 
  for (int row = 0; row < 3; row++)
  {
    digitalWrite(rows[row], LOW); // Set current row LOW

    for (int col = 0; col < 3; col++)
    {
      if (digitalRead(cols[col]) == LOW)
      {
        Serial.println(keys[row][col]); // Print the pressed key
        displayHandler(keys[row][col]);

        while (digitalRead(cols[col]) == LOW)
          ; // Wait for the key to be released
      }
    }

    digitalWrite(rows[row], HIGH); // Reset current row to HIGH
  }
}



//========================Functions==============================================

//------------------------------MQTT functions -------------------------


void fetch_IP()
{
  bool connectionSuccess = Ethernet.begin(mymac); // Yhdistäminen Ethernet-verkkoon ja tallennetaan yhteyden tila
  if (!connectionSuccess)
  {
    Serial.println("Failed to access Ethernet controller"); // Jos yhteys ei onnistunut -> yhteysvirheilmoitus
  }
  else
  {
    Serial.println("Connected with IP: " + Ethernet.localIP()); // Onnistuessa tulostetaan IP-osoite
  }

  // this is custom coded part
  lcd.clear();
  lcd.print("IP:");
  lcd.print(Ethernet.localIP());
  IP = Ethernet.localIP();

  delay(1500);
}


void send_MQTT_message()
{
  if (!client.connected())
  {                        // Tarkistetaan onko yhteys MQTT-brokeriin muodostettu
    connect_MQTT_server(); // Jos yhteyttä ei ollut, kutsutaan yhdistä -funktiota
  }
  if (client.connected())
  {                                                 // Jos yhteys on muodostettu
    client.publish(outTopic, sendData);             // Lähetetään viesti MQTT-brokerille
    Serial.println("Message sent to MQTT server."); // Tulostetaan viesti onnistuneesta lähettämisestä
  }
  else
  {
    Serial.println("Failed to send message: not connected to MQTT server."); // Ei yhteyttä -> Yhteysvirheilmoitus
  }
}

void connect_MQTT_server()
{
  Serial.println("Connecting to MQTT"); // Tulostetaan vähän info-viestiä
  if (client.connect(clientId))
  {                                 // Tarkistetaan saadaanko yhteys MQTT-brokeriin
    Serial.println("Connected OK"); // Yhdistetty onnistuneesti
  }
  else
  {
    Serial.println("Connection failed."); // Yhdistäminen epäonnistui
  }
}

//-----------------------------MQTT functions : End --------------------------------------


//Functions for taking measurements to calculate average values

void takeMeasures()
{
  counter++;
  ADvalSum += analogRead(windDirectionPin);
  freqSum += frequency;
}


//This part is for handling display on pressing keypad buttons

void displayHandler(char pressedKey)
{
  if (pressedKey == '5')
  {
    lcd.clear();
    printWindSpeed(freqAvg);
  }
  else if (pressedKey == '4')
  {
    lcd.clear();
    printWindDirection(ADvalAvg);
  }
  else if (pressedKey == '6')
  {
    lcd.clear();
    printWindDirectionName(ADvalAvg);
  }
}


// functions to attach with interuption 
void pin_ISR_Down()
{
  time2 = time1;
  time1 = millis();
  DD = time1 - time2;
  if ((DD) > 10)
  {
    count++;
  }
}

//Function for printing wind speed values
void printWindSpeed(long freq)
{
  long windSpeed = -0.24 + freq * 0.699;
  windSpeedVal = windSpeed;
  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(windSpeed);
  lcd.print(" m/s");
}


//Function for printing wind direction values 
void printWindDirection(int ADvalue)
{
  float voltVal = (float)ADvalue * (5.00 / 1023.00);
  
  windDirectionVal = voltVal * 360 / 5; //calcualting degree value from volatage value

  lcd.setCursor(0, 1);
  lcd.print("Direc: ");
  lcd.print(windDirectionVal);
  lcd.print((char)223);//for printing degree character 
 
}

//Function for printing wind direction values 
void printWindDirectionName(int ADvalue)
{
  float voltVal = (float)ADvalue * (5.00 / 1023.00);
  String windDirection = "";
  

  //determining direction from volt value 
  if (voltVal <= 1.43)
  {
    windDirection = "N";
  }
  else if (voltVal <= 1.91)
  {
    windDirection = "NE";
  }
  else if (voltVal <= 2.39)
  {
    windDirection = "E";
  }
  else if (voltVal <= 2.87)
  {
    windDirection = "SE";
  }
  else if (voltVal <= 3.34)
  {
    windDirection = "S";
  }
  else if (voltVal <= 3.81)
  {
    windDirection = "EW";
  }
  else if (voltVal <= 4.29)
  {
    windDirection = "W";
  }
  else
  {
  }

  lcd.setCursor(0, 1);
  lcd.print("Direc: ");
  lcd.print(windDirection);
  
 
}
