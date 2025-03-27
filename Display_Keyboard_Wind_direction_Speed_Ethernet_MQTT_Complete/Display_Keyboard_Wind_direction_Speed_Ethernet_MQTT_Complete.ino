#include <LiquidCrystal.h>
#include <Ethernet.h>
#include <TimerOne.h>
#include <PubSubClient.h>

//-------------------Verialbles---------------------------------------------
const int buttonPin = 3;
volatile long timeStart = 0;
volatile long time1 = 0;
volatile long time2 = 0;
volatile int count = 0;
volatile long DD=0;
IPAddress IP ;
long frequency = 0;
int ADval = 0;

int counter = 0;
int ADvalSum =0;
long freqSum = 0;

int ADvalAvg =0;
long freqAvg=0;

//------------------MQTT Prart : Star---------------------------------------
byte server[] = { 10,6,1,14 }; // MQTT-palvelimen IP-osoite
unsigned int Port = 1883;  // MQTT-palvelimen portti
EthernetClient ethClient; // Ethernet-kirjaston client-olio
PubSubClient client(server, Port, ethClient); // PubSubClient-olion luominen

#define outTopic   "ICT4_out_2020" // Aihe, jolle viesti lähetetään

static uint8_t mymac[6] = { 0x44,0x76,0x58,0x10,0x00,0x62 }; // MAC-osoite Ethernet-liitäntää varten

char* clientId = "a731fsd4"; // MQTT-clientin tunniste

//--------------------MQTT Prart : End-----------------------------------------------


//--------------------Display part : Start-------------------------------------------

const int rs = 6, en = 7, d4 = A2, d5 = A3, d6 = A4, d7 = A5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//--------------------Display part : end----------------------------------------------------

//--------------------Keypad part :start-----------------------------------

const int rows[] = {8, 9, 5};  // Use only the first three rows
const int cols[] = {4, A1, 2};  // Use only the first three columns
const char keys[3][3] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'}
};

//--------------------Keypad part :end-------------------------------------


//==========================Setup================================================

void setup() {
    Serial.begin(9600); // Sarjaportin alustaminen
    fetch_IP(); // Kutsutaan IP-osoitteen haku-funktiota
    
    Timer1.initialize(500000);
    Timer1.attachInterrupt(takeMeasures);
    
    pinMode(buttonPin, INPUT);
    
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    attachInterrupt(digitalPinToInterrupt(buttonPin), pin_ISR_Down, FALLING);
    
    // Initialize row pins as OUTPUT and set them HIGH
  for (int i = 0; i < 3; i++) {
    pinMode(rows[i], OUTPUT);
    digitalWrite(rows[i], HIGH);
  }

  // Initialize column pins as INPUT_PULLUP
  for (int i = 0; i < 3; i++) {
    pinMode(cols[i], INPUT_PULLUP);
  }
}





//===========================Loop===============================================
void loop() {
    send_MQTT_message(); // Kutsutaan MQTT-viestin lähettämis-funktiota
    //delay(5000); // 5 sekunnin viive
    ADval = analogRead(A0);
  Serial.print(ADval);
  if (time1 != time2) { 
    if (time1 - time2 > 0) { 
      DD = time1-time2;
      if(DD>10){
      frequency = 1000 / (time1 - time2);
      Serial.print("  |  ");
      Serial.println(frequency);
      }
    } else {
      frequency = 0; 
    }

    if((time1 - timeStart)>=5000){
      lcd.clear();
      ADvalAvg = ADvalSum/counter;
      freqAvg = freqSum/counter;
      ADvalSum =0;
      freqSum=0;
      counter=0;
      timeStart = time1;
    }

    
    
    
    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.print(" | AD val: ");
    Serial.println(ADval);
    delay(500);
    
     
  }

  for (int row = 0; row < 3; row++) {
    digitalWrite(rows[row], LOW); // Set current row LOW

    for (int col = 0; col < 3; col++) {
      if (digitalRead(cols[col]) == LOW) {
        Serial.println(keys[row][col]); // Print the pressed key
        displayHandler(keys[row][col]);
        
        while (digitalRead(cols[col]) == LOW); // Wait for the key to be released
      }
    }

    digitalWrite(rows[row], HIGH); // Reset current row to HIGH
  }
  
    
}





//========================Functions==============================================

//------------------------------MQTT functions : start-------------------------
void fetch_IP() {
    bool connectionSuccess = Ethernet.begin(mymac); // Yhdistäminen Ethernet-verkkoon ja tallennetaan yhteyden tila
    if (!connectionSuccess) {
        Serial.println("Failed to access Ethernet controller"); // Jos yhteys ei onnistunut -> yhteysvirheilmoitus
    } else {
        Serial.println("Connected with IP: " + Ethernet.localIP()); // Onnistuessa tulostetaan IP-osoite
    }

//this is custom coded part
lcd.clear();
  lcd.print("IP:");
  lcd.print(Ethernet.localIP());
  IP = Ethernet.localIP();
  
  delay(1500);
    
}

void send_MQTT_message() {
    if (!client.connected()) { // Tarkistetaan onko yhteys MQTT-brokeriin muodostettu
        connect_MQTT_server(); // Jos yhteyttä ei ollut, kutsutaan yhdistä -funktiota
    }
    if (client.connected()) { // Jos yhteys on muodostettu
        client.publish(outTopic, "EmbedX Connected!"); // Lähetetään viesti MQTT-brokerille
        Serial.println("Message sent to MQTT server."); // Tulostetaan viesti onnistuneesta lähettämisestä
    } else {
        Serial.println("Failed to send message: not connected to MQTT server."); // Ei yhteyttä -> Yhteysvirheilmoitus
    }
}

void connect_MQTT_server() { 
    Serial.println("Connecting to MQTT"); // Tulostetaan vähän info-viestiä
    if (client.connect(clientId)) { // Tarkistetaan saadaanko yhteys MQTT-brokeriin
        Serial.println("Connected OK"); // Yhdistetty onnistuneesti
    } else {
        Serial.println("Connection failed."); // Yhdistäminen epäonnistui
    }    
}

//-----------------------------MQTT functions : End --------------------------------------


// ---------------------------Other Functions : Start-------------------------------------

void takeMeasures(){
  counter++;
  ADvalSum +=ADval;
  freqSum += frequency;
}

void displayHandler(char pressedKey){
  if(pressedKey=='5'){
    lcd.clear();
    printWindSpeed(freqAvg);
  }else if(pressedKey=='4'){
    lcd.clear();
    printWindDirection(ADvalAvg);
  }else if(pressedKey=='6'){
    lcd.clear();
    lcd.print("IP : ");
    lcd.print(IP);
  }
}


void pin_ISR_Down() {
  time2 = time1;
  time1 = millis();
  DD = time1-time2;
  if((DD)>10){
  count++;
  }
}

void printWindSpeed(long freq){
  long windSpeed = -0.24+freq*0.699;
  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(windSpeed);
  lcd.print(" m/s");
}


void printWindDirection(int ADvalue){
  float voltVal = (float)ADvalue * (5.00 / 1023.00);
  String windDirection = "";
   if (voltVal <= 1.43){
    windDirection = "N";
  }else if (voltVal <= 1.91){
    windDirection = "NE";
  } else if (voltVal <= 2.39) {
    windDirection = "E";
  } else if (voltVal <= 2.87) { 
    windDirection = "SE";
  } else if (voltVal <= 3.34) { 
    windDirection = "S";
  } else if (voltVal <= 3.81) { 
    windDirection = "EW";
  } else if (voltVal <= 4.29) { 
    windDirection = "W";
  } else {
    windDirection = "NW";
  }
  
  lcd.setCursor(0, 1);
  lcd.print("Direc: ");
  //lcd.print(voltVal);
  lcd.print(windDirection);
  Serial.print("voltVal:");
  Serial.print(voltVal);
  Serial.print("| Direction:");
  Serial.println(windDirection);
  
 
}
