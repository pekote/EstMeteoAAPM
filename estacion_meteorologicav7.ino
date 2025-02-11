/*********************************************************************************/
/*********************************************************************************/       
/* 2023-12 
 * Ruth Schmit & Victoria Cruz 
 * Pasantias CENPAT - CESIMAR
 * Esc Politécnica 703 Jose Toschke
 * Proyecto Estación Meteorologica
 * 2025-02-11 
 * JGV Hace el promedio de 3 mediciones de viento y las sube aparte de otros ajustes,
 * hace un detach interrupt y la medicion de viento queda en una rutina aparte
 */
/*********************************************************************************/
/*********************************************************************************/       

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ThingSpeak.h"
#include <WiFi.h>
#include <HTTPClient.h>

/*********************************************************************************/
/*********************************************************************************/       

// Definición de pines y constantes
#define RPMsensor 15        // Pin del sensor del anemómetro (ajusta según tu configuración)
#define VELETA_PIN 32       // Pin de la veleta
#define BME_SDA 4            // Pin SDA para comunicación I2C con el sensor BME280
#define BME_SCL 5            // Pin SCL para comunicación I2C con el sensor BME280
#define BME_ADDR 0x76        // Dirección I2C del sensor BME280

// Configura tus credenciales de red WiFi

//const char* ssid = "m9jgv";
//const char* password = "pekote05";
const char* ssid = "PVCasa";
const char* password = "pekote05";
//const char* ssid = "OCTOPUS";
//const char* password = "patagonico";
//const char* ssid = "APPM-WiFI";
//const char* password = "AlmiranteStorni";
//const char* ssid = "ClubAeromodelismo";
//const char* password = "vuelaalto";



// Instancia de cliente Wi-Fi
WiFiClient client;

// Variables para ThingSpeak
unsigned long myChannelNumber = 1;
const char* myWriteAPIKey = "XSA81ITTXJ5JKEQD";
const char* myReadAPIKey = "5J0IATQNI48GEMQA";                            //API KEY THINGSPEAK
unsigned long channelID = 2326126;
unsigned int Field6 = 6;                             //Field to hold first constant 
unsigned int Field7 = 7;                             //Field to hold second constant
unsigned int Field8 = 8;                             //Field to hold third constant

float CteCalib1= 1.0;                          //Campo7 de Teamspeak Multiplicador
float CteCalib2=0;                             //Campo8 de Teamspeak Offset
float OnOff=0;                                 //Campo6 de Teamspeak Apagar=1 Encender=0 subida de datos
float data=0;

float temperature, pressure, humidity, WindDirection;
                                    
// Variables para medición del anemómetro
uint32_t Wind_rpm_meas_interval = 3600;
volatile uint32_t RPMTops = 0;
int RPM, RPM1, RPM2, RPM3, VeletaValue;
unsigned long currentMillis, previousMillis;
unsigned long interval = 30000;
static unsigned long ContactTime;
int timeout_counter = 0;
int timeout_counterThing = 0;

int contador, Decena, Unidad, Decimal, Decimal2;

const int LedPin = 2;    

/*********************************************************************************/
/*********************************************************************************/       
// Función de interrupción para medir RPM
void IRAM_ATTR rpm() {
  digitalWrite(LedPin, !digitalRead(LedPin));
  if ((millis() - ContactTime) > 15) {
    RPMTops=RPMTops+1;
    digitalWrite(LedPin, !digitalRead(LedPin));
    ContactTime = millis();
  }
}
/*********************************************************************************/
/*********************************************************************************/       

// Instancia del sensor BME280
Adafruit_BME280 bme;

/*********************************************************************************/
/*********************************************************************************/       

void setup() {
  Serial.begin(115200);
  pinMode(LedPin, OUTPUT);

  // Inicialización de la comunicación I2C
  Wire.begin(BME_SDA, BME_SCL);

  // Inicialización del sensor BME280
  bme.begin(BME_ADDR);

  // Configuración del pin del anemómetro para la interrupción
  pinMode(RPMsensor, INPUT_PULLUP);

  // Configuración del pin de la veleta y resolución de lectura analógica
  pinMode(VELETA_PIN, INPUT);
  analogReadResolution(12);
  BlinkLed(2, 20, 10);
  ReadBME280();

  // Conexión a la red Wi-Fi
  connectToWiFi();

  // Inicialización de ThingSpeak
  ThingSpeak.begin(client);
  
  currentMillis = millis();
  previousMillis = currentMillis; 

}

/*********************************************************************************/
/*********************************************************************************/       

void loop() {

  BlinkLed(2, 40, 10);
  OnOff = readTSData(channelID, Field6 );
  Serial.print("Field6= ");
  Serial.println(OnOff);
  CteCalib1 = readTSData(channelID, Field7 );
  Serial.print("Field7= ");
  Serial.println(CteCalib1);
  CteCalib2 = readTSData(channelID, Field8 );
  Serial.print("Field8= ");
  Serial.println(CteCalib2);
      
  // digitalWrite(LedPin, digitalRead(15));
  // Lectura del valor analógico de la veleta y cálculo de la dirección del viento
  VeletaValue = analogRead(VELETA_PIN);
  WindDirection = map(VeletaValue, 0, 4095, 0, 359);
  Serial.print("Veleta=");
  Serial.println(WindDirection);
  digitalWrite(LedPin, LOW);

  ReadViento();
  RPM1=RPM;
  ReadViento();
  RPM2=RPM;
  ReadViento();
  RPM3=RPM;
  RPM = (RPM1+RPM2+RPM3)/3;
  
  // Envío de datos por la comunicación serial
  Serial.print("Dirección del Viento: ");
  Serial.print(WindDirection);
  Serial.print(" grados\n");
  Serial.print("Velocidad del Viento Final PROMEDIO: ");
  Serial.print(RPM);
  Serial.print(" Km/h\n");
  
  ReadBME280();
 
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
  ThingSpeak.setField(3, pressure);
  ThingSpeak.setField(4, RPM);
  ThingSpeak.setField(5, WindDirection);


  if(OnOff != 0){                                                              // Solo sube dato a publico si está dentro de un rango de presion valido y aparte si esta habilitado
    int response = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);     // Configuración de campos con los valores para ThingSpeak
    if (response == 200) {
      Serial.println("Actualización del canal ThingSpeak exitosa.");
      timeout_counterThing=0;
  } else {
    timeout_counterThing++;
    Serial.println("Problema al actualizar el canal ThingSpeak. Código de error HTTP: " + String(response));
    if(timeout_counter >= 5){
        ESP.restart();
      } 
    }
  }
  if(OnOff = 0.0){
    Serial.println("Canal ThingSpeak ONOFF DESHABILITADO");
    } 
 
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {                 // if WiFi is down, try reconnecting
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }

  if(temperature>100){
    Serial.println("RESET POR FALLA BME280");
    delay(60000);                               // 1 minutos de espera
    ESP.restart();
    }
  
  if(pressure<100){
    Serial.println("RESET POR FALLA BME280");
    delay(60000);                               // 1 minutos de espera
    ESP.restart();
    }

  if (CteCalib1<0.0){                           // Resetea el micro si se envia un valor negativo en FIELD7 < 0 esto es para resetear el micro via web
    Serial.print("CteCalib1= ");
    Serial.println(CteCalib1);
    Serial.println("RESET POR FIELD7");
    delay(60000);                               // 1 minutos de espera
    ESP.restart();
    }
  Serial.println("");
  Serial.println("Yendo a dormir....");
  BlinkLed(2, 1, 200);
  delay(60000);  // 3 minutos de espera
  BlinkLed(2, 2, 200);
  delay(60000);  // 3 minutos de espera
  BlinkLed(2, 3, 200);
  delay(60000);  // 3 minutos de espera

}


/*********************************************************************************/
/*********************************************************************************/       

// Función para conectar a la red Wi-Fi
void connectToWiFi() {
  Serial.print("Conectando WiFi= ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());
    BlinkLed(2, 2, 10);
    timeout_counter++;
    if(timeout_counter >= 1500){
        ESP.restart();
    }
  }
  Serial.println("Conectado a WiFi");
    
/*
  // if WiFi is down, try reconnecting
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
*/
 
  Serial.println("Conectado a WiFi!!!!!!!!!");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI()); 
}
/*********************************************************************************/
void BlinkLed(int pin,int nr, int wait) // number of blinks (pin del led, numero de veces, tiempo encendido/apagado)
{
  for (contador=0; contador< nr; contador++)
  {
  digitalWrite(pin, HIGH);
  delay(wait); 
  digitalWrite(pin, LOW); 
  delay(wait);
  }
  if (nr=0 )
  {
  digitalWrite(pin, HIGH);
  delay(wait/3); 
  digitalWrite(pin, LOW); 
  delay(wait/3);
  digitalWrite(pin, HIGH);
  delay(wait/3); 
  digitalWrite(pin, LOW); 
  delay(wait/3);
  digitalWrite(pin, HIGH);
  delay(wait/3); 
  digitalWrite(pin, LOW); 
  delay(wait/3);
  }
}

/*********************************************************************************/
void numtTxLed (float numero)
{
  Decena = numero/10;
  Unidad = numero-(Decena*10);
  Decimal = ((numero*10) - Decena*100 - Unidad*10);
  Decimal2 = ((numero*100) - Decena*1000 - Unidad*100 - Decimal*10);
  Serial.println(Decena);
  BlinkLed(LedPin,Decena,300);
  delay(1000);
  Serial.println(Unidad);
  BlinkLed(LedPin,Unidad,300);
  delay(1000);
  Serial.println(Decimal);
  BlinkLed(LedPin,Decimal,300);
  delay(1000);
  Serial.println(Decimal2);
  BlinkLed(LedPin,Decimal2,300); 
  delay(3000);
}
/*********************************************************************************/

float readTSData( long TSChannel,unsigned int TSField ){
  data = 5.0;
  while (data == 5.0) {
  data =  ThingSpeak.readFloatField( TSChannel, TSField, myReadAPIKey );
  Serial.println( "Data read from ThingSpeak: " + String( data, 9 ) );
  }
  return data;
}
/*********************************************************************************/
  // Lecturas del sensor BME280
void ReadBME280() {
  temperature = bme.readTemperature();
  Serial.print("Temp=");
  Serial.println(temperature);
  pressure = bme.readPressure() / 100.0F;
  Serial.print("Pres=");
  Serial.println(pressure);
  humidity = bme.readHumidity();
  Serial.print("Hum=");
  Serial.println(humidity);
}
/*********************************************************************************/
  // Lecturas del sensor Viento
void ReadViento() {
  attachInterrupt(digitalPinToInterrupt(RPMsensor), rpm, FALLING);
  RPMTops = 0;                                                            // Reinicio del contador de RPM
  Serial.println("Espera y cuenta pulsos veleta en RPMTOPS");
  delay(Wind_rpm_meas_interval);                                          // Retardo antes de la próxima medición del anemómetro y envío de datos
  Serial.print("RPMTOPS: ");
  Serial.println(RPMTops);
  Serial.println(millis());
  Serial.println(ContactTime);
  digitalWrite(LedPin, LOW);
  // Almacenamiento de las RPM en la variable RPM
  if ((RPMTops >= 0) and (RPMTops <= 21)) RPM = RPMTops * 1.2;
  if ((RPMTops > 21) and (RPMTops <= 45)) RPM = RPMTops * 1.15;
  if ((RPMTops > 45) and (RPMTops <= 90)) RPM = RPMTops * 1.1;
  if ((RPMTops > 90) and (RPMTops <= 156)) RPM = RPMTops * 1.0;
  if ((RPMTops > 156) and (RPMTops <= 999)) RPM = RPMTops * 1.0;
  Serial.print("RPM: ");
  Serial.println(RPM);
  Serial.print("CteCalib1 ");
  Serial.println(CteCalib1);
  RPM = RPM * CteCalib1;
  Serial.print("RPM Ajustado: ");
  Serial.println(RPM);
  detachInterrupt(digitalPinToInterrupt(RPMsensor));
  }

  
