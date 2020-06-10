#include <SPI.h>
#include <SD.h>
#include "Arduino.h"
#include "Adafruit_VEML7700.h"
#include "RTClib.h"
#include "TinyGPS++.h"
#include <Wire.h>
#include <esp32-hal-i2c.h>
#include <WiFi.h>
#include <esp_system.h>
#include <HTTPClient.h>

#define DEBUG false
#define cardSelect 33

const char* ssid = "*******";
const char* password = "*******";


bool loggedIn = false;
const char* host = "****";

const int port = 000;
char buff[128];
char mac_addr[25];

const char* custom_name = "ESP32 Luxmeter";
Adafruit_VEML7700 veml = Adafruit_VEML7700();
RTC_PCF8523 rtc;
File logfile;
TinyGPSPlus gps;
uint32_t last_millis = millis();
WiFiClient client;

bool inicializar_tarjetaSD(){
  if (!SD.begin(cardSelect)) {
    return false;
    Serial.println("SD NO FUNCIONA O NO SE ENCUENTRA");
  }
  return true;
  Serial.println("SD OK");
}

int queryUrl(char* url){
    HTTPClient http;
    http.begin(url);  //Specify destination for HTTP request
    int httpResponseCode = http.GET();   //Send the actual POST request
    if(httpResponseCode>0){
        String response = http.getString();                       //Get the response to the request
        Serial.print("Response Code: ");
        Serial.print(httpResponseCode);   //Print return code
        Serial.print(": ");
        Serial.println(response);           //Print request answer
    }else{
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
    }
    http.end();  //Free resources
    return httpResponseCode;
}

bool inicializar_veml7700(){
  if (!veml.begin()) {
    return false;
    Serial.println("VEML7700 NO FUNCIONA O NO SE ENCUENTRA");
  }
  return true;
  Serial.println("VEML7700 OK");
  veml.setGain(VEML7700_GAIN_1_8);
  veml.setIntegrationTime(VEML7700_IT_25MS);
  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);
}

bool inicializar_rtc(){
  if (! rtc.begin()) {
    Serial.println("No se pudo encontrar RTC");
    return false;
  }
  if (! rtc.initialized()) {
  Serial.println("¡RTC no está funcionando!");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  return false;
  }
  return true;
}

String nombre_archivo(){
  DateTime now = rtc.now();
  int ano = now.year();
  int mes = now.month();
  int dia = now.day();
  int hora = now.hour();
  int min = now.minute();
  String ano1 = String(ano, DEC);
  String mes1 = String(mes, DEC);
  if(mes<=9){  mes1 = '0'+ String(mes, DEC);}
  String dia1 = String(dia, DEC);
  if(dia<=9){  dia1 = '0'+ String(dia, DEC);}
  String hora1 = String(hora, DEC);
  if(hora<=9){  hora1 = '0'+ String(hora, DEC);}
  String min1 = String(min, DEC);
  if(min<=9){  min1 = '0'+ String(min, DEC);}
  String archivo = '/' + ano1 + mes1 + dia1 + '_' + hora1 + min1 + ".TXT";
  //sprintf(archivo, "%02d%02d%02d.csv", ano1, mes1, dia1 );
  return archivo;
}

void escribir_datos_serial(){
  DateTime now = rtc.now();
  float lux = veml.readLux();
  float white = veml.readWhite();
  float latitud = gps.location.lat();
  float longitud = gps.location.lng();
  float altitud = gps.altitude.meters();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  if(now.month()<=9){Serial.print(0);}
  Serial.print(now.month(), DEC);
  Serial.print('/');
  if(now.day()<=9){Serial.print(0);}
  Serial.print(now.day(), DEC);
  Serial.print("; ");
  if(now.hour()<=9){Serial.print(0);}
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if(now.minute()<=9){Serial.print(0);}
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if(now.second()<=9){Serial.print(0);}
  Serial.print(now.second(), DEC);
  Serial.print("; ");  Serial.print(latitud, 6);
  Serial.print(","); Serial.print(longitud, 6);
  Serial.print("; ");  Serial.print(altitud, 2);
  Serial.print("; "); Serial.print(lux, 2);
  Serial.print("; "); Serial.println(white, 2);
}

void escribir_datos_archivo(){
  DateTime now = rtc.now();
  float lux = veml.readLux();
  float white = veml.readWhite();
  float latitud = gps.location.lat();
  float longitud = gps.location.lng();
  float altitud = gps.altitude.meters();
  logfile.print(now.year(), DEC);
  logfile.print('/');
  if(now.month()<=9){logfile.print(0);}
  logfile.print(now.month(), DEC);
  logfile.print('/');
  if(now.day()<=9){logfile.print(0);}
  logfile.print(now.day(), DEC);
  logfile.print("; ");
  if(now.hour()<=9){logfile.print(0);}
  logfile.print(now.hour(), DEC);
  logfile.print(':');
  if(now.minute()<=9){logfile.print(0);}
  logfile.print(now.minute(), DEC);
  logfile.print(':');
  if(now.second()<=9){logfile.print(0);}
  logfile.print(now.second(), DEC);
  logfile.print("; "); logfile.print(latitud, 6);
  logfile.print(","); logfile.print(longitud, 6);
  logfile.print("; "); logfile.print(altitud);
  logfile.print("; "); logfile.print(lux);
  logfile.print("; "); logfile.println(white);
}

void inicializar_WiFi(){
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) { //comprobando conexión
    delay(1000);
    Serial.println("Conectando al WiFi..");
  }
  Serial.println("Conectado a red WiFi");
  Serial.println(WiFi.localIP());
}

void leer_escribir_mac(){
  byte mac[6];
  WiFi.macAddress(mac);
  snprintf(mac_addr, sizeof(mac_addr), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("MAC: ");
  Serial.println(mac_addr);
}

bool iniciar_conexion_server(){
  // Devuelve -1 si no se puede establecer la conexion con el servidor
  // Devuelve 1 si la conexion se hizo correctamente
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return false;
  }
  return true;
}

float voltaje_bateria(){
  /*
  El valor ADC es un número de 12 bits, por lo que el valor máximo es 4095 (contando desde 0).
  Para convertir el valor entero de ADC a un voltaje real, se tendrá que dividir por el valor máximo de 4095,
  luego se duplica, ya que Adafruit reduce a la mitad el voltaje,
  posteriormente se multiplica por el voltaje de referencia del ESP32 que es 3.3V
  y finalmente se multiplica por el Voltaje de Referencia ADC de 1100mV
  */
  float ADCValor=0.0;
  for (uint8_t idx=1; idx<=20; idx++){ // Se han promediado 20 medidas para eliminar el ruido aleatorio del ADC
    ADCValor += analogRead(A13);
  }
  ADCValor = (ADCValor/20.0)*2*3.3*1.1/4095; //se machaca ADCValor, pero es el Voltaje
  return ADCValor;
}

uint8_t nivel_bateria(float VBAT){
  uint8_t porcentaje = 0;
  if (VBAT >= 4.1){ porcentaje = 100;}
  else if (VBAT >4.0){ porcentaje = 90;}
  else if (VBAT >3.9){ porcentaje = 80;}
  else if (VBAT >3.8){ porcentaje = 75;}
  else if (VBAT >3.7){ porcentaje = 60;}
  else if (VBAT >3.6){ porcentaje = 50;}
  else if (VBAT >3.5){ porcentaje = 40;}
  else if (VBAT >3.4){ porcentaje = 20;}
  else if (VBAT >3.3){ porcentaje = 5;}
  else if (VBAT >3.2){ porcentaje = 0;}
  return porcentaje;
}

void setup() {
  while (!Serial) { delay(10); }
  Serial.begin(9600);
  Serial1.begin(9600); //Esté o no conectado el GPS siempre se va a iniciar el Serial 1
  inicializar_veml7700(); // El luxómetro siempre va a estar conectado
  if (inicializar_tarjetaSD() && inicializar_rtc()){ // Si se encuentra conectado el Adalogging se inicia el rtc y se crea el archivo con la cabecera
    String filename = nombre_archivo();
    logfile = SD.open(filename, FILE_WRITE);
    logfile.println("Fecha;      Hora;     GPS;                 ALT;   Lx;    wLx");
  }
  inicializar_WiFi(); 
  leer_escribir_mac();
}

//Variables booleanas para en el caso de que el Adalogging no esté conectado,
//sólo muestre el error una vez
bool printed_file_error = false;
bool printed_write_error = false;

// Variables del GPS por si no está conectado
float latitud = 0;
float longitud = 0;
float altitud = 0;

void loop() {

  while(Serial1.available()) { gps.encode(Serial1.read()); } //Si el Serial 1 del GPS está disponible, se empieza a leer

  if(gps.location.isValid()){ //Si el GPS está conectado y proporciona LaT, LONG, LAT
    latitud = gps.location.lat();
    longitud = gps.location.lng();
    altitud = gps.altitude.meters();
  }else{
    latitud = 0;
    longitud = 0;
    altitud = 0;
  }

  if(!logfile) { // Si no se ha podido abrir el archivo
    if(!printed_file_error)
    {
      Serial.println("ERROR abriendo el archivo");
      printed_file_error = true; // Para solo escribir el error una vez
      return;
    } 
  }
  if (logfile) {  //Si se ha podido abrir el archivo
    if((millis() - last_millis) > 1000) {
      last_millis = millis();
      if (gps.location.isValid()){ // Si el GPS está estable, es decir, posee LAT, LONG, ALT 
        // Solo se escribe en la SD si se posee LAT, LONG, ALT
        escribir_datos_archivo();
        escribir_datos_serial();
      }
    }
    logfile.flush(); // Para que vaya guardando los datos uno detrás de otro
  } else {
    if(!printed_write_error)
    {
      Serial.println("No se ha podido escribir");
      printed_write_error = true; // Para solo escribir el error una vez
      return;
    }
    
  }
  float VBAT = voltaje_bateria();
  uint8_t porcentaje = nivel_bateria(VBAT);

  if(iniciar_conexion_server()){ // Si se ha realizado la conexión con el servidor se enviarán los datos
    Serial.println();
    snprintf(buff, sizeof(buff), "<E32>#%s#%s#GPSlat=%f#GPSlng=%f#ALT=%f#Lx=%.2f#WLx=%.2f#BAT=%d#V=%.2f#",
            mac_addr, custom_name, latitud, longitud, altitud, veml.readLux(), veml.readWhite(), porcentaje, VBAT);
    client.print(buff);
    Serial.println(buff);
    client.stop();
  }else{
    Serial.println("Fallo al conectar con el servidor");
  }
  delay(1000);
  Serial.print("Estado wifi: "); // Para tener un control del WiFi y ver qué valor proporciona cuando falla
  Serial.println(WiFi.status()); // https://www.arduino.cc/en/Reference/WiFiStatus  
}
