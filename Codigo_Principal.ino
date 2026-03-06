/*
Nombre: Taller 1
Fecha: 04/03/2026
Autor: Felipe Arciniegas
Descripción: El código implementa un sistema de cultivo inteligente con ESP32 
que monitorea sensores ambientales y controla automáticamente iluminación, riego 
y alertas, permitiendo además encender o apagar el sistema mediante RFID, mientras 
reporta todas las variables en el monitor serial.
*/

//================================================
// LIBRERÍAS
//================================================

#include <DHT.h>                     // Librería para sensor de temperatura y humedad DHT
#include <DFRobot_LTR390UV.h>        // Librería para sensor UV LTR390
#include <MFRC522v2.h>               // Librería principal RFID
#include <MFRC522DriverSPI.h>        // Comunicación SPI para RFID
#include <MFRC522DriverPinSimple.h>  // Definición del pin SS para RFID
#include <MFRC522Debug.h>            // Funciones de depuración RFID



//================================================
// ETIQUETADO DE PINES
//================================================

#define LED 33   // LED indicador de tanque bajo
#define LED1 25  // LED de potencia (iluminación planta)
#define LED2 13  // LED de potencia (iluminación planta)

#define DHT_PIN 14  // Pin del sensor DHT22

#define TRIGGER 27  // Pin Trigger del sensor ultrasónico
#define ECHO 26     // Pin Echo del sensor ultrasónico

#define MOTOR 12  // Pin que controla la bomba de agua

#define MOIST A0  // Pin analógico sensor humedad de suelo
#define LDR A3    // Pin analógico sensor de luz



//================================================
// CONSTANTES DEL SISTEMA
//================================================

#define DHT_TYPE DHT22  // Tipo de sensor DHT utilizado

#define VALID_CARD "3933397a"  // UID de tarjeta RFID autorizada

#define MIN_HUM 50  // Humedad mínima del suelo para activar riego
#define MAX_HUM 75  // Humedad máxima del suelo para detener riego

#define MAX_DIST 15  // Distancia máxima (cm) para considerar tanque con agua

#define DHT_INTERVAL 2000    // Intervalo de lectura DHT (ms)
#define SONAR_INTERVAL 1000  // Intervalo lectura sensor ultrasónico
#define RFID_INTERVAL 1000   // Intervalo lectura RFID



//================================================
// OBJETOS DE SENSORES
//================================================

DHT dht(DHT_PIN, DHT_TYPE);  // Inicializa objeto del sensor DHT

DFRobot_LTR390UV ltr390(
  LTR390UV_DEVICE_ADDR,
  &Wire);  // Inicializa sensor UV con protocolo I2C

MFRC522DriverPinSimple ss_pin(5);  // Pin SS del módulo RFID

MFRC522DriverSPI driver{ ss_pin };  // Comunicación SPI RFID

MFRC522 mfrc522{ driver };  // Objeto principal lector RFID



//================================================
// VARIABLES DEL SISTEMA
//================================================

float uv = 0;    // Variable para intensidad UV
float temp = 0;  // Temperatura ambiente
float hum = 0;   // Humedad ambiente

uint32_t moist = 0;  // Humedad del suelo
uint32_t ldr = 0;    // Intensidad de luz

float distance = 0;  // Distancia medida por ultrasónico

bool motor = false;  // Estado lógico de la bomba



// Variables RFID

bool hasCard = false;  // Indica si hay tarjeta detectada

String card = "";      // UID tarjeta leída
String tempCard = "";  // UID temporal



// Variables de máquina de estados

int mef1 = 0;  // MEF control acceso RFID
int mef2 = 0;  // MEF control sistema cultivo



// Variables para temporización

unsigned long dhtRead = 0;       // Tiempo última lectura DHT
unsigned long distanceRead = 0;  // Tiempo última lectura ultrasónico



//================================================
// SUBRUTINAS / FUNCIONES
//================================================


//------------------------------------------------
// Función lectura sensor DHT
//------------------------------------------------

void readDHT() {

  // Evita leer el sensor antes del intervalo definido
  if (millis() - dhtRead <= DHT_INTERVAL) return;

  float t = dht.readTemperature();  // Lee temperatura
  float h = dht.readHumidity();     // Lee humedad

  dhtRead = millis();  // Guarda tiempo de lectura

  // Verifica error de lectura
  if (isnan(t) || isnan(h)) {
    Serial.println("Error al leer DHT22");
    return;
  }

  temp = t;  // Actualiza variable temperatura
  hum = h;   // Actualiza variable humedad
}



//------------------------------------------------
// Función lectura sensor ultrasónico
//------------------------------------------------

void readDistance() {

  // Evita lectura antes del intervalo establecido
  if (millis() - distanceRead <= SONAR_INTERVAL) return;

  digitalWrite(TRIGGER, HIGH);  // Activa trigger
  delayMicroseconds(10);        // Pulso de 10 microsegundos
  digitalWrite(TRIGGER, LOW);   // Desactiva trigger

  distanceRead = millis();  // Guarda tiempo de lectura

  // Convierte tiempo de eco a distancia en cm
  distance = pulseIn(ECHO, HIGH) / 58.0;
}



//------------------------------------------------
// Función lectura RFID
//------------------------------------------------

void readCard() {

  // Si no hay tarjeta presente o no se puede leer
  if (
    !mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return;

  tempCard = "";  // Limpia variable temporal

  // Recorre bytes del UID
  for (unsigned int i = 0; i < mfrc522.uid.size; i++) {

    if (mfrc522.uid.uidByte[i] < 0x10)
      tempCard += "0";  // Agrega cero si valor < 16

    tempCard += String(
      mfrc522.uid.uidByte[i],
      HEX);  // Convierte a hexadecimal
  }

  card = tempCard;  // Guarda UID completo
}



//================================================
// SETUP
//================================================

void setup() {

  Serial.begin(115200);  // Inicializa comunicación serial

  dht.begin();  // Inicializa sensor DHT



  // Configuración pines

  pinMode(LED, OUTPUT);      // LED alerta
  pinMode(MOTOR, OUTPUT);    // Bomba
  pinMode(TRIGGER, OUTPUT);  // Trigger ultrasónico
  pinMode(ECHO, INPUT);      // Echo ultrasónico

  digitalWrite(LED, LOW);      // LED apagado
  digitalWrite(MOTOR, LOW);    // Bomba apagada
  digitalWrite(TRIGGER, LOW);  // Trigger en bajo



  // Inicialización sensor UV

  Serial.print("Connecting to UV sensor");

  while (ltr390.begin() != 0) {

    Serial.print(".");  // Espera conexión
    delay(1000);
  }

  Serial.println(" UV Sensor initialize success");

  ltr390.setALSOrUVSMeasRate(
    ltr390.e18bit,
    ltr390.e100ms);  // Configura resolución y tiempo

  ltr390.setALSOrUVSGain(
    ltr390.eGain3);  // Configura ganancia

  ltr390.setMode(
    ltr390.eUVSMode);  // Modo medición UV



  // Inicialización RFID

  mfrc522.PCD_Init();  // Inicializa lector RFID

  MFRC522Debug::PCD_DumpVersionToSerial(
    mfrc522,
    Serial);  // Imprime versión del lector
}



//================================================
// LOOP PRINCIPAL
//================================================

void loop() {

  //================================================
  // COMUNICACIONES (MONITOR SERIAL)
  //================================================

  Serial.println(
    "Estado2 " + String(mef2) + " Estado1 " + String(mef1) + " Card " + card + " Temp " + String(temp) + " Hum " + String(hum) + " Distance " + String(distance) + " Moisture " + String(moist) + " LDR " + String(ldr) + " UV " + String(uv) + " Motor " + String(digitalRead(MOTOR)) + " Led " + String(digitalRead(LED)));



  //================================================
  // MEF 1 — CONTROL DE ACCESO RFID
  //================================================

  switch (mef1) {

    case 0:
      {
        card = "";   // Limpia tarjeta
        readCard();  // Lee tarjeta

        if (card == VALID_CARD)
          mef1 = 1;  // Activa sistema

        break;
      }

    case 1:
      {
        card = "";
        readCard();

        if (card == VALID_CARD)
          mef1 = 0;  // Apaga sistema

        break;
      }
  }



  //================================================
  // MEF 2 — CONTROL DEL CULTIVO
  //================================================

  switch (mef2) {



      //================================================
      // ESTADO 0 — IDLE
      //================================================

    case 0:
      {

        digitalWrite(MOTOR, LOW);  // Bomba apagada
        digitalWrite(LED, LOW);    // LED alerta apagado
        digitalWrite(LED1, LOW);   // LED cultivo apagado
        digitalWrite(LED2, LOW);

        if (mef1 == 1)
          mef2 = 1;  // Pasa a lectura sensores

        break;
      }



      //================================================
      // ESTADO 1 — TOMA DE DATOS
      //================================================

    case 1:
      {

        readDHT();       // Lee temperatura y humedad
        readDistance();  // Lee nivel tanque

        moist = map(
          analogRead(MOIST),
          0,
          4095,
          0,
          100);  // Humedad suelo %

        ldr = map(
          analogRead(LDR),
          0,
          4095,
          0,
          100);  // Nivel luz %

        uv = ltr390.readOriginalData();  // Intensidad UV

        digitalWrite(LED, LOW);
        digitalWrite(MOTOR, LOW);

        analogWrite(
          LED1,
          map(ldr, 0, 100, 255, 0));  // PWM iluminación

        analogWrite(
          LED2,
          map(ldr, 0, 100, 255, 0));

        if (mef1 == 0)
          mef2 = 0;

        else if (moist < MIN_HUM && distance < MAX_DIST)
          mef2 = 3;  // Activar riego

        else if (distance >= MAX_DIST)
          mef2 = 2;  // Tanque vacío

        break;
      }



      //================================================
      // ESTADO 2 — TANQUE VACÍO
      //================================================

    case 2:
      {

        readDHT();
        readDistance();

        moist = map(analogRead(MOIST), 0, 4095, 0, 100);
        ldr = map(analogRead(LDR), 0, 4095, 0, 100);

        uv = ltr390.readOriginalData();

        analogWrite(LED1, map(ldr, 0, 100, 255, 0));
        analogWrite(LED2, map(ldr, 0, 100, 255, 0));

        digitalWrite(LED, HIGH);  // Alerta tanque vacío
        digitalWrite(MOTOR, LOW);

        if (mef1 == 0)
          mef2 = 0;

        else if (moist < MIN_HUM && distance < MAX_DIST)
          mef2 = 3;

        else if (distance < MAX_DIST)
          mef2 = 1;

        break;
      }



      //================================================
      // ESTADO 3 — RIEGO AUTOMÁTICO
      //================================================

    case 3:
      {

        readDHT();
        readDistance();

        moist = map(analogRead(MOIST), 0, 4095, 0, 100);
        ldr = map(analogRead(LDR), 0, 4095, 0, 100);

        uv = ltr390.readOriginalData();

        analogWrite(LED1, map(ldr, 0, 100, 255, 0));
        analogWrite(LED2, map(ldr, 0, 100, 255, 0));

        digitalWrite(LED, LOW);
        digitalWrite(MOTOR, HIGH);  // Activa bomba

        if (mef1 == 0)
          mef2 = 0;

        else if (moist >= MAX_HUM)
          mef2 = 1;  // Detiene riego

        else if (moist < MIN_HUM && distance > MAX_DIST)
          mef2 = 2;  // Tanque vacío

        break;
      }
  }
}