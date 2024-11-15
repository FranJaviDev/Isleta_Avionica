/**************************************************************************************************

                                DATOS DE NAVEGACIÓN Y AVIÓNICA.


/**************************************************************************************************/

//#define DEBUG  // Comenta esta línea para deshabilitar la depuración
// #define DEBUG  // Descomenta esta línea para habilitar la depuración
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#include <Wire.h>

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>

#include <math.h>         // Para las funciones trigonométricas

//#include "uNavINS.h"  // Filtro Kalaman.
#include "uNavAHRS.h"

#include <ubx.h>      // GPS

#include <ModbusSlave.h>

#include "Datos_isleta_avionica.h"

#include "DatosAvionica.h"

// Define manualmente los UARTs
HardwareSerial mySerial1(USART1);
HardwareSerial mySerial2(USART2);
HardwareSerial mySerial3(USART6); // Para el STM32F411CE, que no tiene un USART3

// Acelerómetros y magnetómetros.
Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;

// Sensor de presión atmosférica BMP280
Adafruit_BMP3XX bmp; // I2C
// Configuración de presión a nivel del mar (en hPa)
const float QNE = 1013.25;
const float PiesMetro = 3.28084;

// GPS
//UBLOX gps(4);
bool newGpsData;
unsigned long prevTOW;
//gpsData uBloxData;

//uNavINS KalmanFilter;
uNavAHRS Filter;
// timers to measure performance
unsigned long tstart, tstop;

Modbus slave;


// Gestor de tareas simple.
unsigned long lastMillis = 0;
unsigned long lastTask1ms = 0;
unsigned long lastTask10ms = 0;
unsigned long lastTask100ms = 0;
unsigned long lastTask1s = 0;
unsigned long lastTask1min = 0;

// Definir las variables a registrar
int16_t var1 = 100;
int16_t var2 = 200;
int16_t var3 = 300;
int16_t var4 = 1500;

// Array para almacenar los registros de 16 bits de Modbus
int16_t modbusRegisters[8];  // 2 registros por cada float (4 floats * 2 = 8 registros)

void setup() {
  
  Serial.begin(9600);         // UART0 para la depuración.
  
  // Configuración de pines para USART1 en caso de ser necesario
  mySerial1.begin(19200, SERIAL_8N1);          // UART1 para la comunicación Modbus (PA9, PA10);

  slave.begin(1, mySerial1);   // Configura el esclavo con ID 1

  mySerial2.begin(9600);          // UART2 para el GPS.

  // Inicia la comunicación I2C
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(10000); 

  // Inicializa sensores.
  inicializa_sensores_mems();
  inicializa_barometro();

  // Led externo.
  pinMode(PC13, OUTPUT);

};

void inicializa_barometro(){
  // Iniciar el sensor BMP280
  if (!bmp.begin_I2C()){
    while (1);
  }
  // Configuración del sensor
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void inicializa_sensores_mems(){    

  DEBUG_PRINTLN("Iniciando sensores...");
  // Inicializar LSM6DSOX
  if (!lsm6dsox.begin_I2C()) { // Usa I2C
    DEBUG_PRINTLN("No se encontró LSM6DSOX. Verifica la conexión!");
    while (1)
      delay(10);
    }
  DEBUG_PRINTLN("LSM6DSOX encontrado!");

  // Configurar LSM6DSOX
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // Inicializar LIS3MDL
  if (!lis3mdl.begin_I2C()) { // Usa I2C
    DEBUG_PRINTLN("No se encontró LIS3MDL. Verifica la conexión!");
    while (1)
      delay(10);
  }
  DEBUG_PRINTLN("LIS3MDL encontrado!");

  // Configurar LIS3MDL
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);  // Configura el rango
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);  // Configura el modo continuo
 
  DEBUG_PRINTLN("Sensores inicializados correctamente.");

};

void loop() { 
  // Obtener el tiempo actual
  unsigned long currentMillis = millis();

  // *****************************************
  //  Tarea que se ejecuta en cada iteración
  // *****************************************

  lee_sensores();
  actualiza_filtro();
  lee_barometro();
  actualiza_registros_modbus();

  // Asignar valores a los registros Modbus
  slave.Hreg(0, var1);  // Registro 0
  slave.Hreg(1, var2);  // Registro 1
  slave.Hreg(2, var3);  // Registro 2
  slave.Hreg(3, var4);  // Registro 3

  slave.poll(); // Procesar solicitudes Modbus

  // Tarea que se ejecuta cada 1 ms
  if (currentMillis - lastTask1ms >= 1) {
    lastTask1ms = currentMillis;

  }

  // Tarea que se ejecuta cada 10 ms
  if (currentMillis - lastTask10ms >= 10) {
    lastTask10ms = currentMillis;

  }

  // Tarea que se ejecuta cada 100 ms
  if (currentMillis - lastTask100ms >= 100) {
    lastTask100ms = currentMillis;

  }

  // Tarea que se ejecuta cada 1 segundo
  if (currentMillis - lastTask1s >= 1000) {
    lastTask1s = currentMillis;
    digitalWrite(PC13, !digitalRead(PC13));
    DEBUG_PRINT("Test."); 
  }

  // Tarea que se ejecuta cada 1 minuto
  if (currentMillis - lastTask1min >= 60000) {
    lastTask1min = currentMillis;
  }

  delay(1);  // Evita sobrecarga del bus
};

void lee_sensores(){

    // Leer datos del LSM6DSOX
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6dsox.getEvent(&accel, &gyro, &temp);

    _datosavionica.RawMemsAcc_x = accel.acceleration.x;
    _datosavionica.RawMemsAcc_y = accel.acceleration.y;
    _datosavionica.RawMemsAcc_z = accel.acceleration.z;

    _datosavionica.RawMemsAngularRate_x = gyro.gyro.x;
    _datosavionica.RawMemsAngularRate_y = gyro.gyro.y;
    _datosavionica.RawMemsAngularRate_z = gyro.gyro.z;

    // Leer datos del LIS3MDL
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);

    _datosavionica.RawMag_x = mag.magnetic.x;
    _datosavionica.RawMag_y = mag.magnetic.y;
    _datosavionica.RawMag_z = mag.magnetic.z;

  
    // Para depuración, imprimir los valores leídos
    DEBUG_PRINTLN("----- Datos de Avionica -----");
    DEBUG_PRINT("Aceleración (m/s²): X=");
    DEBUG_PRINT(_datosavionica.RawMemsAcc_x);
    DEBUG_PRINT(" Y=");
    DEBUG_PRINT(_datosavionica.RawMemsAcc_y);
    DEBUG_PRINT(" Z=");
    DEBUG_PRINTLN(_datosavionica.RawMemsAcc_z);

    DEBUG_PRINT("Tasa Angular (rad/s): X=");
    DEBUG_PRINT(_datosavionica.RawMemsAngularRate_x);
    DEBUG_PRINT(" Y=");
    DEBUG_PRINT(_datosavionica.RawMemsAngularRate_y);
    DEBUG_PRINT(" Z=");
    DEBUG_PRINTLN(_datosavionica.RawMemsAngularRate_z);

    DEBUG_PRINT("Magnetómetro (μT): X=");
    DEBUG_PRINT(_datosavionica.RawMag_x);
    DEBUG_PRINT(" Y=");
    DEBUG_PRINT(_datosavionica.RawMag_y);
    DEBUG_PRINT(" Z=");
    DEBUG_PRINTLN(_datosavionica.RawMag_z);

    DEBUG_PRINTLN("------------------------------\n");
  #ifdef DEBUG
    delay(500); // Espera medio segundo antes de la siguiente lectura
  #endif
};

void lee_barometro(){
    
  // Leer la temperatura en grados Celsius
  _datosavionica.TemperaturaSensorBMP280 = bmp.readTemperature();
  
  // Leer la presión en hPa
  _datosavionica.AtmosfericPressure = bmp.readPressure() / 100.0F;
  
  // Calcular la altitud en metros
  _datosavionica.BarometricAlt = bmp.readAltitude(QNE)*PiesMetro;
  DEBUG_PRINTLN(_datosavionica.BarometricAlt);

};

void actualiza_filtro(){
    //if (Filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT())) {
    tstart = micros();
    if (Filter.update(_datosavionica.RawMemsAngularRate_x,_datosavionica.RawMemsAngularRate_y,_datosavionica.RawMemsAngularRate_z,_datosavionica.RawMemsAcc_x,_datosavionica.RawMemsAcc_y,_datosavionica.RawMemsAcc_z,_datosavionica.RawMag_x,_datosavionica.RawMag_y,_datosavionica.RawMag_z)){
      tstop = micros();
      Serial.print("Pich: ");
      Serial.print(Filter.getPitch_rad()*180.0f/PI);
      Serial.print("\n");
      Serial.print("Roll: ");
      Serial.print(Filter.getRoll_rad()*180.0f/PI);
      Serial.print("\n");
      Serial.print("Yaw: ");
      Serial.print(Filter.getYaw_rad()*180.0f/PI);
      Serial.print("\n");
      _datosavionica.KalmanFilteredPitch=(Filter.getPitch_rad()*180.0f/PI);
      _datosavionica.KalmanFilteredRoll=(Filter.getRoll_rad()*180.0f/PI);
      _datosavionica.KalmanFilteredYaw=(Filter.getYaw_rad()*180.0f/PI);
  
      DEBUG_PRINT(Filter.getHeading_rad()*180.0f/PI);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(Filter.getGyroBiasX_rads());
      DEBUG_PRINT("\t");
      DEBUG_PRINT(Filter.getGyroBiasY_rads());
      DEBUG_PRINT("\t");
      DEBUG_PRINT(Filter.getGyroBiasZ_rads());
      DEBUG_PRINT("\t");
      DEBUG_PRINTLN(tstop - tstart);
    };
};

void actualiza_registros_modbus() {

    // Convertir cada `float` a dos registros de 16 bits
    uint32_t temp;
    
    var1 = (int16_t)(_datosavionica.KalmanFilteredPitch*10.0);
    var2 = (int16_t)(_datosavionica.KalmanFilteredRoll*10.0);
    var3 = (int16_t)(_datosavionica.KalmanFilteredYaw*10.0);
    var4 = (int16_t)(_datosavionica.BarometricAlt);



}


/*
void actualiza_registros_modbus() {
  // Velocidades
  avionicaDataUnion.datos.IAS = _datosavionica.IAS;
  avionicaDataUnion.datos.CAS = _datosavionica.CAS;
  avionicaDataUnion.datos.TAS = _datosavionica.TAS;
  avionicaDataUnion.datos.GS = _datosavionica.GS;
  
  // Datos del GPS
  avionicaDataUnion.datos.GPS_Valid = _datosavionica.GPS_Valid;
  avionicaDataUnion.datos.SatNum = _datosavionica.SatNum;
  avionicaDataUnion.datos.RawGpsTime = _datosavionica.RawGpsTime;
  avionicaDataUnion.datos.RawGpsLat = _datosavionica.RawGpsLat;
  avionicaDataUnion.datos.RawGpsLon = _datosavionica.RawGpsLon;
  avionicaDataUnion.datos.RawGpsAlt = _datosavionica.RawGpsAlt;
  
  avionicaDataUnion.datos.RawGpsYear = _datosavionica.RawGpsYear;
  avionicaDataUnion.datos.RawGpsMonth = _datosavionica.RawGpsMonth;
  avionicaDataUnion.datos.RawGpsDay = _datosavionica.RawGpsDay;

  // Datos de sensores inerciales y magnéticos
  avionicaDataUnion.datos.RawMemsAcc_x = _datosavionica.RawMemsAcc_x;
  avionicaDataUnion.datos.RawMemsAcc_y = _datosavionica.RawMemsAcc_y;
  avionicaDataUnion.datos.RawMemsAcc_z = _datosavionica.RawMemsAcc_z;

  avionicaDataUnion.datos.RawMemsAngularRate_x = _datosavionica.RawMemsAngularRate_x;
  avionicaDataUnion.datos.RawMemsAngularRate_y = _datosavionica.RawMemsAngularRate_y;
  avionicaDataUnion.datos.RawMemsAngularRate_z = _datosavionica.RawMemsAngularRate_z;

  avionicaDataUnion.datos.RawMag_x = _datosavionica.RawMag_x;
  avionicaDataUnion.datos.RawMag_y = _datosavionica.RawMag_y;
  avionicaDataUnion.datos.RawMag_z = _datosavionica.RawMag_z;
  
  // Datos de actitud y posición filtrados
  avionicaDataUnion.datos.KalmanFilteredPitch = _datosavionica.KalmanFilteredPitch;
  avionicaDataUnion.datos.KalmanFilteredRoll = _datosavionica.KalmanFilteredRoll;
  avionicaDataUnion.datos.KalmanFilteredYaw = _datosavionica.KalmanFilteredYaw;
  
  // Datos de presión atmosférica y altitud
  avionicaDataUnion.datos.AtmosfericPressure = _datosavionica.AtmosfericPressure;
  avionicaDataUnion.datos.BarometricAlt = _datosavionica.BarometricAlt;
  avionicaDataUnion.datos.TemperaturaSensorBMP280 = _datosavionica.TemperaturaSensorBMP280;
  avionicaDataUnion.datos.QNH = _datosavionica.QNH;
};

/*
void procesado_inercial_con_gps_incluido(){
  gps.read(&uBloxData);
  if (uBloxData.numSV > 5) {
    if (newData == 1) {
      newData = 0;
      tstart = micros();
      // read the sensor
      Imu.readSensor();
      // update the filter
      Filter.update(uBloxData.iTOW,uBloxData.velN,uBloxData.velE,uBloxData.velD,uBloxData.lat*PI/180.0f,uBloxData.lon*PI/180.0f,uBloxData.hMSL,Imu.getGyroY_rads(),-1*Imu.getGyroX_rads(),Imu.getGyroZ_rads(),Imu.getAccelY_mss(),-1*Imu.getAccelX_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT());
      Serial.print(Filter.getPitch_rad()*180.0f/PI);
      Serial.print("\t");
      Serial.print(Filter.getRoll_rad()*180.0f/PI);
      Serial.print("\t");
      Serial.print(Filter.getYaw_rad()*180.0f/PI);
      Serial.print("\n");
      tstop = micros();
    }
  }
};
*/

void scan_i2c() {
  byte error, address;
  int dispositivos = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en la dirección 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("!");

      dispositivos++;
    } else if (error == 4) {
      Serial.print("Error desconocido en la dirección 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (dispositivos == 0) {
    Serial.println("No se encontraron dispositivos I2C");
  } else {
    Serial.print("Total de dispositivos encontrados: ");
    Serial.println(dispositivos);
  }

}
