#ifndef DATOS_ISLETA_AVIONICA  // Si DATOS_H no está definido
#define DATOS_ISLETA_AVIONICA  // Definir DATOS_H

// Definición de la estructura para los datos de la avionica.
typedef struct {
  // Velocidades.
  float IAS, CAS, TAS;
  float GS;
  
  // Datos del GPS.
  bool GPS_Valid;
  int SatNum;
  float RawGpsTime;
  float RawGpsLat;
  float RawGpsLon;
  float RawGpsAlt;
  int RawGpsYear;
  int RawGpsMonth;
  int RawGpsDay;

  // Datos de sensores inerciales y magnéticos.
  float RawMemsAcc_x;
  float RawMemsAcc_y;
  float RawMemsAcc_z;

  float RawMemsAngularRate_x;
  float RawMemsAngularRate_y;
  float RawMemsAngularRate_z;

  float RawMag_x;
  float RawMag_y;
  float RawMag_z;

  // Datos de actitud y posición filtrados.
  float KalmanFilteredPitch;
  float KalmanFilteredRoll;
  float KalmanFilteredYaw;

  // Datos de presión atmosférica altutid.
  float AtmosfericPressure;
  float BarometricAlt;
  float TemperaturaSensorBMP280;

  float QNH;

} DatosModubus_IsletaAvionica;

//DatosModubus_IsletaAvionica  avionicaData;

// Unión para manejar la estructura como un bloque de registros Modbus
union DatosIsletaAvionicaUnion {
    DatosModubus_IsletaAvionica datos;  // Estructura de datos original
    uint16_t modbusRegisters[sizeof(DatosModubus_IsletaAvionica) / 2];  // Array de 16 bits para Modbus
};

// Instancia de la unión que usaremos para Modbus
extern DatosIsletaAvionicaUnion avionicaDataUnion;




#endif