#ifndef DATOSAVIONICA_H  // Si DATOS_H no está definido
#define DATOSAVIONICA_H  // Definir DATOS_H

// Definición de la estructura para los datos de la avionica.
typedef struct {
  
  // Velocidades.
  float IAS, CAS, TAS;
  float VS0, VS1, VA, VFE, VNO, VNE;
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

} DatosAvionica;

extern DatosAvionica _datosavionica;


#endif // Cierre de DATOS_H