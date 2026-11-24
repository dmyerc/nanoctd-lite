#pragma once

struct dataPacket {
  uint16_t packetID;
  float voltage_mV[3];
  float temp_C;
  // float rtd_C;
  float pressure_hPa;
  float aanderaa_cond;
  float aanderaa_sal;
  float aanderaa_temp;
};
