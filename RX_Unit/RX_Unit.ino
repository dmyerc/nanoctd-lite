#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <string.h>
#include "dataPacket.h"

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             20        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

int16_t txNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;


void setup() {
    Serial.begin(115200);
    // Serial.println("packetID,voltage_mV,temp_C,rtd_C,pressure_hPa,rssi,snr");
    Serial.println("packetID,voltage_mV[0],voltage_mV[1],voltage_mV[2],,temp_C,pressure_hPa,rssi,snr");

    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    txNumber=0;
    Rssi=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    state=STATE_RX;
}

void loop()
{
  switch(state)
  {
    // Shore station only needs to listen
    case STATE_TX:
      state=LOWPOWER;
      break;

    case STATE_RX:
      Radio.Rx(0);
      state=LOWPOWER;
      break;

    case LOWPOWER:
      Radio.IrqProcess( );
      break;

    // want the default state to be listening
    default:
      state = STATE_RX;
      break;
  }
}

void OnTxDone( void )
{
  Serial.print("TX done......");
  state=STATE_RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.print("TX Timeout......");
    state=STATE_TX;
}

// changed this from original
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep();
    
    if (size != sizeof(dataPacket)) {
      Serial.print("Size mismatch");
      state = STATE_RX;
      return;
    }

    dataPacket packet;
    memcpy(&packet, payload, sizeof(dataPacket));

    Serial.print(packet.packetID);
    Serial.print(",");

    Serial.print(packet.voltage_mV[0]);
    Serial.print(",");

    Serial.print(packet.voltage_mV[1]);
    Serial.print(",");

    Serial.print(packet.voltage_mV[2]);
    Serial.print(",");

    Serial.print(packet.temp_C);
    Serial.print(",");

    // Serial.print(packet.rtd_C);
    // Serial.print(",");
    
    Serial.print(packet.pressure_hPa);
    Serial.print(",");

    Serial.print(packet.aanderaa_cond);
    Serial.print(",");
    Serial.print(packet.aanderaa_sal);
    Serial.print(",");
    Serial.print(packet.aanderaa_temp);
    Serial.print(",");

    Serial.print(rssi);
    Serial.print(",");

    Serial.println(snr);
       
    state=STATE_RX;
}