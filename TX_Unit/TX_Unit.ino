#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "SerialTransfer.h"
#include "dataPacket.h"

SerialTransfer myTransfer;

dataPacket recieved_Packet;

// Driver Input: data to be transmitted over RS-485 bus
#define DI 6

// Receiver Output: Data received by MCU (MUST BE 3.3V so use voltage divider on Sensing unit side)
#define RO 7

// Driver Enable and Receiever Enable
#define DE 5
#define RE 4

// Reserve UART0 for serial monitor debugging
// Therefore we map to UART1
HardwareSerial RS485(1);

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

typedef enum {
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
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    // 8N1 means 8 data bits, N no parity bit, 1 stop bit
    RS485.begin(115200, SERIAL_8N1, RO, DI);
    myTransfer.begin(RS485);
 
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
    state=LOWPOWER;

  // Set the DE and RE pins as outputs
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);

  // Set DE and RE to HIGH for transmission mode or LOW for receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
}

bool packet_ready_for_LoRa = false;

void check_RS485 () {

  if(myTransfer.available()) {
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(recieved_Packet, recSize);

    packet_ready_for_LoRa = true;
    state = STATE_TX;
  }
}



void loop()
{
  switch(state)
  {
    case STATE_TX:
      
      if (packet_ready_for_LoRa) {
        // Serial.print("Recieved packet #: ")
        // Serial.println(recieved_Packet.packetID);
        Radio.Send((uint8_t*)(&recieved_Packet),sizeof(recieved_Packet));
        packet_ready_for_LoRa = false;
      }
      state=LOWPOWER;
      break;
    case STATE_RX:
      Radio.Rx( 0 );
      state=LOWPOWER;
      break;
    case LOWPOWER:
      check_RS485();
      Radio.IrqProcess( );
      break;
    default:
      check_RS485();
      break;
  }
}

void OnTxDone( void )
{
  //Serial.print("TX done......");
  state=STATE_RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.print("TX Timeout......");
    state=STATE_TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );

    // Serial.print("Received bytes: ");
    // for (uint16_t i = 0; i < size; i++) {
    //   Serial.print(payload[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();

    state = LOWPOWER;
}
