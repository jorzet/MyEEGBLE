#include <BLE_API.h>


#define DEVICE_NAME            "BLE_Peripheral"
#define TXRX_BUF_LEN           20

BLE                            ble;
Timeout                        timeout;

static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state=0;

// The uuid of service and characteristics
static const uint8_t service1_uuid[]        = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]     = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]     = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[]   = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};

// Initialize value of chars
GattCharacteristic  characteristic1(service1_tx_uuid, tx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );
GattCharacteristic  characteristic2(service1_rx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2};
GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

#define MAX_PACKET_LENGTH 32
#define EEG_POWER_BANDS 8

int g = 0;
static uint8_t datos[TXRX_BUF_LEN];
unsigned long starttime, endtime;
uint8_t cont = 1;
char resp;

boolean update();

// String with most recent error.
        char* readErrors();

        // Returns comme-delimited string of all available brain data.
        // Sequence is as below.
        char* readCSV();

        // Individual pieces of brain data.
        uint8_t readSignalQuality();
        uint8_t readAttention();
        uint8_t readMeditation();
        uint32_t* readPowerArray();
        uint32_t readDelta();
        uint32_t readTheta();
        uint32_t readLowAlpha();
        uint32_t readHighAlpha();
        uint32_t readLowBeta();
        uint32_t readHighBeta();
        uint32_t readLowGamma();
        uint32_t readMidGamma();

        uint8_t packetData[MAX_PACKET_LENGTH];
        boolean inPacket;
        uint8_t latestByte;
        uint8_t lastByte;
        uint8_t packetIndex;
        uint8_t packetLength;
        uint8_t checksum;
        uint8_t checksumAccumulator;
        uint8_t eegPowerLength;
        boolean hasPower;
        void clearPacket();
        void clearEegPower();
        boolean parsePacket();

        void printPacket();
        void init();
        void printCSV(); // maybe should be public?
        void printDebug();

        // With current hardware, at most we would have...
        // 3 x 3 char uint8_t
        // 8 x 10 char uint32_t
        // 10 x 1 char commas
        // 1 x 1 char 0 (string termination)
        // -------------------------
        // 100 characters
        char csvBuffer[100];

        // Longest error is
        // 22 x 1 char uint8_ts
        // 1 x 1 char 0 (string termination)
        char latestError[23];

        uint8_t signalQuality;
        uint8_t attention;
        uint8_t meditation;

        boolean freshPacket;

        // Lighter to just make this public, instead of using the getter?
        uint32_t eegPower[EEG_POWER_BANDS];

void init() {
    // It's up to the calling code to start the stream
    // Usually Serial.begin(9600);
    freshPacket = false;
    inPacket = false;
    packetIndex = 0;
    packetLength = 0;
    eegPowerLength = 0;
    hasPower = false;
    checksum = 0;
    checksumAccumulator = 0;

    signalQuality = 200;
    attention = 0;
    meditation = 0;

    clearEegPower();
}

boolean update() {
    if (Serial.available()) {
        latestByte = Serial.read();

        // Build a packet if we know we're and not just listening for sync bytes.
        if (inPacket) {

            // First byte after the sync bytes is the length of the upcoming packet.
            if (packetIndex == 0) {
                packetLength = latestByte;
                ////Serial.print(packetLength);
                ////Serial.println("aaaaa");

                // Catch error if packet is too long
                if (packetLength > MAX_PACKET_LENGTH) {
                    // Packet exceeded max length
                    // Send an error
                    sprintf(latestError, "ERROR: Packet too long %i", packetLength);
                    inPacket = false;
                }
            }
            else if (packetIndex <= packetLength) {
                // Run of the mill data bytes.

                // Print them here

                // Store the byte in an array for parsing later.
                packetData[packetIndex - 1] = latestByte;

                // Keep building the checksum.
                checksumAccumulator += latestByte;
            }
            else if (packetIndex > packetLength) {
                // We're at the end of the data payload.

                // Check the checksum.
                checksum = latestByte;
                checksumAccumulator = 255 - checksumAccumulator;

                // Do they match?
                if (checksum == checksumAccumulator) {
                    boolean parseSuccess = parsePacket();

                    if (parseSuccess) {
                        freshPacket = true;
                    }
                    else {
                        // Parsing failed, send an error.
                        sprintf(latestError, "ERROR: Could not parse");
                        // good place to print the packet if debugging
                    }
                }
                else {
                    // Checksum mismatch, send an error.
                    sprintf(latestError, "ERROR: Checksum");
                    // good place to print the packet if debugging
                }
                // End of packet

                // Reset, prep for next packet
                inPacket = false;
            }

            packetIndex++;
        }

        // Look for the start of the packet
        if ((latestByte == 170) && (lastByte == 170) && !inPacket) {
            // Start of packet
            inPacket = true;
            packetIndex = 0;
            checksumAccumulator = 0;
        }

        // Keep track of the last byte so we can find the sync byte pairs.
        lastByte = latestByte;
    }

    if (freshPacket) {
        freshPacket = false;
        return true;
    }
    else {
        return false;
    }

}

void clearPacket() {
    for (uint8_t i = 0; i < MAX_PACKET_LENGTH; i++) {
        packetData[i] = 0;
    }
}

void clearEegPower() {
    // Zero the power bands.
    for(uint8_t i = 0; i < EEG_POWER_BANDS; i++) {
        eegPower[i] = 0;
    }
}

boolean parsePacket() {
    // Loop through the packet, extracting data.
    // Based on mindset_communications_protocol.pdf from the Neurosky Mindset SDK.
    // Returns true if passing succeeds
    hasPower = false;
    boolean parseSuccess = true;
    int rawValue = 0;

    clearEegPower();    // clear the eeg power to make sure we're honest about missing values
    
    for (uint8_t i = 0; i < packetLength; i++) {
        switch (packetData[i]) {
            case 0x2:
                ////Serial.println("signalQuality");
                signalQuality = packetData[++i];
                break;
            case 0x4:
                ////Serial.println("attention");
                attention = packetData[++i];
                break;
            case 0x5:
                ////Serial.println("meditation");
                meditation = packetData[++i];
                break;
            case 0x83:
                // ASIC_EEG_POWER: eight big-endian 3-uint8_t unsigned integer values representing delta, theta, low-alpha high-alpha, low-beta, high-beta, low-gamma, and mid-gamma EEG band power values
                // The next uint8_t sets the length, usually 24 (Eight 24-bit numbers... big endian?)
                // We dont' use this value so let's skip it and just increment i
                i++;
                ////Serial.println("eegPower");
                // Extract the values
                for (int j = 0; j < EEG_POWER_BANDS; j++) {

                    eegPower[j] = ((uint32_t)packetData[++i] << 16) | ((uint32_t)packetData[++i] << 8) | (uint32_t)packetData[++i];
                }

                hasPower = true;
                // This seems to happen once during start-up on the force trainer. Strange. Wise to wait a couple of packets before
                // you start reading.
                break;
            case 0x80:
                // We dont' use this value so let's skip it and just increment i
                // uint8_t packetLength = packetData[++i];
                
                i++;
                rawValue = ((int)packetData[++i] << 8) | packetData[++i];
                ////Serial.println(rawValue);
               
                datos[g] = rawValue;
                g++;
                
                ////Serial.println("%d", rawValue);
                break;
            default:
                // Broken packet ?
                /*
                //Serial.print(F("parsePacket UNMATCHED data 0x"));
                //Serial.print(packetData[i], HEX);
                //Serial.print(F(" in position "));
                //Serial.print(i, DEC);
                printPacket();
                */
                ////Serial.println("parseSuccess");
                parseSuccess = false;
                break;
        }
    }
    return parseSuccess;
}

// Keeping this around for debug use
void printCSV() {
    // Print the CSV over serial
    //Serial.print(signalQuality, DEC);
    //Serial.print(",");
    //Serial.print(attention, DEC);
    //Serial.print(",");
    //Serial.print(meditation, DEC);

    if (hasPower) {
        for(int i = 0; i < EEG_POWER_BANDS; i++) {
            ////Serial.print(",");
            ////Serial.print(eegPower[i], DEC);
        }
    }

    ////Serial.println("");
}

char* readErrors() {
    return latestError;
}

char* readCSV() {
    // spit out a big string?
    // find out how big this really needs to be
    // should be popped off the stack once it goes out of scope?
    // make the character array as small as possible

    if(hasPower) {

        sprintf(csvBuffer,"%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
            signalQuality,
            attention,
            meditation,
            eegPower[0],
            eegPower[1],
            eegPower[2],
            eegPower[3],
            eegPower[4],
            eegPower[5],
            eegPower[6],
            eegPower[7]
        );

        return csvBuffer;
    }
    else {
        sprintf(csvBuffer,"%d,%d,%d",
            signalQuality,
            attention,
            meditation
        );

        return csvBuffer;
    }
}

// For debugging, print the entire contents of the packet data array.
void printPacket() {
    //Serial.print("[");
    for (uint8_t i = 0; i < MAX_PACKET_LENGTH; i++) {
        //Serial.print(packetData[i], DEC);

            if (i < MAX_PACKET_LENGTH - 1) {
                //Serial.print(", ");
            }
    }
    //Serial.println("]");
}

void printDebug() {
    //Serial.println("");
    //Serial.println("--- Start Packet ---");
    //Serial.print("Signal Quality: ");
    //Serial.println(signalQuality, DEC);
    //Serial.print("Attention: ");
    //Serial.println(attention, DEC);
    //Serial.print("Meditation: ");
    //Serial.println(meditation, DEC);

    if (hasPower) {
        //Serial.println("");
        //Serial.println("EEG POWER:");
        //Serial.print("Delta: ");
        //Serial.println(eegPower[0], DEC);
        //Serial.print("Theta: ");
        //Serial.println(eegPower[1], DEC);
        //Serial.print("Low Alpha: ");
        //Serial.println(eegPower[2], DEC);
        //Serial.print("High Alpha: ");
        //Serial.println(eegPower[3], DEC);
        //Serial.print("Low Beta: ");
        //Serial.println(eegPower[4], DEC);
        //Serial.print("High Beta: ");
        //Serial.println(eegPower[5], DEC);
        //Serial.print("Low Gamma: ");
        //Serial.println(eegPower[6], DEC);
        //Serial.print("Mid Gamma: ");
        //Serial.println(eegPower[7], DEC);
    }

    //Serial.println("");
    //Serial.print("Checksum Calculated: ");
    //Serial.println(checksumAccumulator, DEC);
    //Serial.print("Checksum Expected: ");
    //Serial.println(checksum, DEC);

    //Serial.println("--- End Packet ---");
    //Serial.println("");
}

uint8_t readSignalQuality() {
    return signalQuality;
}

uint8_t readAttention() {
    return attention;
}

uint8_t readMeditation() {
    return meditation;
}

uint32_t* readPowerArray() {
    return eegPower;
}

uint32_t readDelta() {
    return eegPower[0];
}

uint32_t readTheta() {
    return eegPower[1];
}

uint32_t readLowAlpha() {
    return eegPower[2];
}

uint32_t readHighAlpha() {
    return eegPower[3];
}

uint32_t readLowBeta() {
    return eegPower[4];
}

uint32_t readHighBeta() {
    return eegPower[5];
}

uint32_t readLowGamma() {
    return eegPower[6];
}

uint32_t readMidGamma() {
    return eegPower[7];
}


/* *********************************************** */
void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  //Serial.println("Disconnected!");
  //Serial.println("Restarting the advertising process");
  ble.startAdvertising();
}

void gattServerWriteCallBack(const GattWriteCallbackParams *Handler) {
  uint8_t buf[TXRX_BUF_LEN];
  uint16_t bytesRead, index;

  //Serial.println("onDataWritten : ");
  if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
    ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
    //Serial.print("bytesRead: ");
    //Serial.println(bytesRead, HEX);
    for(index=0; index<bytesRead; index++) {
      resp = buf[index];
      Serial.write(buf[index]);
    }
    //Serial.println("");
  }
}

void m_uart_rx_handle() {   //update characteristic data
  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), datos, TXRX_BUF_LEN);
  memset(datos, 0x00,TXRX_BUF_LEN);
  rx_state = 0;
  g=0;
}

void uart_handle(uint32_t id, SerialIrq event) {   /* Serial rx IRQ */
  
      timeout.attach_us(m_uart_rx_handle, 100000);
}

/************************************************** */

void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);
  init();
  //Serial.attach(uart_handle);
  pinMode(13, OUTPUT);
  //digitalWrite(13, LOW);
  
  

  ble.init();
  ble.onDisconnection(disconnectionCallBack);
  ble.onDataWritten(gattServerWriteCallBack);

  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                   (const uint8_t *)"TXRX", sizeof("TXRX") - 1);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                   (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid_rev));
  // set adv_type
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  // add service
  ble.addService(uartService);
  // set device name
  ble.setDeviceName((const uint8_t *)"Simple Chat");
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // start advertising
  ble.startAdvertising();
  //Serial.println("Advertising Start!");

}

void loop() {
  // put your main code here, to run repeatedly:
  /*update();
  digitalWrite(13, HIGH);
  Serial.println(g);
  if(g==TXRX_BUF_LEN){
          digitalWrite(13, LOW);
          g = 0;
          rx_state = 0;
  }*/
  

   if(ble.getGapState().connected && resp == '1'){
        update();
        digitalWrite(13, HIGH);
        if(g==TXRX_BUF_LEN-1){
          datos[16] = readSignalQuality();
          datos[17] = 0x00;
          //Serial.println(g);
          digitalWrite(13, LOW);
          m_uart_rx_handle();
          cont++;
        }
    }
    //count = 0;
    /*starttime = millis();
    endtime = starttime;
    while ((endtime-starttime) <= 1000){
      update();
      //count = count + 1;
      //Serial.println(count);
      Serial.print(g);
      }*/
      
      

  
    
      
}

