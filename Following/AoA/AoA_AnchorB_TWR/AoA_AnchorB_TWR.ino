#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

#define PASS 10

// connection pins
const uint8_t PIN_SCK = 18;  //Clock 데이터
const uint8_t PIN_MOSI = 23; //Master Out Salve In
const uint8_t PIN_MISO = 19; //Maister In Slave Out
const uint8_t PIN_SS = 4;  // ss||cs Slave(Chip) Select 여러 슬레이브 중 하나 선택 
const uint8_t PIN_RST = 15; // Reset
const uint8_t PIN_IRQ = 17;  // Interrupt Request 

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:02";

byte main_anchor_address[] = {0x01, 0x00};

uint16_t next_anchor = 3;

double range_self;

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};

float A = 1, H = 1, Q = 0, R = 4, x = 0.3, P =6;
int passed_time = 0;
bool first = true;

float kalman(double distance){
  if(first){
    first = false;
  }
  else if (abs(distance - x) > 1 && passed_time < PASS){
    passed_time += 1;
    return x;
  }
  if(passed_time == PASS){
    passed_time = 0;
  }
    
  float xp = A * x;
  float pp = A * P * A + Q;
  float K = pp * H / (H * pp * H + R);
  x = xp + K*(distance - H * xp);
  P = pp - K * H * pp;
  return x;
}

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("### arduino-DW1000Ng-ranging-anchor-B ###"));
    // initialize the driver
    #if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
    
    DW1000Ng::setEUI(EUI);

    DW1000Ng::setPreambleDetectionTimeout(64);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

    DW1000Ng::setNetworkId(RTLS_APP_ID);
    DW1000Ng::setDeviceAddress(2);
	
    DW1000Ng::setAntennaDelay(16436);
    
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
  
}

void transmitRangeReport() {
    byte rangingReport[] = {DATA, SHORT_SRC_AND_DEST, DW1000NgRTLS::increaseSequenceNumber(), 0,0, 0,0, 0,0, 0x60, 0,0 };
    DW1000Ng::getNetworkId(&rangingReport[3]);
    memcpy(&rangingReport[5], main_anchor_address, 2);
    DW1000Ng::getDeviceAddress(&rangingReport[7]);
    DW1000NgUtils::writeValueToBytes(&rangingReport[10], static_cast<uint16_t>((range_self*1000)), 2);
    DW1000Ng::setTransmitData(rangingReport, sizeof(rangingReport));
    DW1000Ng::startTransmit();
}
 
void loop() {     
        RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
        if(result.success) {
            delay(2); // Tweak based on your hardware
            range_self = result.range;
            Serial.print("origin: ");
            Serial.println(range_self);
            range_self = kalman(range_self);
            Serial.print("kalman: ");
            Serial.println(range_self);
            transmitRangeReport();

            String rangeString = "Range: "; rangeString += range_self; rangeString += " m";
            rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
            Serial.println(rangeString);
        }
}
