#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>
#include <cmath>

#define ANTENNA_DIST 0.3 // distance between antenna main and b (cm)
#define PASS 10

//UART pin 설정
#define TX_PIN 26
#define RX_PIN 27

typedef struct Position {
    double x;
    double y;
} Position;

// UART
HardwareSerial MySerial(1);


// connection pins
const uint8_t PIN_SCK = 18;  //Clock 데이터
const uint8_t PIN_MOSI = 23; //Master Out Salve In
const uint8_t PIN_MISO = 19; //Master In Slave Out
const uint8_t PIN_SS = 4;  // ss||cs Slave(Chip) Select 여러 슬레이브 중 하나 선택 
const uint8_t PIN_RST = 15; // Reset
const uint8_t PIN_IRQ = 17;  // Interrupt Request 


// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:01";

Position position_self = {0,0};
Position position_B = {3,0};
Position position_C = {3,2.5};

double range_self;
double range_B;
double range_C;

boolean received_B = false;

byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};

byte anchor_b[] = {0x02, 0x00};
uint16_t next_anchor = 2;
byte anchor_c[] = {0x03, 0x00};

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
    true /* This allows blink frames */
};

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));
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
    DW1000Ng::setDeviceAddress(1);
	
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

    MySerial.begin(9600,SERIAL_8N1, RX_PIN, TX_PIN);
}

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

void calcDistanceAngle(double &distance, double&angle){
    double a, b, c;
    
    a = ANTENNA_DIST; // BC, a, BC
    b = range_self; // AC, b, TM
    c = range_B; // AB, c, TB

    distance = sqrt((2*pow(b,2) + 2*pow(c,2) - pow(a,2)) / 4);

    angle = acos((pow(distance,2) + pow(a/2,2) - pow(b,2)) / (2 * distance *(a/2)));

    angle = angle * (180/PI);
    
}


void loop() {
    if(DW1000NgRTLS::receiveFrame()){
        size_t recv_len = DW1000Ng::getReceivedDataLength();
        byte recv_data[recv_len];
        DW1000Ng::getReceivedData(recv_data, recv_len);


        if(recv_data[0] == BLINK) {
            DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tag_shortAddress);
            DW1000NgRTLS::waitForTransmission();

            RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
            if(!result.success) return;
            range_self = result.range;
            range_self = kalman(range_self); // kalman 필터 적용


            String rangeString = "Range: "; rangeString += range_self; rangeString += " m";
            rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
            Serial.println(rangeString);

        } else if(recv_data[9] == 0x60) {
            double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10],2) / 1000.0);
            String rangeReportString = "Range from: "; rangeReportString += recv_data[7];
            rangeReportString += " = "; rangeReportString += range;
            Serial.println(rangeReportString);
            if(recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1]) {
                range_B = range;
                
                double distance, angle;
    
                calcDistanceAngle(distance, angle);

                String fin_data;
                fin_data = "[RANGE] Tag-Main: ";
                fin_data += range_self; 
                fin_data += ", Tag-B: ";
                fin_data += range_B;
                Serial.println(fin_data);

                String dist_angle;
                dist_angle += "[Distance] distance: ";
                dist_angle += distance;
                dist_angle += "\n[Angle] angle: ";
                dist_angle += angle;
                Serial.println(dist_angle);
            

//                //dictionary 형식
//                String uart_data = "{ distance: "; uart_data += distance;
//                uart_data += ", angle: "; uart_data += angle;
//                uart_data += "}";

                // 단순한 형식
                String uart_data;
                uart_data += distance; uart_data += " ";
                uart_data += angle;
               
                MySerial.println(uart_data);
                String uart_text;
                uart_text += "Data sent to Jetson Orin Nano: ";
                uart_text += uart_data;
                uart_text += angle;
                Serial.println(uart_text);

            } 

            else {
                // received_B = false;
                Serial.println("[ERROR] Received Data is neither Blink nor Anchor_B");
            }
        }
    }

    
}
