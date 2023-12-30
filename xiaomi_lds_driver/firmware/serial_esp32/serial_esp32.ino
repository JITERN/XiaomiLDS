/* 
PACKET STRUCTURE


    <HEAD> <ANGLE> <SPEED_LSB> <SPEED_MSB> <16 BYTES FOR DATA> <CHECKSUM_LSB> <CHECKSUM_MSB>

    Of which inside the 16 bytes of data, there are four sets of the following bytes: 

    <DISTANCE_LSB><DISTANCE_MSB><SIGNAL_STR_LSB><SIGNAL_STR_MSB>

*/

#define lidar Serial2 //pin RX2 on ESP32


// index of the bytes
#define ANGLE_IDX 1
#define SPEED_LSB 2
#define SPEED_MSB 3
#define DATA_1 4
#define DATA_2 8
#define DATA_3 12
#define DATA_4 16 
#define CHECKSUM_LSB 20
#define CHECKSUM_MSB 21

#define PACKET_SIZE 22  // size of packet
#define DATA_SIZE 7 // angle, speed, distance x 4, irradiance, validity
int data[DATA_SIZE]; // [angle, speed, distance 1, distance 2, distance 3, distance 4, checksum]
uint8_t packet[PACKET_SIZE];    // packet buffer
const unsigned char HEAD_BYTE = 0xFA;   // start byte of the packet
unsigned int packetIndex      = 0;      // packet index
uint8_t receivedByte    = 0;    // received byte
uint8_t lowStrengthFlag = 0; 
bool PACKET_OK  = true;         // if the packet is valid
bool waitPacket = true;         // true if waiting for a packet

#define M_PIN1 12
#define M_PIN2 13

int PWM = 180;
int goal_speed = 250;//rpm


TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
//  lidar.begin(115200);
  lidar.begin(115200,SERIAL_8N1, 4,5); //int8_t rxPin=4, int8_t txPin=5 
     
  Serial.begin(115200);
  pinMode(M_PIN1,OUTPUT);
  pinMode(M_PIN2,OUTPUT);
  analogWrite(M_PIN1,PWM);
  digitalWrite(M_PIN2,LOW);


  xTaskCreatePinnedToCore(Task1code,"Task1",10000,NULL,1,&Task1,0);                         
  delay(500); 

  xTaskCreatePinnedToCore(Task2code,"Task2",10000,NULL,1,&Task2,1);          
  delay(500); 
}

void Task1code( void * parameter ){ //simple control loop
  for(;;){
  if (data[1]<goal_speed){PWM++;}
  else if (data[1]>goal_speed){PWM--;}
  analogWrite(M_PIN1,PWM);
  delay(500);
  }
}


void Task2code( void * parameter ){  //read and process Lidar data
    // check if any packet if arrived
   for(;;){
    if (lidar.available() > 0) {
        receivedByte = lidar.read();
//        Serial.print(receivedByte,HEX);
//
//    if (receivedByte == HEAD_BYTE) {
//         Serial.println();}

        if (waitPacket) { // wait for a new packet to arrive
            if (receivedByte == HEAD_BYTE) {
                

                packetIndex = 0;    // initialise packet index
                waitPacket = false;
                packet[packetIndex++] = receivedByte;
            }
        } else {  // if currently receiving packet
            if (packet[0] == HEAD_BYTE) { // ensure the head of the packet is valid
                packet[packetIndex++] = receivedByte; // store received byte  
                if (packetIndex >= PACKET_SIZE) { // if packet buffer is full
                    waitPacket = true; // wait for a new packet

                    decodePacket(packet, PACKET_SIZE); // process the packet 
                     
                    for (int i = 0; i < DATA_SIZE; i++) {//print all the data
                        Serial.print(data[i]); 
                        Serial.print('\t'); 
       
                    }
                    Serial.println();    
                      
                }
            }
        }

    }
   }

}

void decodePacket(uint8_t packet[], int packetSize) {
    int data_idx = 0; 

    for (int idx = 0; idx < DATA_SIZE; idx++) data[idx] = 0;  // initialise data array

    for (int i = 0; i < packetSize; i++){
//       Serial.print("0x"); Serial.print(packet[i]); Serial.print('\t');
    
        if (i == 0) {   // header byte
          // Serial.print("data: ");
          continue;
        }
        else if (i == 1) {
            uint16_t angle = (packet[i] - 0xA0) * 4;  // convert to values between 0 ~ 360
            if (angle > 360) return; 
            // Serial.print(angle); 
            data[data_idx++] = angle;
            // Serial.print('\t');
        }
        else if (i == 2) {
            int speed = 0x00; 
            speed |= ( (packet[3]<<8) | packet[2])/64;     

            // an attempt to smoothen the speed readings since they sometimes spikes due to unknown issue
//            currentSpeed = abs(speed/64-currentSpeed) > 100 ? currentSpeed*0.95 + (speed/64)*0.05: speed/64; 
            // Serial.print(currentSpeed);
            data[data_idx++] = speed;
            // Serial.print('\t');
        }
        else if (i == 4 || i == 8 || i == 12 || i == 16) {
            uint16_t distance = 0x00;
            distance |= ((packet[i+1]&0x3F) << 8) | packet[i]; 
            // Serial.print(distance);
            data[data_idx++] = distance;

            // Serial.print('\t');
            // Serial.print('\t'); Serial.print(packet[i+3]>>8 + packet[i+2]);
            // if (packet[i+1]&& 1<<7) {
            //   Serial.print("inv: ");
            //   if (packet[i+1] && 1 << 6) {
            //     Serial.print("str: ");
            //   }
            // }     
            // else {
            //   Serial.print(distance/10.0);
            // }
        }
      
    }

    uint16_t chksum = checksum(packet, (unsigned int)(packet[PACKET_SIZE-2] + (packet[PACKET_SIZE-1]<<8)),PACKET_SIZE-2);
    data[data_idx++] = chksum; 
    
}

uint16_t checksum(uint8_t packet[], uint16_t sum, uint8_t size)
{
    uint32_t chk32 = 0;
    uint16_t data[size/2]; 
    uint8_t  sensorData[size]; 

    for (int i = 0; i < size; i++) sensorData[i] = packet[i];

    for (int i = 0; i < size/2; i++) {
        data[i] = ((sensorData[i*2+1] << 8) + sensorData[i*2]);
        chk32 = (chk32 << 1) + data[i];
    }

    uint32_t checksum=(chk32 & 0x7FFF) + (chk32 >> 15);
    return  (uint16_t) (checksum & 0x7FFF) == sum;
}

void loop() {
  
}
