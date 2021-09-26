#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

#define ON LOW
#define OFF HIGH

int LED_R = 2;
int LED_B = 4;
int LED_G = 15;

CAN_device_t CAN_cfg;
CAN_frame_t tx_frame, rx_frame;
char i = 33;

void sendMessage() {
  tx_frame.data.u8[7] = i;
  printf("sent %d, %.8s\n", i, tx_frame.data.u8);
  ESP32Can.CANWriteFrame(&tx_frame);
}

void checkMessage() {
  if(xQueueReceive(CAN_cfg.rx_queue,&rx_frame, 0)==pdTRUE){
    digitalWrite(LED_G, ON);

    printf("New %s frame", (rx_frame.FIR.B.FF==CAN_frame_std ? "standard" : "extended"));
    if(rx_frame.FIR.B.RTR==CAN_RTR) printf(" RTR");
    printf(" from 0x%08x, DLC %d\r\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    
    for(int i = 0; i < 8; i++) printf("%c\t", (char)rx_frame.data.u8[i]);

    if(!strncmp((char*)rx_frame.data.u8, "Kill die", 8)) {
      printf("\n---*** Now leaving! WILL CAUSE CPU RESET ***---\n\n");
      digitalWrite(LED_R, ON);
      vTaskSuspendAll();  // suspend all except current
      vTaskSuspend(NULL); // suspend current, will cause FreeRTOS to reboot, will reenter @ setup()
    }
    printf("\n");
  }
  digitalWrite(LED_G, OFF);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello world");

  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x101;
  tx_frame.FIR.B.DLC = 8;
  strcpy((char*)tx_frame.data.u8, "mymsg: ");

  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);
  digitalWrite(LED_B, OFF);
  digitalWrite(LED_G, OFF);
  digitalWrite(LED_R, OFF);   

  CAN_cfg.speed=CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_25;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(10,sizeof(CAN_frame_t));
  ESP32Can.CANInit();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (i >= 127) i = 33;
  //printf("%d\n", i++); 
  i++;
  sendMessage();
  checkMessage();
  delay(500);
}
