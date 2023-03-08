int m1sig = 0;
int m2sig = 0;
unsigned long previousMicros1=0;
unsigned long previousMicros2=0;


void setup() {
  Serial.begin(9600);   // opens serial port, sets data rate to 9600 bps
  pinMode(13, OUTPUT); //MOTOR1_PULSE
  pinMode(12, OUTPUT); //MOTOR2_PULSE
  pinMode(11, OUTPUT); //MOTOR1_DIR
  pinMode(10, OUTPUT); //MOTOR1_DIR
  Serial.println("Serialized!");

}

void loop() {
if(Serial.available() > 0){
  String inp = Serial.readStringUntil('\n');

  inp.trim();               
  int n = inp.indexOf(" ");
  m1sig = inp.substring(0,n).toInt();
    Serial.print("m1sig : ");
    Serial.println(m1sig);
  m2sig = inp.substring(n).toInt();
    Serial.print("m2sig : ");
    Serial.println(m2sig);

  if(m1sig < 0) {
    digitalWrite(11, LOW);
    m1sig = -m1sig;
  } else {
    digitalWrite(11, HIGH); 
  }
  
  if(m2sig < 0){
    digitalWrite(10, LOW);
    m2sig = -m2sig;
  } else {
    digitalWrite(10, HIGH);
  }
}    

  unsigned long currentMicros = micros();
  if(m1sig != 0 && (unsigned long) (currentMicros - previousMicros1) >= (unsigned long) m1sig){
    digitalWrite(13,!digitalRead(13));
    previousMicros1=currentMicros;
  }
  
  if(m2sig !=  0 && (unsigned long) (currentMicros - previousMicros2) >= (unsigned long) m2sig){
    digitalWrite(12,!digitalRead(12));
    previousMicros2=currentMicros;  
  }
}