
// Use all serial ports on the Pico

SerialPIO Serial3(2, 3);
SerialPIO Serial4(4, 5);
SerialPIO Serial5(6, 7);
SerialPIO Serial6(10, 11);

void setup() {
  Serial.begin(115200);   // USB
  Serial1.begin(1500000);  // 0,1
  Serial2.begin(115200);  // 8,9
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
}

void setup1() {}

void loop() {
  if (Serial2.available()) {      
    Serial1.write(Serial2.read());  
    Serial1.flush();
  }

  if (Serial3.available()) {  
    Serial1.write(Serial3.read()); 
    Serial1.flush();
  }

}

void loop1() {

    if (Serial4.available()) {     
    Serial1.write(Serial4.read());  
    Serial1.flush();
  }

  if (Serial5.available()) {       
    Serial1.write(Serial5.read());
    Serial1.flush();

    if (Serial6.available()) {    
      Serial1.write(Serial6.read()); 
      Serial1.flush();
    }
  }
}