#include <QTRSensors.h>
#include <LiquidCrystal.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   8     // emitter is controlled by digital pin 2

// Defining the LEDs on the MSP423

#define RLED RED_LED        
#define GLED GREEN_LED
#define BLED BLUE_LED
#define LED 78

// Defining the LCD pins

const int rs = 36, en = 35, d4 = 34, d5 = 33, d6 = 32, d7 = 31;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Defining the Ultrasonic variables

int echo = 30, triger = 10, distance = 200;
unsigned long duration;
unsigned long echotime;
unsigned long startTime;

// Defining the controller variables

int P = 0, I = 0, D = 0, previousError = 0;
float Kp = 7, Kd = 40, Ki = 0;
float error = 0;
char mode = 'L';
int counter = 0;

boolean sign = false;
boolean LEDState = HIGH;
int cnt = 0;
int x = 4, y = 2, absx;
boolean z = true;
boolean ready = true;

// Defining the QTR-8RC pins and variables

QTRSensorsRC qtrrc((unsigned char[]) {6, 29, 28, 27, 26, 25, 24, 23},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

//WIFI Variables


#ifndef __CC3200R1M1RGC__
#include <SPI.h>                // Do not include SPI for CC3200 LaunchPad
#endif
#include <WiFi.h>


// Define structures and classes


// Define variables and constants
char wifi_name[] = "Robateria";
char wifi_password[] = "robateria1234";
WiFiServer myServer(80);
uint8_t oldCountClients = 0;
uint8_t countClients = 0;


/********************************************************************************************************************************/
////////////////////////////////////////////////////////FUNCTIONS/////////////////////////////////////////////////////////////////


void ultrasonicInit(){
  pinMode(triger, OUTPUT);
  pinMode(echo, INPUT);
}
void motorInit(){
  pinMode(40, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
}

// Ultrasonic sensor checking the distance return the distance in mm.4
// The distance is between the sensor and obstacle  

void checkDistance(){
  if(ready){
    digitalWrite(triger, HIGH);
    delayMicroseconds(10);
    digitalWrite(triger, LOW);
    echotime = micros();
    ready = false;
  }
  cnt++;
  if(cnt > 20){
    cnt = 0;
    ready = true;
  }
  distance = duration * 0.034/2;
}

// The intrupt function related to the Ultrasonic's echo pin 

void ult(){
    ready = true;
    duration = micros() - echotime;
}

// Motor Control functions

void motorBackward(int right, int left){
  digitalWrite(11, HIGH); 
  digitalWrite(12, HIGH);
  analogWrite(40, right);
  analogWrite(39, left);
}

void motorForward(int right, int left){
  digitalWrite(11, LOW); 
  digitalWrite(12, LOW);
  analogWrite(40, right);
  analogWrite(39, left); 
}

void motorTurnRight(int x){
  digitalWrite(11, LOW); 
  digitalWrite(12, HIGH);
  analogWrite(40, x);
  analogWrite(39, x);
}

void motorTurnLeft(int x){
  digitalWrite(11, HIGH); 
  digitalWrite(12, LOW);
  analogWrite(40, x);
  analogWrite(39, x);
}

void motorBreak(){
    digitalWrite(40, 0);
    digitalWrite(39, 0);  
}

// Defining the errors related to the PID controller
// Also trigerring the ultrasonic sensor

int lineError(){
  qtrrc.read(sensorValues);
  //checkDistance();           //Uncomment it if you want to activate the ultrasonic
  
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] > 2000 && sensorValues[4] > 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error = 0;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] > 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error = 1;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] > 2000 && sensorValues[5] > 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error = 2;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] > 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error = 3;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] > 2000 && sensorValues[6] > 2000 && sensorValues[7] < 2000){
    error = 4;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] > 2000 && sensorValues[7] < 2000){
    error = 5;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] > 2000 && sensorValues[7] > 2000){
    error = 6;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] > 2000){
    error = 7;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] > 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error =- 1;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] > 2000 && sensorValues[3] > 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error =- 2;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] > 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error =- 3;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] > 2000 && sensorValues[2] > 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error =- 4;
  }
  if(sensorValues[0] < 2000 && sensorValues[1] > 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error =- 5;
  }
  if(sensorValues[0] > 2000 && sensorValues[1] > 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error =- 6;
  }
  if(sensorValues[0] > 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    error =- 7;
  }
  return error;
}

int calculatePID()
{
  int error = lineError();
  P = error;
  I = I + error;
  D = error - previousError;
  int PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
  return PIDvalue;
}

void motorPIDControl(int x){
  
    int PIDvalue = calculatePID();
    int leftMotorSpeed = x - PIDvalue; //Adjust the speed
    int rightMotorSpeed = x + PIDvalue; //Adjust the speed
  if(distance > 10){
    motorForward(rightMotorSpeed, leftMotorSpeed);
  }else{
    motorBreak(); 
  }
}

// Check the signs on the ground. T stands for a line which is perpandicular to the line that robot is following. 
//N stands for No line and L stands for normal line
char checkSign(){
  qtrrc.read(sensorValues);
  if(sensorValues[0] > 2000 && sensorValues[1] > 2000 && sensorValues[6] > 2000 && sensorValues[7] > 2000){
    return 'T' ;
  }
  else if(sensorValues[0] < 2000 && sensorValues[1] < 2000 && sensorValues[2] < 2000 && sensorValues[3] < 2000 && sensorValues[4] < 2000 && sensorValues[5] < 2000 && sensorValues[6] < 2000 && sensorValues[7] < 2000){
    return 'N' ;
  }else {return 'L';}
}

// Turn Right till the robot finds the new desired line
void turnR(){
  int counter = 0;
    boolean state = true, lastState = true;
    motorTurnRight(80);
    while(counter < 3){
     qtrrc.read(sensorValues);
     if(sensorValues[0] > 2000){
      state = true;
      }else{
      state = false;
       }
      if(state ^ lastState){
        counter++;
       }
        lastState = state;
     }
}

// Turn Left till the robot finds the new desired line
void turnL(){
  int counter = 0;
  boolean state = true, lastState = true;
  motorTurnLeft(80);
    while(counter < 3){
      qtrrc.read(sensorValues);
      if(sensorValues[7] > 2000){
        state = true;
      }else{
        state = false;
       }
      if(state ^ lastState){
        counter++;
      }
        lastState = state;
    }
}

// Turn Right till the robot finds the line that it was following before
void turnB(){
  int counter = 0;
  boolean state = true, lastState = false;
  motorTurnLeft(60);
    while(counter < 4){
      qtrrc.read(sensorValues);
      if(sensorValues[4] > 2200){
        state = true;
      }else{
        state = false;
       }
      if(state ^ lastState){
        counter++;
      }
        lastState = state;
    }
}

// Check if the robot pass through any perpendicular line to the desired line that robot is following

boolean passLine(){
  mode = checkSign();
  int counter = 0;
  if(mode == 'T'){
    boolean state = true, lastState = true;
    while(counter < 1){
      qtrrc.read(sensorValues);
      if(sensorValues[0] > 2000 && sensorValues[1] > 2000 && sensorValues[2] > 2000 && sensorValues[3] > 2000 && sensorValues[4] > 2000 && sensorValues[5] > 2000 && sensorValues[6] > 2000 && sensorValues[7] > 2000){
        state = true;
        }else{
          state = false;
        }
        if(state ^ lastState){
          counter++;
        }
        lastState = state;
      }
      LEDState = !LEDState;
      digitalWrite(RLED, LEDState);
      digitalWrite(GLED, LEDState);
      digitalWrite(BLED, LEDState);
      return true;
      }
  return false;  
}

// Going to the correspond coordinate

void findRout(){
  int xcounter = 0, ycounter = 0, zcounter = 0;
  while(ycounter <= y){
    if(passLine()){
      ycounter++;
    }
    motorPIDControl(100);
  }
    if(x > 0){
      turnR();
      motorPIDControl(100);
      absx = x;
    }else{
      turnL();
      motorPIDControl(100);
      absx = -x;
      z = !z;
    }
  while(xcounter < absx){
      if(passLine()){
        xcounter++;
      }
      motorPIDControl(100);
  }
  if(z){
    turnL();
    }else{
      turnR();
      }
  while(zcounter < 1){
    if(passLine()){
      zcounter++;
    }
    motorPIDControl(60);
  }
  motorBreak();
  lcd.setCursor(0, 1);
  lcd.print("Congrats");
}

// Returning Home

void returnHome(){
  lcd.clear();
  lcd.print("Returning Home");
  int ycounter = 0, xcounter = 0, zcounter = 0;
  turnB();
  while(zcounter < 1){
    if(passLine()){
      zcounter++;
      }
    motorPIDControl(80);
  }
  if(z){
    turnR();
    }else{
      turnL();
      }
  while(xcounter < absx){
    if(passLine()){
      xcounter++;
    }
    motorPIDControl(100);
  }
  if(x > 0){
    turnL();
  }else{
    turnR();
  }
  while(ycounter <= y){
    if(passLine()){
      ycounter++;
    }
    motorPIDControl(100);
  }
  motorBreak();
  delay(1000);
  turnB();
  motorBreak();
  mode = 'L';
}

void setup() {

  //Initializing Serial
  Serial.begin(115200);

  
  //Initializing WIFI
    delay(500);
    
    Serial.println("*** LaunchPad CC3200 WiFi Web-Server in AP Mode");
    
    // Start WiFi and create a network with wifi_name as the network name
    // with wifi_password as the password.
    Serial.print("Starting AP... ");
    WiFi.beginNetwork(wifi_name, wifi_password);
    while (WiFi.localIP() == INADDR_NONE)
    {
        // print dots while we wait for the AP config to complete
        Serial.print('.');
        delay(300);
    }
    Serial.println("DONE");
    
    Serial.print("LAN name = ");
    Serial.println(wifi_name);
    Serial.print("WPA password = ");
    Serial.println(wifi_password);
    IPAddress ip = WiFi.localIP();
    Serial.print("Webserver IP address = ");
    Serial.println(ip);
    
    Serial.print("Web-server port = ");
    myServer.begin();                           // start the web server on port 80
    Serial.println("80");
    Serial.println();
    lcd.begin(16, 2);
    
  // Print a message to the LCD.
  lcd.begin(16, 2);
  lcd.print("Hi From Robateria!");
  
  // Setting the backlight of the LED
  pinMode(38, OUTPUT);
  analogWrite(38, 125);
 
  // Initializing the Ultrasonic pins
  pinMode(triger, OUTPUT);
  pinMode(echo, INPUT);

  //Initializing LEDs
  pinMode(LED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  pinMode(RLED, OUTPUT);

  //Initializing the Ultrasonic Sensor
  attachInterrupt(echo, ult, FALLING);
  ultrasonicInit();
  // Initializing the motor  
  motorInit();
}

void loop() {

  // LCD

  lcd.clear();
  lcd.print("Hi From Robateria!");
  
  //WIFI Setting
  
  countClients = WiFi.getTotalDevices();
    
    // Did a client connect/disconnect since the last time we checked?
    if (countClients != oldCountClients)
    {
        if (countClients > oldCountClients)
        {  // Client connect
            //            digitalWrite(RED_LED, !digitalRead(RED_LED));
            Serial.println("Client connected to AP");
            for (uint8_t k = 0; k < countClients; k++)
            {
                Serial.print("Client #");
                Serial.print(k);
                Serial.print(" at IP address = ");
                Serial.print(WiFi.deviceIpAddress(k));
                Serial.print(", MAC = ");
                Serial.println(WiFi.deviceMacAddress(k));
                Serial.println("CC3200 in AP mode only accepts one client.");
            }
        }
        else
        {  // Client disconnect
            //            digitalWrite(RED_LED, !digitalRead(RED_LED));
            Serial.println("Client disconnected from AP.");
            Serial.println();
        }
        oldCountClients = countClients;
    }
    
    WiFiClient myClient = myServer.available();
    
    if (myClient)
    {                             // if you get a client,
        Serial.println(". Client connected to server");           // print a message out the serial port
        char buffer[150] = {0};                 // make a buffer to hold incoming data
        int8_t i = 0;
        while (myClient.connected())
        {            // loop while the client's connected
            if (myClient.available())
            {             // if there's bytes to read from the client,
                char c = myClient.read();             // read a byte, then
                Serial.write(c);                    // print it out the serial monitor
                if (c == '\n') {                    // if the byte is a newline character
                    
                    // if the current line is blank, you got two newline characters in a row.
                    // that's the end of the client HTTP request, so send a response:
                    if (strlen(buffer) == 0)
                    {
                        // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                        // and a content-type so the client knows what's coming, then a blank line:
                        myClient.println("HTTP/1.1 200 OK");
                        myClient.println("Content-type:text/html");
                        myClient.println();
                        
                        // the content of the HTTP response follows the header:
                        myClient.println("<html><head><title>ROBATERIA</title></head><body align=center>");
                        myClient.println("<h1 align=center><font color=\"red\">Please choose your book</font></h1>");
                        myClient.print("<button onclick=\"location.href='/1'\">Computernetze</button>");
                        myClient.println(" <button onclick=\"location.href='/2'\">Kunststoffe</button><br>");
                        myClient.print("<button onclick=\"location.href='/3'\">Komplexitatsheorie</button>");
                        myClient.println(" <button onclick=\"location.href='/4'\">Embedded Systems</button><br>");
                        myClient.print("<button onclick=\"location.href='/5'\">Advanced Physics</button>");
                        myClient.println(" <button onclick=\"location.href='/6'\">Computer Graphics</button><br>");
                        myClient.print("<button onclick=\"location.href='/7'\">Change Management</button>");
                        myClient.println(" <button onclick=\"location.href='/8'\">C++ Introduction</button><br>");
                        myClient.print("<button onclick=\"location.href='/9'\">BWL kompakt</button>");
                        myClient.println(" <button onclick=\"location.href='/T0'\">Grundlagen Informatik</button><br>");
                        myClient.print("<button onclick=\"location.href='/T1'\">Spieltheorie</button>");
                        myClient.println(" <button onclick=\"location.href='/T2'\">Diskrete Strukturen</button><br>");
                        myClient.print("<button onclick=\"location.href='/T3'\">Grundkurs Java</button>");
                        myClient.println(" <button onclick=\"location.href='/T4'\">Rechneratchitekrur</button><br>");
                        myClient.print("<button onclick=\"location.href='/T5'\">Start-up Skills</button>");
                        myClient.println(" <button onclick=\"location.href='/T6'\">Operation Mangement</button><br>");

                        
                        // The HTTP response ends with another blank line:
                        myClient.println();
                        // break out of the while loop:
                        break;
                    }
                    else
                    {      // if you got a newline, then clear the buffer:
                        memset(buffer, 0, 150);
                        i = 0;
                    }
                }
                else if (c != '\r')
                {    // if you got anything else but a carriage return character,
                    buffer[i++] = c;      // add it to the end of the currentLine
                }
                
                Serial.println();
                String text = buffer;
                // Check to see what is the client request:
                if (text.endsWith("GET /1"))
                {
                    x = -2;
                    y = 1;
                    z = true;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /2"))
                {
                    x = -2;
                    y = 1;
                    z = false;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /3"))
                {
                    x = -1;
                    y = 1;
                    z = true;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /4"))
                {
                    x = -1;
                    y = 1;
                    z = false;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /5"))
                {
                    x = 1;
                    y = 1;
                    z = true;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /6"))
                {
                    x = 1;
                    y = 1;
                    z = false;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /7"))
                {
                    x = 2;
                    y = 1;
                    z = true;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /8"))
                {
                    x = 2;
                    y = 1;
                    z = false;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /9"))
                {
                    x = -2;
                    y = 2;
                    z = true;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /T0"))
                {
                    x = -2;
                    y = 2;
                    z = false;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /T1"))
                {
                    x = -1;
                    y = 2;
                    z = true;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /T2"))
                {
                    x = -1;
                    y = 2;
                    z = false;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /T3"))
                {
                    x = 1;
                    y = 2;
                    z = true;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /T4"))
                {
                    x = 1;
                    y = 2;
                    z = false;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /T5"))
                {
                    x = 2;
                    y = 2;
                    z = true;
                    findRout();
                    delay(1000);
                    returnHome();
                }
                if (text.endsWith("GET /T6"))
                {
                    x = 2;
                    y = 2;
                    z = false;
                    findRout();
                    delay(1000);
                    returnHome();
                }
            }
        }
        
        // close the connection:
        myClient.stop();
        Serial.println(". Client disconnected from server");
        Serial.println();    
    }
}
