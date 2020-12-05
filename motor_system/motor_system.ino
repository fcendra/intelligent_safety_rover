#include <Wire.h>
#include <Adafruit_SSD1331.h>
//Mega2560 pin 20 (SDA), pin 21 (SCL)
//SPI pin
// MOSI 51
//MISO 50
//SCK 52
//SS 53
///////////////////////////////////////////////////////
//OLED Display config
#define Sclk 34
#define Mosi 33
#define Rst 37
#define Dc 36
#define Cs 35

#define BLACK           0x0000
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0 
#define WHITE           0xFFFF
#define BACKGROUND      0x0000

Adafruit_SSD1331 display = Adafruit_SSD1331(Cs, Dc, Mosi, Sclk, Rst);
///////////////////////////////////////////////////////
//Coordinates (x,y)
int x = 0;
int y = 0;
int turn_x_coordinate = 0;
int turn_y_coordinate = 0;
int face_direction_index = 0;
int array_of_face_direction[4] = {0, 1, 2, 3};  //{East, South, West, North}
///////////////////////////////////////////////////////
//Color sensor
#define colorpin_out 26  //brown
#define colorpin_s2  27  //orange
#define colorpin_s3  28  //yellow
#define colorpin_s1  29  //purple
#define colorpin_s0  30  //blue
#define colorpin_LED 31  //LED

int colorValueC = 0;int colorValueR = 0;int colorValueG = 0;int colorValueB = 0;
int colorCheckCntR = 0; int colorCheckCntG = 0;int colorCheckCntB = 0;int colorCheckCnt = 5;
int colorCnt = 0;
int color_status = 0;
///////////////////////////////////////////////////////
//Black Line
//see black tape is 1, LED off
//see white is 0, LED on
#define blackLinePinAnalogTR A12  //Top Right
#define blackLinePinTR       42   //Top Right
#define blackLinePinAnalogTL A13  //Top Left
#define blackLinePinTL       43   //Top Left
#define blackLinePinAnalogFR A14  //Far Right (near the wheel)
#define blackLinePinFR       44   //Far Right (near the wheel)
#define blackLinePinAnalogFL A15  //Far Left (near the wheel)
#define blackLinePinFL       45   //Far Left (near the wheel)

bool blackLineTR = 0;        //top right
bool blackLineTL = 0;        // top left
bool blackLineFR = 0;        //far right
bool blackLineFL = 0;        //far left
int  blackLineAnalogTR = 0;  //top right
int  blackLineAnalogTL = 0;  //top left
int  blackLineAnalogFR = 0;  //far right
int  blackLineAnalogFL = 0;  //far left
///////////////////////////////////////////////////////
//Photo Diode
#define photoPinAnalogR  A8  //Photo Right
#define photoPinR        38  //Photo Right
#define photoPinAnalogL  A9  //Photo Left
#define photoPinL        39  //Photo Left

bool photoR = 0;      //right
bool photoL = 0;      //left
int photoAnalogR = 0; //right
int photoAnalogL = 0; //left
///////////////////////////////////////////////////////
//Power detection
#define voltageValuePin A2
float current_voltageValue = 0;
float  voltageValue = 0;
///////////////////////////////////////////////////////
//Ultra Sonic
#define trigPin 13
#define echoPin 12

unsigned long UltraSonicStartTime = 0;
int UltraSonicDone = 1;
int    duration;
float  distance;
float current_distance;
///////////////////////////////////////////////////////
//Alert sensor
#define redPin 22
#define greenPin 23
#define buzzerPin 24
///////////////////////////////////////////////////////
//interrupt pin
int interruptL1 = 2;int interruptL2 = 3;
int interruptR1 = 18;int interruptR2 = 19;
///////////////////////////////////////////////////////
//motor driver LM298 control pin
int pin1R = 9;int pin2R = 8;
int pin1L = 10;int pin2L = 11;
///////////////////////////////////////////////////////
// encoder parameter
//encoderValue
long encoderValueR = 0;
long encoderValueL = 0;
///////////////////////////////////////////////////////
//Reset encoderValue
int encoderValueR_Reset = 0;
int encoderValueL_Reset = 0;
///////////////////////////////////////////////////////
//encoderValue Diff
long encoderValueR_Diff = 0;
long encoderValueL_Diff = 0;
///////////////////////////////////////////////////////
int forwardStart = 0;
int leftStart = 0;
int rightStart = 0;
int targetValue = 0;
///////////////////////////////////////////////////////
//Rooms allocation
#define inputToRPI1 46
#define inputToRPI2 47
#define inputToRPI3 48
#define inputToRPI4 49
#define receive_from_rpi 50
#define trigger_rpi 51
#define face_result 52

char Incoming_value = 10;
int combination_value = 0;
int current_room = 0;
////////////////////////////////////////////////////
//Room Combinations to be sent to RPI
void sentCombinationToRPI() {
    int combination_list[12][4] = {{1,1,1,1}, {1,1,1,0}, {1,1,0,0} ,{1,0,0,0},
                                    {0,0,0,1}, {0,0,1,1}, {0,1,1,1}, {0,1,0,1},
                                    {0,1,0,0}, {0,1,1,0}, {1,0,1,0}, {1,0,0,1}};
    int pins_list[4] = {inputToRPI1, inputToRPI2, inputToRPI3, inputToRPI4};
    for (int i = 0; i < 4; i++) {
        digitalWrite(pins_list[i], combination_list[combination_value-1][i]);
    }
}
////////////////////////////////////////////////////
//Setup program
void setup() {
    Serial3.begin(9600);
    //OLED display setup
    display.begin();
    display.fillScreen(BLACK);
//    tftPrintTest();  
    display.fillScreen(BACKGROUND);
    display.setCursor(0,0);
    //Color sensor setup
    pinMode(colorpin_LED, OUTPUT);
    pinMode(colorpin_out, INPUT);
    pinMode(colorpin_s0, OUTPUT);
    pinMode(colorpin_s1, OUTPUT);
    pinMode(colorpin_s2, OUTPUT);
    pinMode(colorpin_s3, OUTPUT);
    // turn on LED
    digitalWrite(colorpin_LED, 1);
    //100% freq
    digitalWrite(colorpin_s0, 1);
    digitalWrite(colorpin_s1, 1);
    //detect red
    digitalWrite(colorpin_s2, 1);
    digitalWrite(colorpin_s3, 0);
    /////////////////////////////////////////
    //Black Line sensor
    pinMode(blackLinePinTR, INPUT);pinMode(blackLinePinTL, INPUT);
    pinMode(blackLinePinFR, INPUT);pinMode(blackLinePinFL, INPUT);
    /////////////////////////////////////////
    //Photo sensor
    pinMode(photoPinR, INPUT);
    pinMode(photoPinL, INPUT);
    /////////////////////////////////////////
    //Ultra Sonic sensor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    /////////////////////////////////////////
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    /////////////////////////////////////////
    //Motor
    pinMode(pin1L, OUTPUT);pinMode(pin2L, OUTPUT);
    pinMode(pin1R, OUTPUT);pinMode(pin2R, OUTPUT);
    /////////////////////////////////////////
    //Encoder
    //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
    pinMode(interruptL1, INPUT_PULLUP);pinMode(interruptL2, INPUT_PULLUP);
    pinMode(interruptR1, INPUT_PULLUP);pinMode(interruptR2, INPUT_PULLUP);
    //setup interrupt
    attachInterrupt(digitalPinToInterrupt(interruptL1), countL, FALLING);
    attachInterrupt(digitalPinToInterrupt(interruptR1), countR, FALLING);
    //input to raspberry pi
    pinMode(inputToRPI1, OUTPUT);
    pinMode(inputToRPI2, OUTPUT);
    pinMode(inputToRPI3, OUTPUT);
    pinMode(inputToRPI4, OUTPUT);
    //Wait for android output
    while (true) {
        digitalWrite(greenPin, HIGH);
        if(Serial3.available() > 0) {
            Incoming_value = Serial3.read()-'0';             //Read the incoming data and store it into variable Incoming_value
            Serial.print(Incoming_value);
            Serial.print("\n");
            combination_value = Incoming_value;   //Convert char string to integer
            digitalWrite(greenPin, LOW);
            break;
        }    
    }
    sentCombinationToRPI(); //Send combinations to Raspbery Pi
    delay(10000);
}
//////////////////////////////////////////////////////
//Display coordinate
void displayCoordinate(int x, int y) {
    display.fillScreen(BLACK);
    display.setTextColor(MAGENTA);  
    display.setTextSize(1.7);
    display.setCursor(0, 5);
    display.println("Coordinate:");
    display.setTextSize(2); 
    display.setCursor(15, 30);
    display.setTextColor(WHITE); 
    display.print("(");
    display.setCursor(30, 30);
    display.setTextColor(WHITE); 
    display.print(x);
    display.setCursor(45, 30);
    display.setTextColor(WHITE); 
    display.print(",");
    display.setCursor(60, 30);
    display.setTextColor(WHITE);
    display.println(y);
    display.setCursor(75, 30);
    display.setTextColor(WHITE); 
    display.print(")");
}
//////////////////////////////////////////////////////
//Switch statement to update coordinate
void addCoordinateValue() {
    switch (array_of_face_direction[face_direction_index])
    {
    case 0: //East
        x++;
        break;
    case 1: //South
        y--;
        break;
    case 2: //West
        x--;
        break;
    case 3: //North
        y++;
        break;
    default:
        break;
    }
}
//////////////////////////////////////////////////////
//Coordinates calculation
void coordinate(int FR, int FL){
    bool far_left = false;
    bool far_right = false;
    if (51 < FL < 600) {far_left = true;}
    if (51 < FR < 600) {far_right = true;}
    if (FL <= 50) {far_left = false;}
    if (FR <= 50) {far_right = false;}
    if (far_left == true && far_right == true) {addCoordinateValue();}
}
//////////////////////////////////////////////////////
//Input routine
void InputCapture() {
    //black
    blackLineTR = digitalRead(blackLinePinTR);blackLineTL = digitalRead(blackLinePinTL);
    blackLineFR = digitalRead(blackLinePinFR);blackLineFL = digitalRead(blackLinePinFL);
    blackLineAnalogTR = analogRead(blackLinePinAnalogTR);blackLineAnalogTL = analogRead(blackLinePinAnalogTL);
    blackLineAnalogFR = analogRead(blackLinePinAnalogFR);blackLineAnalogFL = analogRead(blackLinePinAnalogFL);
    //photo diode
    photoR     = digitalRead(photoPinR);photoL     = digitalRead(photoPinL);
    photoAnalogR = analogRead(photoPinAnalogR);photoAnalogL = analogRead(photoPinAnalogL);
    //Power Sensor
    voltageValue = analogRead(voltageValuePin);
    voltageValue = (25 * voltageValue / 1024)*1.25;
}
//////////////////////////////////////////////////////
//Motor control
void MotorControl(int color_status) {
    if (color_status == 1){
        //Turn right 90 degree
        if (rightStart == 0){
            stopCar();
//            if (x == 3 && y == 2) {targetValue = 15;}
            targetValue = 27;
            encoderValueL = 0;
            encoderValueR = 0;
            rightStart = 1;
        }
        while (abs(encoderValueL) < targetValue || abs(encoderValueR) < targetValue) {turnCarOnsiteR();}
        stopCar();
        rightStart = 0;
        face_direction_index = (face_direction_index == 3) ? 0 : face_direction_index + 1;
    }
    else if (color_status == 2) {
        //Turn left 90 degree
        if (leftStart == 0){
            stopCar();
            targetValue = 37;
            encoderValueL = 0;
            encoderValueR = 0;
            leftStart = 1;
        }
        while (abs(encoderValueR) < targetValue || abs(encoderValueL) < targetValue) {turnCarOnsiteL();}
        stopCar();
        rightStart = 0;
        face_direction_index = (face_direction_index == 0) ? 3 : face_direction_index - 1;
    }
    else if (color_status == 0){
        //forward 100 click
        if (forwardStart == 0) {
            stopCar();
            targetValue = 500;
            encoderValueL = 0;
            encoderValueR = 0;
            forwardStart = 1;
        }
        if (encoderValueL > targetValue || encoderValueR > targetValue) {
            stopCar();
            forwardStart = 0;
        }
        else {
            if (encoderValueL > encoderValueR) {turnCarL();}
            else if (encoderValueR > encoderValueL) {turnCarR();}
            else {forwardCar();}
        }
    }
}
//////////////////////////////////////////////////////
//Interrupt subroutine
void countL() {
    if (digitalRead(interruptL2)) {encoderValueL--;}
    else {encoderValueL++;}
}
void countR() {
    if (digitalRead(interruptR2)) {encoderValueR++;}
    else {encoderValueR--;}
}
//////////////////////////////////////////////////////
//Motor subrountine
void turnCarL() {digitalWrite(pin1R, 1);digitalWrite(pin2R, 0);digitalWrite(pin1L, 0);digitalWrite(pin2L, 0);}
void turnCarR() {digitalWrite(pin1R, 0);digitalWrite(pin2R, 0);digitalWrite(pin1L, 1);digitalWrite(pin2L, 0);}
void turnCarOnsiteL() {digitalWrite(pin1R, 1);digitalWrite(pin2R, 0);digitalWrite(pin1L, 0);digitalWrite(pin2L, 1);}
void turnCarOnsiteR() {digitalWrite(pin1R, 0);digitalWrite(pin2R, 1);digitalWrite(pin1L, 1);digitalWrite(pin2L, 0);}
void forwardCar() {digitalWrite(pin1R, 1);digitalWrite(pin2R, 0);digitalWrite(pin1L, 1);digitalWrite(pin2L, 0);}
void backwardCar() {digitalWrite(pin1R, 0);digitalWrite(pin2R, 1);digitalWrite(pin1L, 0);digitalWrite(pin2L, 1);}
void stopCar() {digitalWrite(pin1R, 0);digitalWrite(pin2R, 0);digitalWrite(pin1L, 0);digitalWrite(pin2L, 0);}
void forwardRightWheel() {digitalWrite(pin1R, 1);digitalWrite(pin2R, 0);}
void forwardLeftWheel() {digitalWrite(pin1L, 1);digitalWrite(pin2L, 0);}
void backwardRightWheel() {digitalWrite(pin1R, 0);digitalWrite(pin2R, 1);}
void backwardLeftWheel() {digitalWrite(pin1L, 0);digitalWrite(pin2L, 1);}
void stopRightWheel() {digitalWrite(pin1R, 0);digitalWrite(pin2R, 0);}
void stopLeftWheel() {digitalWrite(pin1L, 0);digitalWrite(pin2L, 0);}
//////////////////////////////////////////////////////
//UltraSonic subroutine
void UltraSonic() {
    if (UltraSonicDone) {
        UltraSonicDone = 0;
        UltraSonicStartTime = millis();
        digitalWrite(trigPin, LOW);  // Added this line
    }
    if (millis() > UltraSonicStartTime + 2) {digitalWrite(trigPin, HIGH);}
    if (millis() > UltraSonicStartTime + 12) {
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH, 3000); //set 3000ns as timout
        distance = (duration / 2) / 29.1;
        UltraSonicDone = 1;
    }
}
////////////////////////////////////////////////////\
//Alert function
void turnOnAlert() {digitalWrite(redPin, HIGH);tone(buzzerPin,2000,500);}
void turnOffAlert() {digitalWrite(redPin, LOW);}
void good_result_indication() {
    digitalWrite(greenPin, HIGH);
    delay(500);
    digitalWrite(greenPin, LOW);
}
////////////////////////////////////////////////////
//Read color sensor
void ColorInput() {
    if (colorCnt == 0) {
        //read Clear value
        colorValueC = pulseIn(colorpin_out, LOW);
        //Set Red filter
        digitalWrite(colorpin_s2, 0);
        digitalWrite(colorpin_s3, 0);
        colorCnt++;
    }
    else if (colorCnt == 1) {
        //read Red value
        colorValueR = pulseIn(colorpin_out, LOW);
        //Set Blue filter
        digitalWrite(colorpin_s2, 0);
        digitalWrite(colorpin_s3, 1);
        colorCnt++;
    }
    else if (colorCnt == 2) {
        //read Blue value
        colorValueB = pulseIn(colorpin_out, LOW);
        //Set Green filter
        digitalWrite(colorpin_s2, 1);
        digitalWrite(colorpin_s3, 1);
        colorCnt++;
    }
    else {
        //read Green value
        colorValueG = pulseIn(colorpin_out, LOW);
        //Set Clear filter
        digitalWrite(colorpin_s2, 1);
        digitalWrite(colorpin_s3, 0);
        colorCnt = 0;
    }
}
////////////////////////////////////////////////////
//Color check 
int ColorCheck() {
    if (x != turn_x_coordinate || y != turn_y_coordinate){
        //Check Red Color
        if ((150 < colorValueC && colorValueC < 300) &&
            (200 < colorValueR && colorValueR < 400) &&
            (400 < colorValueB && colorValueB < 800) &&
            (600 < colorValueG && colorValueG < 1200)) {
            colorCheckCntR++;
        } else {colorCheckCntR = 0;}
        //Continous detection before notification
        if (colorCheckCntR > colorCheckCnt) {
            Serial.print(" Red is detected. ");
            colorCheckCntR = colorCheckCnt;
            addCoordinateValue();
            turn_x_coordinate = x;
            turn_y_coordinate = y;
            return 2;
        }
        //Check Green Color
        if ((100 < colorValueC && colorValueC < 250) &&
            (300 < colorValueR && colorValueR < 600) &&
            (500 < colorValueB && colorValueB < 700) &&
            (250 < colorValueG && colorValueG < 500)) {
            colorCheckCntG++;
        } else {colorCheckCntG = 0;}
        //Continous detection before notification
        if (colorCheckCntG > colorCheckCnt) {
            Serial.print(" Green is detected. ");
            colorCheckCntG = colorCheckCnt;
            addCoordinateValue();
            turn_x_coordinate = x;
            turn_y_coordinate = y;
            return 1;
        }
    }
    return 0;
}
////////////////////////////////////////////////////
//Display vehicle's sensor current status
void PrintCurrentStatus() {
    Serial.print("|TR = ");Serial.print(blackLineTR);Serial.print(",");Serial.print(blackLineAnalogTR);
    Serial.print("|TL = ");Serial.print(blackLineTL);Serial.print(",");Serial.print(blackLineAnalogTL);
    Serial.print("|FR = ");Serial.print(blackLineFR);Serial.print(",");Serial.print(blackLineAnalogFR);
    Serial.print("|FL = ");Serial.print(blackLineFL);Serial.print(",");Serial.print(blackLineAnalogFL);
    Serial.print("|PR = ");Serial.print(photoR);Serial.print(",");Serial.print(photoAnalogR);
    Serial.print("|PL = ");Serial.print(photoL);Serial.print(",");Serial.print(photoAnalogL);
    Serial.print("|voltage = ");Serial.print(voltageValue);
    Serial.print("|ultra = ");Serial.print(distance);
    Serial.print(" encoderValueL= ");Serial.print(encoderValueL);
    Serial.print(" encoderValueR= ");Serial.print(encoderValueR);
    Serial.println("");
}
////////////////////////////////////////////////////
//Wait data from Raspberry Pi
void waiting_for_raspberyPi_response(){
    digitalWrite(inputToRPI1, 1); //Triggered Raspberry pi to start face recognition system
    while (true){ 
        stopCar();
        digitalWrite(inputToRPI1, LOW);
        if (digitalRead(receive_from_rpi) == HIGH) { //Raspberry pi has replied back
//            digitalRead(face_result);
            delay(5000);
            if(digitalRead(face_result) == LOW){
                digitalWrite(face_result, LOW);
                good_result_indication();
                current_room++;
//                digitalWrite(inputToRPI1, LOW);
                delay(4000);
                break;
            }
            else if(digitalRead(face_result) == HIGH){
                digitalWrite(face_result, LOW);
                turnOnAlert();
                turnOffAlert();
                current_room++;
//                digitalWrite(inputToRPI1, LOW);
                delay(4000);
                break;
            }
        }
    }
//    digitalWrite(inputToRPI1, LOW);
}
////////////////////////////////////////////////////
//Rooms allocation
void checkRooms(){
    switch (current_room)
    {
    case 0:
    //Room A
        if (x == 2  && y == 0) {waiting_for_raspberyPi_response();}
        break;
    case 1:
    //Room B
        if (x == 2  && y == 2) {waiting_for_raspberyPi_response();}
        break;
    case 2:
    //Room C
        if (x == 4  && y == 2) {
            waiting_for_raspberyPi_response();
            while (true) {stopCar();}
        }
        break;
    }
}
////////////////////////////////////////////////////
//Main program
void loop() {
    int current_x = x;
    int current_y = y;
    InputCapture();
    ColorInput();
    color_status = ColorCheck();
    MotorControl(color_status);
    // UltraSonic();
    // PrintCurrentStatus();
    coordinate(blackLineAnalogFR, blackLineAnalogFL);
    if (x != current_x || y != current_y) {displayCoordinate(x, y);}
    checkRooms();
    Serial.print(combination_value);
    Serial.print("\n");
}
