#include <ESP32Servo.h>
#include <Encoder.h>

#include <Wire.h>
#include <DS3231.h>

#include <Button.h>



// Servo pins.
const int SERVO_LIFT_PIN = 32;
const int SERVO_LEFT_PIN = 33;
const int SERVO_RIGHT_PIN = 25;


// Encoder pins.
const int ENCODER_L_PIN1 = 0;
const int ENCODER_L_PIN2 = 4;
const int ENCODER_L_BTN_PIN = 15;

const int ENCODER_R_PIN1 = 16;
const int ENCODER_R_PIN2 = 17;
const int ENCODER_R_BTN_PIN = 26;

const int ENCODER_LIFT_PIN1 = 18;
const int ENCODER_LIFT_PIN2 = 19;
const int ENCODER_LIFT_INIT = 70;


// When in calibration mode, adjust the following factors until the servos move exactly 90 degrees.
const int SERVO_LEFT_FACTOR = 630;
const int SERVO_RIGHT_FACTOR = 640;


const int Z_OFFSET = 230;  // Adjust until pen head is in correct height from surface in all lifts (higher is lower).
const int LIFT0 = 1110 + Z_OFFSET;  // On drawing surface.
const int LIFT1 = 925 + Z_OFFSET;   // Between numbers.
const int LIFT2 = 735 + Z_OFFSET;   // Going towards sweeper.


// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel either to the X or Y axis.
const int SERVO_LEFT_NULL = 1950;
const int SERVO_RIGHT_NULL = 815;

const int WISHY = 3;  // Offset of the Y coordinats of the plate-wisher.
const int LIFT_SPEED = 2000;  // Speed of liftimg arm (higher is slower).


// Length of arms.
const float L1 = 35;
const float L2 = 55.1;
const float L3 = 13.2;
const float L4 = 45;

// Origin points of left and right servos.
const int O1X = 24;
const int O1Y = -25;
const int O2X = 49;
const int O2Y = -25;

// Home coordinates, where the eraser is.
const volatile double HOME_X = 72.2;
const volatile double HOME_Y = 45.5;


const float SCALE = 0.9;  // Font scale.
const int MOVE_DELAY_MS = 2;  // Delay between each servo angle.



int servoLift = 1500;

Servo servo_lift;
Servo servo_left;
Servo servo_right;


Encoder encoderL(ENCODER_L_PIN1, ENCODER_L_PIN2);
int encPosL;
int encLastPosL = -1;

Encoder encoderR(ENCODER_R_PIN1, ENCODER_R_PIN2);
int encPosR;
int encLastPosR = -1;

Encoder encoderLift(ENCODER_LIFT_PIN1, ENCODER_LIFT_PIN2);
int encPosLift;
int encLastPosLift = -1;

RTClib rtc;

Button leftEncoderBtn(ENCODER_L_BTN_PIN, 10);  // 10 is debounce in ms.
Button rightEncoderBtn(ENCODER_R_BTN_PIN, 10);


volatile double lastX = HOME_X;  // 75;
volatile double lastY = HOME_Y;  // 47.5;


struct TimeHHMM {
    int hh;
    int mm;
};

TimeHHMM currTime;  // Keeps track of the current time.


// 0: Calibration, 1: Encoder control, 2: Auto.
int currentMode = 0;


void setup() {
    Serial.begin(9600);
    Wire.begin();

    servo_lift.attach(SERVO_LIFT_PIN);
    servo_left.attach(SERVO_LEFT_PIN);
    servo_right.attach(SERVO_RIGHT_PIN);


    encoderL.write(HOME_X * 4);
    encoderR.write(HOME_Y * 4);
    encoderLift.write(ENCODER_LIFT_INIT * 4);

    leftEncoderBtn.begin();
    rightEncoderBtn.begin();

    if(currentMode != 0) {
        lift(LIFT2);
        drawTo(HOME_X, HOME_Y);
        lift(LIFT0);
    }

    delay(2000);
}


void loop() {

    handleEncoderBtns();
    
    // Calibration.
    if(currentMode == 0) {
        drawTo(-3, 29.2);
        delay(500);
        drawTo(74.1, 28);
        delay(500);
    }
    
    // Manual control.
    else if(currentMode == 1) {
        if(updateEncoder(encoderL, encPosL, encLastPosL)) {
            set_XY(encPosL, encPosR);
        }
        
        if(updateEncoder(encoderR, encPosR, encLastPosR)) {
            set_XY(encPosL, encPosR);
        }
        
        if(updateEncoder(encoderLift, encPosLift, encLastPosLift)) {
            servo_lift.write(encPosLift);
        }
    }

    // Auto mode.
    else if(currentMode == 2) {
        drawCurrentTime(SCALE);
        
        //drawTime(4, 3, 2, 1, SCALE);
        delay(1000);
    }
}


void handleEncoderBtns() {
    
    if(leftEncoderBtn.pressed()) {
        currentMode += 1;
        
        if(currentMode == 3) {
            currentMode = 0;
        }

        Serial.print("Current mode: ");
        Serial.println(currentMode);
    }

    if(rightEncoderBtn.pressed()) {
        erase();
    }
}


bool updateEncoder(Encoder encoder, int &encPos, int &encLastPos) {
    encPos = encoder.read() / 4;

    if(encPos != encLastPos) {
        encLastPos = encPos;
        return true;
    } else {
        return false;
    }
}


void drawCurrentTime(float scale1) {
    // Draw current time.
    
    DateTime currDateTime = rtc.now();
    TimeHHMM newTime = {currDateTime.hour(), currDateTime.minute()};

    // Check if the time have changed.
    if(newTime.hh != currTime.hh || newTime.mm != currTime.mm) {
        int h1 = newTime.hh / 10;
        int h2 = newTime.hh % 10;
        
        int m1 = newTime.mm / 10;
        int m2 = newTime.mm % 10;

        currTime = newTime;
        
        erase();
        drawTime(h1, h2, m1, m2, scale1);
    }
}


void drawTime(int h1, int h2, int m1, int m2, float scale1) {
    // Draw given time.
    goHome();

    lift(LIFT1);
    
    drawNumber(5, 25, h1, scale1);  // Digit 1
    drawNumber(19, 25, h2, scale1); // Digit 2
    
    drawColon(28, 25, scale1);
    
    drawNumber(34, 25, m1, scale1);  // Digit 3
    drawNumber(48, 25, m2, scale1);  // Digit 4
    
    goHome();
}


void goHome() {
    lift(LIFT2);  // Lift all the way up.
    drawTo(HOME_X, HOME_Y);
    lift(LIFT0);
    delay(500);
    //lift(LIFT2);
}


void drawColon(float bx, float by, float scale1) {    
    drawTo(bx + 5 * scale1, by + 15 * scale1);
    lift(LIFT0);
    bogenGZS(bx + 5 * scale1, by + 15 * scale1, 0.1 * scale1, 1, -1, 1);
    delay(10);
    
    lift(LIFT1);
    
    drawTo(bx + 5 * scale1, by + 5 * scale1);
    lift(LIFT0);
    bogenGZS(bx + 5 * scale1, by + 5 * scale1, 0.1 * scale1, 1, -1, 1);
    delay(10);
    
    lift(LIFT1);
}


void erase() {
    goHome();
    
    lift(LIFT0);  // Go down, just before doing the erase movements.
    drawTo(70, HOME_Y);
    drawTo(65-WISHY, HOME_Y);

    drawTo(65-WISHY, HOME_Y);
    drawTo(5, HOME_Y);
    drawTo(5, HOME_Y);
    drawTo(63-WISHY, 46);
    drawTo(63-WISHY, 42);

    drawTo(5, 42);
    drawTo(5, 38);
    drawTo(63-WISHY, 38);
    drawTo(63-WISHY, 34);

    drawTo(5, 34);
    drawTo(5, 29);
    drawTo(6, 29);
    drawTo(65-WISHY, 26);

    drawTo(5, 26);
    drawTo(60-WISHY, 40);

    drawTo(HOME_X, HOME_Y);
    lift(LIFT0);
}


void drawNumber(float bx, float by, int num, float scale1) {
    switch (num) {
        case 0:
            drawTo(bx + 12 * scale1, by + 6 * scale1);
            lift(LIFT0);
            bogenGZS(bx + 7 * scale1, by + 10 * scale1, 10 * scale1, -0.8, 6.7, 0.5);
            lift(LIFT1);
            break;
            
        case 1:
            drawTo(bx + 3 * scale1, by + 15 * scale1);
            lift(LIFT0);
            drawTo(bx + 10 * scale1, by + 20 * scale1);
            drawTo(bx + 10 * scale1, by + 0 * scale1);
            lift(LIFT1);
            break;
        
        case 2:
            drawTo(bx + 2 * scale1, by + 12 * scale1);
            lift(LIFT0);
            bogenUZS(bx + 8 * scale1, by + 14 * scale1, 6 * scale1, 3, -0.8, 1);
            drawTo(bx + 1 * scale1, by + 0 * scale1);
            drawTo(bx + 12 * scale1, by + 0 * scale1);
            lift(LIFT1);
            break;
        
        case 3:
            drawTo(bx + 2 * scale1, by + 17 * scale1);
            lift(LIFT0);
            bogenUZS(bx + 5 * scale1, by + 15 * scale1, 5 * scale1, 3, -2, 1);
            bogenUZS(bx + 5 * scale1, by + 5 * scale1, 5 * scale1, 1.57, -3, 1);
            lift(LIFT1);
            break;
        
        case 4:
            drawTo(bx + 10 * scale1, by + 0 * scale1);
            lift(LIFT0);
            drawTo(bx + 10 * scale1, by + 20 * scale1);
            drawTo(bx + 2 * scale1, by + 6 * scale1);
            drawTo(bx + 12 * scale1, by + 6 * scale1);
            lift(LIFT1);
            break;
        
        case 5:
            drawTo(bx + 2 * scale1, by + 5 * scale1);
            lift(LIFT0);
            bogenGZS(bx + 5 * scale1, by + 6 * scale1, 6 * scale1, -2.5, 2, 1);
            drawTo(bx + 5 * scale1, by + 20 * scale1);
            drawTo(bx + 12 * scale1, by + 20 * scale1);
            lift(LIFT1);
            break;
            
        case 6:
            drawTo(bx + 2 * scale1, by + 10 * scale1);
            lift(LIFT0);
            bogenUZS(bx + 7 * scale1, by + 6 * scale1, 6 * scale1, 2, -4.4, 1);
            drawTo(bx + 11 * scale1, by + 20 * scale1);
            lift(LIFT1);
            break;
        
        case 7:
            drawTo(bx + 2 * scale1, by + 20 * scale1);
            lift(LIFT0);
            drawTo(bx + 12 * scale1, by + 20 * scale1);
            drawTo(bx + 2 * scale1, by + 0);
            lift(LIFT1);
            break;
        
        case 8:
            drawTo(bx + 5 * scale1, by + 10 * scale1);
            lift(LIFT0);
            bogenUZS(bx + 5 * scale1, by + 15 * scale1, 5 * scale1, 4.7, -1.6, 1);
            bogenGZS(bx + 5 * scale1, by + 5 * scale1, 5 * scale1, -4.7, 2, 1);
            lift(LIFT1);
            break;
        
        case 9:
            drawTo(bx + 9 * scale1, by + 11 * scale1);
            lift(LIFT0);
            bogenUZS(bx + 7 * scale1, by + 15 * scale1, 5 * scale1, 4, -0.5, 1);
            drawTo(bx + 5 * scale1, by + 0);
            lift(LIFT1);
            break;
  }
}


void lift(int lift) {
    if (servoLift >= lift) {
        while (servoLift >= lift) {
            servoLift--;
            servo_lift.writeMicroseconds(servoLift);                
            delayMicroseconds(LIFT_SPEED);
        }
    }
    else {
        while (servoLift <= lift) {
            servoLift++;
            servo_lift.writeMicroseconds(servoLift);
            delayMicroseconds(LIFT_SPEED);
        }
    }
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
    float inkr = -0.05;
    float count = 0;
    
    do {
        drawTo(sqee * radius * cos(start + count) + bx,
        radius * sin(start + count) + by);
        count += inkr;
    }
    while ((start + count) > ende);
}


void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
    float inkr = 0.05;
    float count = 0;
    
    do {
        drawTo(sqee * radius * cos(start + count) + bx,
        radius * sin(start + count) + by);
        count += inkr;
    }
    while ((start + count) <= ende);
}


void drawTo(double pX, double pY) {
    double dx, dy, c;
    int i;
    
    // dx dy of new point
    dx = pX - lastX;
    dy = pY - lastY;
    //path lenght in mm, times 4 equals 4 steps per mm
    c = floor(7 * sqrt(dx * dx + dy * dy));
    
    if (c < 1) c = 1;
    
    for (i = 0; i <= c; i++) {
        // draw line point by point
        set_XY(lastX + (i * dx / c), lastY + (i * dy / c));
        delay(MOVE_DELAY_MS);
    }
    
    lastX = pX;
    lastY = pY;
}


double return_angle(double a, double b, double c) {
    // cosine rule for angle between c and a
    return acos((a * a + c * c - b * b) / (2 * a * c));
}


void set_XY(double Tx, double Ty) {
    delay(1);
    double dx, dy, c, a1, a2, Hx, Hy;
    
    // calculate triangle between pen, servoLeft and arm joint
    // cartesian dx/dy
    dx = Tx - O1X;
    dy = Ty - O1Y;
    
    // polar lemgth (c) and angle (a1)
    c = sqrt(dx * dx + dy * dy); // 
    a1 = atan2(dy, dx); //
    a2 = return_angle(L1, L2, c);
    
    servo_left.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVO_LEFT_FACTOR) + SERVO_LEFT_NULL));
    
    // calculate joinr arm point for triangle of the right servo arm
    a2 = return_angle(L2, L1, c);
    Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
    Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);
    
    // calculate triangle between pen joint, servoRight and arm joint
    dx = Hx - O2X;
    dy = Hy - O2Y;
    
    c = sqrt(dx * dx + dy * dy);
    a1 = atan2(dy, dx);
    a2 = return_angle(L1, L4, c);
    
    servo_right.writeMicroseconds(floor(((a1 - a2) * SERVO_RIGHT_FACTOR) + SERVO_RIGHT_NULL));
}
