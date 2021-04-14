/*
  Main Arduino sketch for OpenCat, the bionic quadruped walking robot.
  Updates should be posted on GitHub: https://github.com/PetoiCamp/OpenCat

  Rongzhong Li
  Jan.3rd, 2021
  Copyright (c) 2021 Petoi LLC.

  This sketch may also includes others' codes under MIT or other open source
  license. Check those liscences in corresponding module test folders. Feel
  free to contact us if you find any missing references.

  The MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include <Arduino.h>

#define MAIN_SKETCH
#include "IRHandler.hpp"
#include "WriteInstinct/OpenCat.h"

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define PACKET_SIZE 42
#define OVERFLOW_THRESHOLD 128

//#if OVERFLOW_THRESHOLD>1024-1024%PACKET_SIZE-1   // when using
//(1024-1024%PACKET_SIZE) as the overflow resetThreshold, the packet buffer may
// be broken
// and the reading will be unpredictable. it should be replaced with previous
// reading to avoid jumping
#define FIX_OVERFLOW
//#endif
#define HISTORY 2
int8_t lag = 0;

// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr[3];
float yprLag[HISTORY][3];

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU

// return status after each device operation (0 = success, !0 = error)
int8_t devStatus;

// expected DMP packet size (default is 42 bytes)
uint16_t packetSize;

// count of all bytes currently in FIFO
uint16_t fifoCount;

// FIFO storage buffer
uint8_t fifoBuffer[PACKET_SIZE];

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// https://brainy-bits.com/blogs/tutorials/ir-remote-arduino
#include <IRremote.h>

IRrecv irrecv(IR_RECEIVER); // create instance of 'irrecv'
IRHandler irh;

byte hold = 0;

int8_t offsetLR = 0;
bool checkGyro = true;
int8_t skipGyro = 2;

#define COUNT_DOWN 60

uint8_t timer = 0;
//#define SKIP 1
#ifdef SKIP
byte updateFrame = 0;
#endif
byte firstMotionJoint;
byte jointIdx = 0;

// bool Xconfig = false;

unsigned long usedTime = 0;
void getFIFO() { // get FIFO only without further processing
  while (fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
}

void getYPR() { // get YPR angles from FIFO data, takes time
  // wait for MPU interrupt or extra packet(s) available
  // while (!mpuInterrupt && fifoCount < packetSize) ;
  if (mpuInterrupt || fifoCount >= packetSize) {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // PTL(fifoCount);
    // check for overflow (this should never happen unless our code is too
    // inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount > OVERFLOW_THRESHOLD) { // 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // otherwise, check for DMP data ready interrupt (this should happen
      // frequently)

      // -- RzLi --
#ifdef FIX_OVERFLOW
#ifdef DEVELOPER
      PTLF("reset FIFO!"); // FIFO overflow! Using last reading!
#endif
      lag = (lag - 1 + HISTORY) % HISTORY;
#endif
      // --
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      getFIFO();

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#ifdef MPU_YAW180
      ypr[2] = -ypr[2];
      ypr[1] = -ypr[1];

#endif
#endif
      for (byte g = 1; g < 3; g++)
        ypr[g] *= degPerRad; // ypr converted to degree

        // overflow is detected after the ypr is read. it's necessary to keep a
        // lag record of previous reading.  -- RzLi --
#ifdef FIX_OVERFLOW
      for (byte g = 1; g < 3; g++) {
        yprLag[lag][g] = ypr[g];
        ypr[g] = yprLag[(lag - 1 + HISTORY) % HISTORY][g];
      }
      lag = (lag + 1) % HISTORY;
#endif

#ifdef DEVELOPER
      PT(ypr[0]);
      PTF("\t");
      PT(ypr[1]);
      PTF("\t");
      PTL(ypr[2]);
#endif
    }
  }
}

void checkBodyMotion() {
  // if (!dmpReady) return;
  getYPR();
  // --
  // deal with accidents
  if (fabs(ypr[1]) > LARGE_PITCH ||
      fabs(ypr[2]) > LARGE_ROLL) { // wait until stable
    if (!hold)
      for (byte w = 0; w < 50; w++) {
        getYPR();
        delay(10);
      }

    // check again
    if (fabs(ypr[1]) > LARGE_PITCH || fabs(ypr[2]) > LARGE_ROLL) {

      if (!hold) {
        irh.token = T_SKILL;
        if (fabs(ypr[2]) > LARGE_ROLL) {
          irh.setNewCmd("rc");
          irh.newCmdIdx = 4;
        } else {
          irh.setNewCmd(ypr[1] < LARGE_PITCH ? "lifted" : "dropped");
          irh.newCmdIdx = 1;
        }
      }
      hold = 10;
    }
  }

  // recover
  else if (hold) {
    if (hold == 1) {
      irh.token = T_SKILL;
      irh.setNewCmd("balance");
      irh.newCmdIdx = 1;
    }
    hold--;
    if (!hold) {
      char temp[CMD_LEN];
      strcpy(temp, irh.newCmd);
      irh.setNewCmd(irh.lastCmd);
      irh.setLastCmd(temp);
      irh.newCmdIdx = 1;
      meow();
    }
  }
  // calculate deviation
  for (byte i = 0; i < 2; i++) {
    RollPitchDeviation[i] =
        ypr[2 - i] - motion.expectedRollPitch[i]; // all in degrees
    // PTL(RollPitchDeviation[i]);
    RollPitchDeviation[i] =
        sign(ypr[2 - i]) * max(fabs(RollPitchDeviation[i]) - levelTolerance[i],
                               0); // filter out small angles
  }
}

void setupSerial() {

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(10);

  // wait for ready
  while (!Serial)
    ;

  // empty buffer
  while (Serial.available() && Serial.read())
    ;

  delay(100);
}

void setupMPU() {
  PTLF("Connect MPU6050");
  mpu.initialize();
  // do
  {
    delay(500);
    // verify connection
    PTLF("Test connection");
    PTL(mpu.testConnection() ? F("MPU successful")
                             : F("MPU failed")); // sometimes it shows "failed"
                                                 // but is ok to bypass.
  } // while (!mpu.testConnection());
}

void setupDMP() {

  do {
    PTLF("Initialize DMP");
    devStatus = mpu.dmpInitialize();
    delay(500);
    // supply your own gyro offsets here, scaled for min sensitivity

    for (byte i = 0; i < 4; i++) {
      PT(EEPROMReadInt(MPUCALIB + 4 + i * 2));
      PT(" ");
    }
    PTL();
    mpu.setZAccelOffset(EEPROMReadInt(MPUCALIB + 4));
    mpu.setXGyroOffset(EEPROMReadInt(MPUCALIB + 6));
    mpu.setYGyroOffset(EEPROMReadInt(MPUCALIB + 8));
    mpu.setZGyroOffset(EEPROMReadInt(MPUCALIB + 10));
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      PTLF("Enable DMP");
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      PTLF("Enable interrupt");
      attachInterrupt(INTERRUPT, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to
      // use it
      PTLF("DMP ready!");
      // dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      PTLF("DMP failed (code ");
      PT(devStatus);
      PTLF(")");
      PTL();
    }
  } while (devStatus);
}

void setupIR() {
  // PTLF("IR Receiver Button Decode");
  irrecv.enableIRIn(); // Start the receiver
}

void setupServos() {

  pwm.begin();

  // Analog servos run at ~60 Hz updates
  pwm.setPWMFreq(60 * PWM_FACTOR);
  delay(200);

  // meow();
  irh.setLastCmd("rest");
  motion.loadBySkillName(irh.lastCmd);
  for (int8_t i = DOF - 1; i >= 0; i--) {
    pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
    servoCalibs[i] = servoCalib(i);
    calibratedDuty0[i] = (SERVOMIN + PWM_RANGE / 2 +
                          float(middleShift(i) + servoCalibs[i]) *
                              pulsePerDegree[i] * rotationDirection(i));

    // PTL(SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) *
    // pulsePerDegree[i] * rotationDirection(i) );
    calibratedPWM(i, motion.dutyAngles[i]);
    delay(20);
  }
  // use the fluctuation of voltage caused by servos as entropy pool
  randomSeed(analogRead(0));
  shutServos();
  irh.token = T_REST;
}

void setup() {
  pinMode(BUZZER, OUTPUT);
#ifdef PIXEL_PIN
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  pixels.setPixelColor(3, pixels.Color(0, 0, LIT_ON));

  pixels.show(); // Send the updated pixel colors to the hardware.
#endif
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  // Wire.setClock(400000);
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  setupSerial();

  PTLF("\n* Start *");
  PTLF("Initialize I2C");

  setupMPU();
  setupDMP();

  // opening music
#if WALKING_DOF == 8
  playMelody(MELODY);
#endif

  setupIR();

  assignSkillAddressToOnboardEeprom();
  PTL();

  setupServos();

  beep(30);

  pinMode(BATT, INPUT);
  pinMode(BUZZER, OUTPUT);
  meow();
}

void normalOperation() {

  irh.resetNewCmd();

  // input block
  // else if (t == 0) {
  if (irrecv.decode(&irh.results)) {
    String IRsig = irParser(irh.translateIR());
    // PTL(IRsig);
    if (IRsig != "") {
      irh.setNewCmd(IRsig.c_str());
      if (strlen(irh.newCmd) == 1)
        irh.token = irh.newCmd[0];
      else
        irh.token = T_SKILL;
      irh.newCmdIdx = 2;
    }
    irrecv.resume(); // receive the next value
  }
  if (Serial.available() > 0) {
    irh.token = Serial.read();
    irh.newCmdIdx = 3;
  }

  // MPU block
  {
#ifdef GYRO // if opt out the gyro, the calculation can be really fast
    if (checkGyro) {
      if (!(timer % skipGyro)) {
        checkBodyMotion();

      } else if (mpuInterrupt || fifoCount >= packetSize) {
        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        getFIFO();
      }
    }
#endif
  }
  // accident block
  //...
  //...
  // for obstacle avoidance and auto recovery
  if (irh.newCmdIdx) {
    PTL(irh.token);
    beep(irh.newCmdIdx * 4);
    // this block handles argumentless tokens
    switch (irh.token) {
    //        case T_HELP: {
    //            PTLF("* info *");// print the help document. not implemented
    //            on NyBoard Vo due to limited space break;
    //          }
    case T_REST: {
      irh.setLastCmd("rest");
      skillByName(irh.lastCmd);
      break;
    }
    case T_GYRO: {
      if (!checkGyro)
        checkBodyMotion();
      //            countDown = COUNT_DOWN;
      checkGyro = !checkGyro;
      irh.token = T_SKILL;
      break;
    }
    case T_PAUSE: {
      tStep = !tStep;
      if (tStep)
        irh.token = T_SKILL;
      else
        shutServos();
      break;
    }
    //        case T_RAMP: { //if the robot is on a ramp, flip the adaption
    //        direction
    //            ramp = (ramp == 1) ? -1.5 : 1;
    //            token = T_SKILL;
    //            break;
    //          }
    case T_SAVE: {
      PTLF("save offset");
      saveCalib(servoCalibs);
      break;
    }
    case T_ABORT: {
      PTLF("aborted");
      for (byte i = 0; i < DOF; i++) {
        servoCalibs[i] = servoCalib(i);
      }
      break;
    }

    // this block handles array like arguments
    case T_INDEXED: // indexed joint motions: joint0, angle0, joint1, angle1,
                    // ...
    case T_LISTED:  // list of all 16 joint: angle0, angle2,... angle15
      // case T_MELODY: //for melody
      {
        String inBuffer = Serial.readStringUntil('~');
        int8_t numArg = inBuffer.length();
        char *list = inBuffer.c_str();
        char *targetFrame = new char[DOF];
        for (int i = 0; i < DOF; i += 1) {
          targetFrame[i] = currentAng[i];
        }
        if (irh.token == T_INDEXED) {
          for (int i = 0; i < numArg; i += 2) {
            targetFrame[list[i]] = list[i + 1];
          }
        } else if (irh.token == T_LISTED) {
          for (int i = 0; i < DOF; i += 1) {
            targetFrame[i] = list[i];
          }
        }
        transform(targetFrame, 1,
                  3); // need to add angleDataRatio if the angles are large
        delete[] targetFrame;

        //            if (token == T_INDEXED) {
        //              for (int i = 0; i < numArg; i += 2) {
        //                calibratedPWM(list[i], list[i + 1]);
        //              }
        //            }
        //            else if (token == T_LISTED) {
        //              allCalibratedPWM(list);
        //            }

        break;
      }
    case T_JOINTS: { // show the list of current joint anles
      printRange(DOF);
      printList(currentAng);
      break;
    }
    case T_CALIBRATE: // calibration
    case T_MOVE:      // move jointIndex to angle
    case T_MEOW:      // meow (repeat, increament)
    case T_BEEP: // beep(tone, duration): tone 0 is pause, duration range is
                 // 0~255
    {
      String inBuffer = Serial.readStringUntil('\n');
      char *temp = new char[30];
      strcpy(temp, inBuffer.c_str());
      char *pch;
      pch = strtok(temp, " ,");
      do { // it supports combining multiple commands at one time
        // for example: "m8 40 m8 -35 m 0 50" can be written as "m8 40 8 -35 0
        // 50" the combined commands should be less than four. string len <=30
        // to be exact.
        int target[2] = {};
        byte inLen = 0;
        for (byte b = 0; b < 2 && pch != NULL; b++) {
          target[b] = atoi(pch);
          pch = strtok(NULL, " ,\t");
          inLen++;
        }
        PT(irh.token);
        printList(target, 2);
        float angleInterval = 0.2;
        int angleStep = 0;
        if (irh.token == T_CALIBRATE) {
          // PTLF("calibrating [ targetIdx, angle ]: ");

          // first time entering the calibration function
          if (irh.isLastCmd("c")) {
            irh.setLastCmd("c");
            motion.loadBySkillName("calib");
            transform(motion.dutyAngles);
            checkGyro = false;
          }
          if (inLen == 2)
            servoCalibs[target[0]] = target[1];
          PTL();
          printRange(DOF);
          printList(servoCalibs);
          // yield();
          int duty = SERVOMIN + PWM_RANGE / 2 +
                     float(middleShift(target[0]) + servoCalibs[target[0]] +
                           motion.dutyAngles[target[0]]) *
                         pulsePerDegree[target[0]] *
                         rotationDirection(target[0]);
          pwm.setPWM(pin(target[0]), 0, duty);
        } else if (irh.token == T_MOVE) {
          // SPF("moving [ targetIdx, angle ]: ");
          angleStep =
              floor((target[1] - currentAng[target[0]]) / angleInterval);
          for (int a = 0; a < abs(angleStep); a++) {
            int duty = SERVOMIN + PWM_RANGE / 2 +
                       float(middleShift(target[0]) + servoCalibs[target[0]] +
                             currentAng[target[0]] +
                             a * angleInterval * angleStep / abs(angleStep)) *
                           pulsePerDegree[target[0]] *
                           rotationDirection(target[0]);
            pwm.setPWM(pin(target[0]), 0, duty);
          }
          currentAng[target[0]] = motion.dutyAngles[target[0]] = target[1];
        } else if (irh.token == T_MEOW) {
          meow(target[0], 0, 50, 200, 1 + target[1]);
        } else if (irh.token == T_BEEP) {
          beep(target[0], (byte)target[1]);
        }

        delay(50);
      } while (pch != NULL);
      delete[] pch;
      delete[] temp;
      break;
    }

      //        case T_XLEG: {
      //            for (byte s = 0; s < 23; s++) {
      //
      //              motion.loadDataFromProgmem(steps[s]);
      //              transform(motion.dutyAngles, 1,2);
      //            }
      //            skillByName("balance");
      //            break;
      //          }

    default:
      if (Serial.available() > 0) {
        String inBuffer = Serial.readStringUntil('\n');
        irh.setNewCmd(inBuffer.c_str());
      }
    }
    while (Serial.available() && Serial.read())
      ; // flush the remaining serial buffer in case the commands are parsed
        // incorrectly
    // check above
    if (irh.isNewCmd("") && irh.isNewCmd(irh.lastCmd)) {
      //      PT("compare lastCmd ");
      //      PT(lastCmd);
      //      PT(" with newCmd ");
      //      PT(token);
      //      PT(newCmd);
      //      PT("\n");
      if (irh.token == T_UNDEFINED) {
      }; // some words for undefined behaviors

      if (irh.token == T_SKILL) { // validating key

        motion.loadBySkillName(irh.newCmd);

        char lr = irh.newCmd[strlen(irh.newCmd) - 1];
        offsetLR = (lr == 'L' ? 15 : (lr == 'R' ? -15 : 0));

        // motion.info();
        timer = 0;
        if (irh.isNewCmd("balance") && irh.isNewCmd("lifted") &&
            irh.isNewCmd("dropped")) {

          irh.setLastCmd(irh.newCmd);
        }
        // Xconfig = strcmp(newCmd, "wkX") ? false : true;

#ifdef POSTURE_WALKING_FACTOR
        postureOrWalkingFactor =
            (motion.period == 1 ? 1 : POSTURE_WALKING_FACTOR);
#endif
        // if posture, start jointIdx from 0
        // if gait, walking DOF = 8, start jointIdx from 8
        //          walking DOF = 12, start jointIdx from 4
        firstMotionJoint = (motion.period <= 1) ? 0 : DOF - WALKING_DOF;
        //          if (Xconfig) { //for smooth transition
        //            int *targetAngle;
        //            targetAngle = new int [WALKING_DOF];
        //            for (byte i = 0; i < WALKING_DOF; i++)
        //              targetAngle[i] = motion.dutyAngles[i];
        //            targetAngle[WALKING_DOF - 6] =
        //            motion.dutyAngles[WALKING_DOF - 2] -5;
        //            targetAngle[WALKING_DOF - 5] =
        //            motion.dutyAngles[WALKING_DOF - 1] -5;
        //            targetAngle[WALKING_DOF - 2] =
        //            motion.dutyAngles[WALKING_DOF - 2] + 180;
        //            targetAngle[WALKING_DOF - 1] =
        //            motion.dutyAngles[WALKING_DOF - 1] + 180; transform(
        //            targetAngle,  1,2, firstMotionJoint); delete []
        //            targetAngle;
        //          }
        //          else

        if (motion.period < 1) {
          int8_t repeat = motion.loopCycle[2] - 1;
          byte frameSize = 20;
          for (byte c = 0; c < abs(motion.period);
               c++) { // the last two in the row are transition speed and
                      // delay
            transform(motion.dutyAngles + c * frameSize, motion.angleDataRatio,
                      motion.dutyAngles[16 + c * frameSize] / 4.0);
#ifdef GYRO // if opt out the gyro, the calculation can be really fast
            if (motion.dutyAngles[18 + c * frameSize]) {
              int triggerAxis = motion.dutyAngles[18 + c * frameSize];
              int triggerAngle = motion.dutyAngles[19 + c * frameSize];

              float currentYpr = ypr[abs(triggerAxis)];
              float previousYpr = currentYpr;
              while (1) {
                getYPR();
                currentYpr = ypr[abs(triggerAxis)];
                PT(currentYpr);
                PTF("\t");
                PTL(triggerAngle);
                if ((180 - fabs(currentYpr) >
                     2) // skip the angle when the reading jumps from 180 to
                        // -180
                    && (triggerAxis * currentYpr < triggerAxis * triggerAngle &&
                        triggerAxis * previousYpr >
                            triggerAxis *
                                triggerAngle)) // the sign of triggerAxis will
                                               // deterine whether the current
                                               // angle should be larger or
                                               // smaller than the trigger angle
                  break;
                previousYpr = currentYpr;
              }
            } else
#endif
            {
              delay(motion.dutyAngles[17 + c * frameSize] * 50);
            }
            if (c == motion.loopCycle[1] && repeat > 0) {
              c = motion.loopCycle[0] - 1;
              repeat--;
            }
          }
          skillByName("balance", 1, 2, false);
          irh.setLastCmd("balance");
          for (byte a = 0; a < DOF; a++)
            currentAdjust[a] = 0;
          hold = 0;
        } else {
          transform(motion.dutyAngles, motion.angleDataRatio, 1,
                    firstMotionJoint);
        }
        jointIdx = 3; // DOF; to skip the large adjustment caused by MPU
                      // overflow. joint 3 is not used.
        if (!irh.isNewCmd("rest")) {
          shutServos();
          irh.token = T_REST;
        }
      } else {
        irh.lastCmd[0] = irh.token;
        memset(irh.lastCmd + 1, '\0', CMD_LEN - 1);
      }
    }
  }

  // motion block
  {
    if (irh.token == T_SKILL) {
      if (jointIdx == DOF) {
#ifdef SKIP
        if (updateFrame++ == SKIP) {
          updateFrame = 0;
#endif
          // timer = (timer + 1) % abs(motion.period);
          timer += tStep;
          if (timer == abs(motion.period)) {
            timer = 0;
            //              if (countDown == 0)
            //                checkGyro = true;
            //              else countDown--;
          } else if (timer == 255)
            timer = abs(motion.period) - 1;
#ifdef SKIP
        }
#endif
        jointIdx =
#ifdef HEAD // start from head
            0;
#else
#ifdef TAIL
            2;
#else
            DOF - WALKING_DOF;
#endif
#endif
      }
#ifndef TAIL
      if (jointIdx == 1)
        jointIdx = DOF - WALKING_DOF;
#endif
      if (jointIdx < firstMotionJoint && abs(motion.period) > 1) {
        calibratedPWM(jointIdx,
                      (jointIdx != 1 ? offsetLR : 0) // look left or right
                          + 10 * sin(timer * (jointIdx + 2) * M_PI /
                                     abs(motion.period)) // look around
#ifdef GYRO
                          + (checkGyro ? adjust(jointIdx) : 0)
#endif
        );
      } else if (jointIdx >= firstMotionJoint) {
        int dutyIdx = timer * WALKING_DOF + jointIdx - firstMotionJoint;
        calibratedPWM(
            jointIdx,
            motion.dutyAngles[dutyIdx] *
                    motion.angleDataRatio //+ ((Xconfig && (jointIdx == 14 ||
                                          // jointIdx == 15)) ? 180 : 0)
#ifdef GYRO
                + (checkGyro ? (!(timer % skipGyro) ? adjust(jointIdx)
                                                    : currentAdjust[jointIdx])
                             : 0)
        //+ (checkGyro ? ((!(timer % skipGyro) && countDown == 0) ?
        // adjust(jointIdx) : currentAdjust[jointIdx]) : 0)
#endif
        );
        //          if (jointIdx == 8) {
        //            PT(currentAdjust[jointIdx]); PT("\t"); PTL(
        //            adjust(jointIdx) );
        //          }
      }
      jointIdx++;
    } else
      timer = 0;
  }
}

#ifdef NyBoard_V0_1
const int LOW_BATTERY_LEVEL = 650;
#else
const int LOW_BATTERY_LEVEL = 300;
#endif

void loop() {
  float voltage = analogRead(BATT);

  if (voltage < LOW_BATTERY_LEVEL) {
    // give the cat a break when voltage drops after sprint
    // adjust the thresholds according to your batteries' voltage
    // if set too high, the robot will keep crying.
    // If too low, Nybble may faint due to temporary voltage drop
    PTL("check battery");
    PTL(voltage); // relative voltage
    meow();

  } else {
    normalOperation();
  }
}
