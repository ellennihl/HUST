//gör konverterung till char array                                                           DONE
//koppla bromsgivare. Om hög, sendmsg att current = 0. asså accenable==false och current = 0
//koppla upp till bms och ta emot värdena. uppdattera idna här
//koppla upp till mc och ta emot värdena. uppdatera id
//kolla så att koden funkar
//koppla upp till rpi och synca
//gör min unit test

/**
 * andra ex, ex1 + otestad CAN kod.
 */
//CAN----------------------------------------------------------------------
#include <SPI.h>
#include "mcp2515_can.h"
#include <TimeInterrupt.h>        //For interrupt RTC handler

const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;

mcp2515_can CAN(SPI_CS_PIN);      //Set CS pin

unsigned char len = 0;
unsigned char buf[8];             //Stores received CAN data
unsigned char flagRecv = 0;
unsigned char flagSend = 0;
unsigned long canId = 0;

//BMS
float packCurrent = 0;
float packVoltage = 0;
float highTempBMS = 0;
float avgTempBMS = 0;
float packSOH = 0;
float avgCurrent = 0;
float majorErrorBMS = 0;
float highID = 0;
float pack_kW = 0;
float lowCellVoltage = 0;
float highCellVoltage = 0;
float avgCellVoltage = 0;
float patch = 0;
float minorErrorBMS = 0;
float packSOC = 0;

//MC
float receiveErrorCountMC = 0;
float transmitErrorCountMC = 0;
float velocity_ms = 0;
float velocity_rpm = 0;
float heatsinkTemp = 0;
float motorTemp = 0;
float dspBoardTemp = 0;

//ALL TYPES OF DATA TO SEND------------------------------------------------
//BMS
unsigned char toBMS[8] = {1, 0, 0, 0, 0, 0, 0, 0};      //bms byte 0 först (index 0). Denna ser till att kontaktorn håller sig igång
//MC
unsigned char mtrDrvCmd[8] = {0x00, 0x40, 0x9c, 0x46, 0x00, 0x00, 0x00, 0x00}; //tourqe mode(20000) and current = 0 %
unsigned char mtrDrvCmdNoAcc[8] = {0x00, 0x40, 0x9c, 0x46, 0x00, 0x00, 0x00, 0x00};
//{0x00, 0x40, 0x9c, 0x46, 0x0a, 0xd7, 0xa3, 0x3c}; //denna skickar 20000 rpm och 0.02 current
float stmpFloat[2] = {0,0};
//FYLL I HÄR MED RIKTIGA VÄRDEN O NAMN SEN. TEX MTRCMD
unsigned char stmpChar[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char stmpChar1[8] = {0x00, 0x00, 0x00, 0x00, 0x46, 0x9c, 0x40, 0x00};

//Buttons------------------------------------------------------------------
int neutralPin = 22;
int racePin = 23;
int reversePin = 24;
int motorTogglePin = 25;
int accPin = A15;

int breakPin = 30;

//LEDs---------------------------------------------------------------------
int neutralLED = 26;
int raceLED = 27;
int reverseLED = 28;
int motorToggleLED = 29;
int breakLightsPin = 31;

//Other--------------------------------------------------------------------
int buzzerPin = 3;
bool accEnable = false;
bool calibrated = false;
bool motorOK = false;       //motor button on or off, true means we are allowed to send data to mc
bool breaking = false;
int accMin = 279;
int accMax = 555;
float acc = 0;
int mode = 0;               //neutral = 0, race = 1, reverse = 2
int fromPi = 0; 

void setup() {//-----------------------------------------------------------
  Serial.begin(115200);

  if (CAN.begin(CAN_500KBPS) != 0) {
    //Serial.println("CANiniterror");
    while(1);
  }
  //Serial.println("CANinitOK");

  //BMS STARTUP------------------------------------------  
  //delay(5000);//ladda kondensatorerna
  CAN.sendMsgBuf(0x6B1, 0, 8, toBMS);                 //send stmp==1,0,0,0,0,0,0,0 to id=0x6B1 for enable of contactor
  //-----------------------------------------------------
  
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING); //start interrupt

  //Initialize Timer interrupt every 200 ms for CAN send msg to bms and mc
  TimeInterrupt.begin(PRECISION);                     //Initialize the interrupt with high precision timing
  TimeInterrupt.addInterrupt(timing_Handler, 200);    //Call 'method_name' every (milliseconds)

  initPins();

  if(digitalRead(motorTogglePin == LOW)){            //If motor button on, it is OK to send data to mc
    motorOK = true;
    }
  if(digitalRead(breakPin) == HIGH){
    breaking = true;
    }
    
  changeModeLEDs();
  //calibrated = true;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1ändra
  //kalibrera accpin NEJ det ska pi:n fråga om
  //sätt på motorn NEJ det ska loopens knappläsning ha koll på
  //calibrated = true;
}

void loop() {
//--------------------------READ RECEIVE--------------------------
  if(flagRecv){
    flagRecv = 0;
    readReceive();
    }

//------------------------READ BROMSGIVARE------------------------
  if(digitalRead(breakPin) == HIGH && !breaking){
    CAN.sendMsgBuf(0x700, 0, 8, mtrDrvCmdNoAcc);  //stop accelerating
    digitalWrite(breakLightsPin, HIGH);
    breaking = true;
    accEnable = false;
    }
  else if(digitalRead(breakPin) == LOW && breaking){
    digitalWrite(breakLightsPin, LOW);
    breaking = false;
    accEnable = true;
    }

//------------------------READ MOTOR BTN--------------------------
  readMotorBtn();
  
//-------------------SEND MOTOR DRIVE COMMAND---------------------
  updateAcc();                                //Update value of acc
  if(flagSend){
    flagSend = 0;
    CAN.sendMsgBuf(0x6B1, 0, 8, toBMS);       //Tell BMS to keep the contactor on
    if(motorOK){                              //skicka sendmsg med de uppdaterade värdena, om !motorok, skicka först current=0 sen sluta skicka
      updateMotorDriveCommand();              //updatemotor updates the mtrDrvCmd array
      CAN.sendMsgBuf(0x700, 0, 8, mtrDrvCmd); //sen så skickar den den uppdaterade arrayen.byt id!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      
    /*Serial.println("broms="+String(breaking)+"\nacc="+String(acc)+"\nmode="+String(mode));
    for(int i=0; i<8; i++){
      Serial.print(mtrDrvCmd[i],HEX);
      Serial.print(" ");*/
        //}
      }
    }
  //sendMotorDriveCommand();//Update value to send to mc//detta gör ju interrupten dock. behöver inte uppdatera varje varv, eftersom den redan gör d var 200 msNEJ

//--------------------COMMUNICATION WITH PI-----------------------  
  if(Serial.available() > 0){ //Serial.readStringUntil('\n'); om jag vill läsa string
    fromPi = Serial.read(); //C = calibrate, P = pin OK, D = send DATA, R = restart hela systemet, F = fast data, S = slow data
    switch(fromPi){
    case 'D': //Denna ska inte användas sen
      Serial.println(String(acc));
      break;
    case 'C':               //Calibrate
      //cal
      if(mode == 0){//vi måste va i låg hastighet också!!!
        calibrated = calibrateAcc();
      }
      else{
        Serial.println("NO");//raspberryn väntar ju på OK, så den avbryter om det kommer något annat
      }
      break;
    case 'R'://Restart
      //nollställ variabler o kör om allting
      break;
    case 'F':               //Fast data
      Serial.println("V" + String(random(0, 120)));
      Serial.println("A" + String(acc));
      break;
    case 'S':               //Slow data
      Serial.println("BC" + String(random(0, 100)));
      break;
    default:
      break;
    }
  }
//------------------------------------------------------------------------------

  //Serial.println("accPin = " + String(updateAcc())); //skicka acc rätt ofta men kanse inte varje varv KANSKE IF FLAGSEND???!!!!!!!!!!!!!!!!!!!!!!!!!!
//--------------------------CHANGE MODE--------------------------
  if(calibrated){
    if(digitalRead(neutralPin) == HIGH && mode!=0){// && hastighet <= 0 km/h
      accEnable = false;
      mode = 0;
      changeModeLEDs();
      //Serial.println("neutral");
      //skicka till skölden konstant 0 till acc
      }
    else if(digitalRead(racePin) == HIGH && mode!=1){
      accEnable = true;
      mode = 1;
      changeModeLEDs();
      //Serial.println("race");
      //skicka analogread(acc) till sköld
      }
    else if(digitalRead(reversePin) == HIGH && mode!=2){//&& hastighet <= 0 && acc <= 0
      accEnable = true;
      mode = 2;
      changeModeLEDs();
      //Serial.println("reverse");
      //skicka till mc -20000rpm så det blir reverse
      }
  }
  //Serial.print("looptime: ");
  //Serial.println(millis() - looptime);
}

/**
 *  If we turn off motor btn, stop sending data to mc so that the motor turns off.
 *  
 */
void readMotorBtn(){
    if(digitalRead(motorTogglePin) == HIGH/* && motorOK*/){//From on to off
      //first send speed = 0 to mc, then nothing.
      //unsigned char stmpChar[8] = {0x3c, 0xa3, 0xd7, 0x0a, 0x46, 0x9c, 0x40, 0x00};//0.2 och 20 000
      CAN.sendMsgBuf(0x700, 0, 8, mtrDrvCmdNoAcc);  //stop the motor from moving
      motorOK = false;                              //then this will make the interrupt handler not send more msg to motor
      accEnable = false;
      mode = 0;                                     //TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
      changeModeLEDs();
      }
      
  else if(digitalRead(motorTogglePin) == LOW /*&& !motorOK*/){//From off to on
    motorOK = true;
    accEnable = true;
    changeModeLEDs();
    }
    
  }

/**
 * Calibrate the accelerator pedals together with the Rapsberry Pi
 * through Serial communication.
 */
bool calibrateAcc(){
  Serial.println("OK");//raspberry har skickat 'C' o väntar på svar från Arduino
  while(Serial.available() <= 0);  //vänta på rpi ger oss instruktioner
  if(Serial.read() == '1'){//MIN
    accMin = analogRead(accPin);
    Serial.println(String(accMin));
    while(Serial.available() <= 0);
    if(Serial.read() == '2'){//MAX
      accMax = analogRead(accPin);
      Serial.println(String(accMax));
      while(Serial.available() <= 0);
      if(Serial.read() == '3'){//DONE
        return true;
        }
        else{
          return false;
        }
      }
      else{
        return false;
      }
    }
      else{
        return false;
      }
    }

/**
 * Updates the value of acc, which is the value of the accelerator pedals
 * in percent. Makes sure the values returned are always 0<=acc<=0.99
 */
void updateAcc(){
  if((accMax > accMin) && calibrated){//gränserna för annars blir det NaN
    acc = (float)((float)(analogRead(accPin) - (float)accMin) / ((float)accMax-(float)accMin)); //till procent
    if(acc > 0.99){//om det blev nåt fel med kalibreringen och det råkar bli över värdet SÄTT KANSKE ATT VI INTE FÅR HA 100 % UTAN MAX 90 %
        acc = 0.99;
        //skicka till pi att vi bör kalibrera
      }
    if(acc < 0.02){
        acc = 0;
      }
    }
  else{
    acc = 0;
    }
  }

/**
 * Initialization of pins
 */
void initPins(){
  
  pinMode(neutralPin, INPUT);
  pinMode(racePin, INPUT);
  pinMode(reversePin, INPUT);
  pinMode(motorTogglePin, INPUT);
  pinMode(accPin, INPUT);
  pinMode(breakPin, INPUT);

  pinMode(neutralLED, OUTPUT);
  pinMode(raceLED, OUTPUT);
  pinMode(reverseLED, OUTPUT);
  pinMode(motorToggleLED, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(breakLightsPin, OUTPUT);
  
  }

/**
 * Changes the corresponding LED to the current mode.
 */
void changeModeLEDs(){
  digitalWrite(neutralLED, LOW);
  digitalWrite(raceLED, LOW);
  digitalWrite(reverseLED, LOW);
  if(mode == 0){
    digitalWrite(neutralLED, HIGH);
    }
  else if(mode == 1 && motorOK){
    digitalWrite(raceLED, HIGH);
    }
  else if(mode == 2 && motorOK){
    digitalWrite(reverseLED, HIGH);
    }
  if(digitalRead(motorTogglePin) == LOW){
    digitalWrite(motorToggleLED, HIGH);
    }
  else if(digitalRead(motorTogglePin) == HIGH){
    digitalWrite(motorToggleLED, LOW);
    }
  }

/**
 * Handles CAN Shield interrupt if there is data to receive.
 */
void MCP2515_ISR() {
  noInterrupts();
  flagRecv = 1;
  interrupts();
}
/**
 * Handles the timing interrupt. Sends to BMS to keep the contactor on, and Driver Motor Commands to MC, every 200 ms.
 */
void timing_Handler(){//JAG KAN INTE SKICKA CAN.SENDMSG I INTERRUPTEN FATTAR JAG VÄL. DENNA SKA BARA SÄTTA FLAGGA KANSKEXXX
  noInterrupts();
  flagSend = 1;
  interrupts();
  }

/**
 * Send motor drive command to mc. 
 * -20 000<=rpm<=20 000
 * 0<=current<=1
 * -20 000 rpm enters tourqe mode and drives forward
 * +20 000 rpm enters tourqe mode and drives backwards
 * Current is proportional to acc. Send acc.
 * Motor Drve command ID = Driver Control Base Address + 0x01
 * Bit 63-32: current
 * Bit 31-0: rpm
 */
void updateMotorDriveCommand(){//skickar datan från paddeln till mc!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!fixa meddelanden
  float accToSend = acc;
  float rpmToSend = 20000;
  //if massa grejer, kolla mode, hastighet, osv
      if(calibrated){
      switch(mode){         //neutral = 0, race = 1, reverse = 2
        case(0):            //neutral. Acc is not sent. Stand still.
          accToSend = 0;
          rpmToSend = 20000;
            //sendMotorDriveCommand(0, -20000);
          break;
        case(1):            //Race. Tourqe mode. +20 000 rpm forward. Send updated acc.
          accToSend = acc;
          rpmToSend = 20000;
            //sendMotorDriveCommand(acc, -20000);
          break;
        case(2):            //Reverse. -20 000 rpm backwards.
          accToSend = acc;
          rpmToSend = -20000;
          //sendMotorDriveCommand(acc, 20000);
          break;
        default:
          break;
        }
    }
    else{                   //If pedals are not calibrated, send constant 0's.
      accToSend = 0;
      rpmToSend = 20000;
      }
      
      unsigned char accHEX[4];           //create char array to store acc value with IEEE 754 standard, same with rpm
      unsigned char rpmHEX[4];       
      memcpy(accHEX, &accToSend, 4);     //store the value in this array
      memcpy(rpmHEX, &accToSend, 4);
      memcpy(mtrDrvCmd, &rpmHEX, 4);     //in motordrivecommand, the first 4 bytes is rpm
      memcpy(&mtrDrvCmd[4], &accHEX, 4); //then, the last 4 bytes are filled with acc
    //med dessa uppdaterade värdena ska jag nu fylla mtrDrvCmd!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!gör en float to chararray
    
    
    //update mtrdrvcmd according to acc och mode!!!
    //if(mode == 0)...
    //mtrDrvCmd[0] = accToSend;//current från paddel
    //mtrDrvCmd[1] = rpmToSend;//rpm. 20 000 om reverse mode är igång
    //CAN.sendMsgBuf(0x00, 0, 8, (byte*)stmpFloat);//kolla IDt, den är driver control base address
    //SERIAL_PORT_MONITOR.println("CAN BUS sendMsgBuf ok!");
  }

/**
 * parameters:
//BMS
float packCurrent = 0;
float packVoltage = 0;
float highTempBMS = 0;
float avgTempBMS = 0;
float packSOH = 0;
float avgCurrent = 0;
float majorErrorBMS = 0;
float highID = 0;
float pack_kW = 0;
float lowCellVoltage = 0;
float highCellVoltage = 0;
float avgCellVoltage = 0;
float patch = 0;
float minorErrorBMS = 0;
float packSOC = 0;

//MC
float receiveErrorCountMC = 0;
float transmitErrorCountMC = 0;
float velocity_ms = 0;
float velocity_rpm = 0;
float heatsinkTemp = 0;
float motorTemp = 0;
float dspBoardTemp = 0;
 */
void readReceive(){//sätt på buzzer om vi får ett error eller motortemp e för hög eller så!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!fixa vilka värden vi vill spara
  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
    if(canId == 0x6B0){            //DATA FROM BMS
      /*packCurrentBMS = (float)((buf[1]<<8) + buf[0]);
      packSOCBMS = (float)((buf[3]<<8) + buf[2]);
      highTempBMSBMS = (float)buf[4];
      avgTempBMS = (float)((buf[6]<<8) + buf[5]);
      majorErrorBMS = (float)buf[7];*/
      packCurrent = (buf[1]<<8) + buf[0];
      packVoltage = (buf[3]<<8) + buf[2];
      highTempBMS = buf[4];
      avgTempBMS = buf[5];
      highID = buf[6];
      pack_kW = buf[7];
      //Serial.println("packcurrent: " + String(packCurrent) + "packSOC: " + String(packSOC) +"highTempBMS: " + String(highTempBMS) + "avgTempBMS: " + String(avgTempBMS) + "Major error: " + String(majorErrorBMS));
      }
    else if(canId == 0x6B1){      //DATA2 FROM BMS
        //Serial.println("här kmr annan bms data");
      }
    else if(canId = 0x6B2){
      lowCellVoltage = (buf[1]<<8) + buf[0];
      highCellVoltage = (buf[3]<<8) + buf[2];
      avgCellVoltage = (buf[5]<<8) + buf[4];
      avgCurrent = (buf[7]<<8) + buf[6];
      }
    else if(canId = 0x6B4){
      patch = (buf[1]<<8) + buf[0];
      minorErrorBMS = (buf[3]<<8) + buf[2];
      majorErrorBMS = (buf[5]<<8) + buf[4];
      packSOH = buf[6];
      packSOC = buf[7];
      }
      //MC 400 BROADCAST BASE ADDRESS AND MC 500 DRIVE COMMAND BASE ADDRESS
    else if(canId == 0x401){      //MC STATUS INFORMATION
      receiveErrorCountMC = (float)buf[8];//bit 63->8st error count
      transmitErrorCountMC = (float)buf[7];
      // Serial.println("receive error count= " + String(receiveErrorCountMC) + "transmit error count= " + String(transmitErrorCountMC));
      }
    else if(canId == 0x403){      //MC VELOCITY MEASUREMENT
      unsigned char tempFloat0_31[4] = {buf[0], buf[1], buf[2], buf[3]};
      unsigned char tempFloat63_32[4] = {buf[4], buf[5], buf[6], buf[7]};
      velocity_ms = *(float*)&tempFloat63_32;
      velocity_rpm = *(float*)&tempFloat0_31;
      }
    else if(canId == 0x40b){      //MC HEAT SINK AND MOTOR TEMPERATURE MEASUREMENT
      unsigned char tempFloat0_31[4] = {buf[0], buf[1], buf[2], buf[3]};
      unsigned char tempFloat63_32[4] = {buf[4], buf[5], buf[6], buf[7]};
      heatsinkTemp = *(float*)&tempFloat63_32;
      motorTemp = *(float*)&tempFloat0_31;
      }
    else if(canId == 0x40C){      //MC DSP BOARD TEMPERATURE MEASUREMENT
      unsigned char tempFloat0_31[4] = {buf[0], buf[1], buf[2], buf[3]};
      velocity_rpm = *(float*)&tempFloat0_31;
      }
      

    //if id==0x6B0 är det bms och ska ta emot byte0,2,4,5
    //if motortemp > 90 grader celcius: SLUTA SKICKA TILL MOTORN och börja buzza!!    
        //if canid == 0x0+0xxx för alla olika fall, sen skicka. Switch case?
        //ta buf, decode så vi förstår vad det är. skicka den till rpi. gör kanske om till uint32_t? för o läsa
  }
}
