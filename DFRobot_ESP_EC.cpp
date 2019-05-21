/*
 * file DFRobot_ESP_EC.cpp * @ https://github.com/GreenPonik/DFRobot_ESP_EC_BY_GREENPONIK
 *
 * Arduino library for Gravity: Analog EC Sensor / Meter Kit V2, SKU: DFR0300
 * 
 * Based on the @ https://github.com/DFRobot/DFRobot_EC
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * ##################################################
 * ##################################################
 * ########## Fork on github by GreenPonik ##########
 * ############# ONLY ESP COMPATIBLE ################
 * ##################################################
 * ##################################################
 * 
 * version  V1.0
 * date  2019-05
 */

#include "Arduino.h"
#include "DFRobot_ESP_EC.h"
#include "EEPROM.h"

#define KVALUEADDR 0x0A //the start address of the K value stored in the EEPROM
#define RES2 820.0
#define ECREF 200.0

DFRobot_ESP_EC::DFRobot_ESP_EC()
{
    this->_ecvalue = 0.0;
    this->_kvalue = 1.0;
    this->_kvalueLow = 1.0;
    this->_kvalueHigh = 1.0;
    this->_cmdReceivedBufferIndex = 0;
    this->_voltage = 0.0;
    this->_temperature = 25.0;
}

DFRobot_ESP_EC::~DFRobot_ESP_EC()
{
}

void DFRobot_ESP_EC::begin()
{
    //check if calibration values (kvalueLow and kvalueHigh) are stored in eeprom
    this->_kvalueLow = EEPROM.readFloat(KVALUEADDR); //read the calibrated K value from EEPROM
    if (this->_kvalueLow == float())
    {
        this->_kvalueLow = 1.0; // For new EEPROM, write default value( K = 1.0) to EEPROM
        EEPROM.writeFloat(KVALUEADDR, this->_kvalueLow);
        EEPROM.commit();
    }

    this->_kvalueHigh = EEPROM.readFloat(KVALUEADDR + sizeof(float)); //read the calibrated K value from EEPRM
    if (this->_kvalueHigh == float())
    {
        this->_kvalueHigh = 1.0; // For new EEPROM, write default value( K = 1.0) to EEPROM
        EEPROM.writeFloat(KVALUEADDR + sizeof(float), this->_kvalueHigh);
        EEPROM.commit();
    }
    this->_kvalue = this->_kvalueLow; // set default K value: K = kvalueLow
}

float DFRobot_ESP_EC::readEC(float voltage, float temperature)
{
    float value = 0, valueTemp = 0;
    this->_rawEC = 1000 * voltage / RES2 / ECREF;
    valueTemp = this->_rawEC * this->_kvalue;
    //automatic shift process
    //First Range:(0,2); Second Range:(2,20)
    if (valueTemp > 2.5)
    {
        this->_kvalue = this->_kvalueHigh;
    }
    else if (valueTemp < 2.0)
    {
        this->_kvalue = this->_kvalueLow;
    }

    value = this->_rawEC * this->_kvalue;                  //calculate the EC value after automatic shift
    value = value / (1.0 + 0.0185 * (temperature - 25.0)); //temperature compensation
    this->_ecvalue = value;                                //store the EC value for Serial CMD calibration
    return value;
}

void DFRobot_ESP_EC::calibration(float voltage, float temperature, char *cmd)
{
    this->_voltage = voltage;
    this->_temperature = temperature;
    strupr(cmd);
    ecCalibration(cmdParse(cmd)); // if received Serial CMD from the serial monitor, enter into the calibration mode
}

void DFRobot_ESP_EC::calibration(float voltage, float temperature)
{
    this->_voltage = voltage;
    this->_temperature = temperature;
    if (cmdSerialDataAvailable() > 0)
    {
        ecCalibration(cmdParse()); // if received Serial CMD from the serial monitor, enter into the calibration mode
    }
}

boolean DFRobot_ESP_EC::cmdSerialDataAvailable()
{
    char cmdReceivedChar;
    static unsigned long cmdReceivedTimeOut = millis();
    while (Serial.available() > 0)
    {
        if (millis() - cmdReceivedTimeOut > 500U)
        {
            this->_cmdReceivedBufferIndex = 0;
            memset(this->_cmdReceivedBuffer, 0, (ReceivedBufferLength));
        }
        cmdReceivedTimeOut = millis();
        cmdReceivedChar = Serial.read();
        if (cmdReceivedChar == '\n' || this->_cmdReceivedBufferIndex == ReceivedBufferLength - 1)
        {
            this->_cmdReceivedBufferIndex = 0;
            strupr(this->_cmdReceivedBuffer);
            return true;
        }
        else
        {
            this->_cmdReceivedBuffer[this->_cmdReceivedBufferIndex] = cmdReceivedChar;
            this->_cmdReceivedBufferIndex++;
        }
    }
    return false;
}

byte DFRobot_ESP_EC::cmdParse(const char *cmd)
{
    byte modeIndex = 0;
    if (strstr(cmd, "ENTEREC") != NULL)
        modeIndex = 1;
    else if (strstr(cmd, "EXITEC") != NULL)
        modeIndex = 3;
    else if (strstr(cmd, "CALEC") != NULL)
        modeIndex = 2;
    return modeIndex;
}

byte DFRobot_ESP_EC::cmdParse()
{
    byte modeIndex = 0;
    if (strstr(this->_cmdReceivedBuffer, "ENTEREC") != NULL)
        modeIndex = 1;
    else if (strstr(this->_cmdReceivedBuffer, "EXITEC") != NULL)
        modeIndex = 3;
    else if (strstr(this->_cmdReceivedBuffer, "CALEC") != NULL)
        modeIndex = 2;
    return modeIndex;
}

void DFRobot_ESP_EC::ecCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean ecCalibrationFinish = 0;
    static boolean enterCalibrationFlag = 0;
    static float compECsolution;
    float KValueTemp;
    switch (mode)
    {
    case 0:
        if (enterCalibrationFlag)
        {
            Serial.println(F(">>>Command Error<<<"));
        }
        break;

    case 1:
        enterCalibrationFlag = 1;
        ecCalibrationFinish = 0;
        Serial.println();
        Serial.println(F(">>>Enter EC Calibration Mode<<<"));
        Serial.println(F(">>>Please put the probe into the 1413us/cm or 12.88ms/cm buffer solution<<<"));
        Serial.println();
        break;

    case 2:
        if (enterCalibrationFlag)
        {
            if ((this->_rawEC > 0.9) && (this->_rawEC < 1.9))
            {                                                                          //recognize 1.413us/cm buffer solution
                compECsolution = 1.413 * (1.0 + 0.0185 * (this->_temperature - 25.0)); //temperature compensation
            }
            else if ((this->_rawEC > 9) && (this->_rawEC < 16.8))
            {                                                                          //recognize 12.88ms/cm buffer solution
                compECsolution = 12.88 * (1.0 + 0.0185 * (this->_temperature - 25.0)); //temperature compensation
            }
            else
            {
                Serial.print(F(">>>Buffer Solution Error Try Again<<<   "));
                ecCalibrationFinish = 0;
            }
            KValueTemp = RES2 * ECREF * compECsolution / 1000.0 / this->_voltage; //calibrate the k value
            if ((KValueTemp > 0.5) && (KValueTemp < 1.5))
            {
                Serial.println();
                Serial.print(F(">>>Successful,K:"));
                Serial.print(KValueTemp);
                Serial.println(F(", Send EXITEC to Save and Exit<<<"));
                if ((this->_rawEC > 0.9) && (this->_rawEC < 1.9))
                {
                    this->_kvalueLow = KValueTemp;
                }
                else if ((this->_rawEC > 9) && (this->_rawEC < 16.8))
                {
                    this->_kvalueHigh = KValueTemp;
                }
                ecCalibrationFinish = 1;
            }
            else
            {
                Serial.println();
                Serial.println(F(">>>Failed,Try Again<<<"));
                Serial.println();
                ecCalibrationFinish = 0;
            }
        }
        break;
    case 3:
        if (enterCalibrationFlag)
        {
            Serial.println();
            if (ecCalibrationFinish)
            {
                if ((this->_rawEC > 0.9) && (this->_rawEC < 1.9))
                {
                    EEPROM.writeFloat(KVALUEADDR, this->_kvalueLow);
                }
                else if ((this->_rawEC > 9) && (this->_rawEC < 16.8))
                {
                    EEPROM.writeFloat(KVALUEADDR + sizeof(float), this->_kvalueHigh);
                }
                Serial.print(F(">>>Calibration Successful"));
            }
            else
            {
                Serial.print(F(">>>Calibration Failed"));
            }
            Serial.println(F(",Exit EC Calibration Mode<<<"));
            Serial.println();
            ecCalibrationFinish = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}
