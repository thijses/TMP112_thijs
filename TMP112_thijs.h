
// https://www.ti.com/lit/ds/symlink/tmp112.pdf?ts=1666697025800
/*
a little library for interfacing with the Texas Instruments TMP112 temperature sensor
only I2C is implemented, SMBus support (which would mostly be use of the Alert-pin-general-call-thingy and very low freq use)
I did not bother to implement the High-Speed Mode I2C functionality (1kHz~2.85MHz instead of 1kHz~400kHz fast-mode)

this is a library for interfacing with the temperature sensor: TMP112 (specifically the TMP112NAIDRLR in my case)
it supports I2C (/SMBus) interfacing from 1kHz to 2.83MHz (note: special instruction required for >400kHz operation)
note: I2C is not the most efficient protocol, so at 1kHz, retrieving 1 measurement may well take 25 milliseconds
it only has 4 registers, the latter half of which are only relevant if you want the ALERT pin to output like a binary thermostat.
the 'pointer byte' (indicating which register to target), can only vary the 2 LSBits at the end. At boot/reset, the pointer value is 00
register 00 is the temperature output register (Read-only): it's a straightforward 12bit temperature measurement (val*0.0625 to get deg C)
register 01 is the configuration register (Read/Write): 
the format is always MSByte first, LSByte second.
in slave-receiver mode (R/W bit LOW), the first byte (after the address) sets the 'pointer', the (optional) next 2 bytes write to that register
in slave-transmitter mode (R/W bit HIGH), it reads from the register the pointer was last set to
By defualt, the device is in continuous conversion mode, which just delays measurements based on the CR bits in the CONF register
The maximum conversion rate in this mode is 8 (see datasheet for CR table)
However, in ShutDown mode, the One-Shot bit can be used to manually trigger a conversion (which takes about 26ms)
The maximum conversion rate in that mode is "30 or more" (according to the datasheet).
Most of the time, the device operates in I2C 'fast-mode', which is 1kHz ~ 400kHz.
To activate High-Speed mode, "the master device must issue an Hs-mode master code (0000 1xxx) as the first byte after a START condition"
After a STOP is read however, the device reverts back to fast-mode. This library does NOT implement High-Speed mode.

here is one example of some very basic arduino code someone else wrote: https://github.com/DcubeTechVentures/TMP112



TODO:
- STM32 alternate pins testing
- add example sketch filenames to library.json
- test ESP32 with useWireLib
- test 328p (both optimized and wireLib)
- check Wire.h function return values for I2C errors
- test if 'static' vars in the ESP32 functions actually are static (connect 2 sensors?)

*/

#ifndef TMP112_thijs_h
#define TMP112_thijs_h

#include "Arduino.h" // always import Arduino.h

//#define TMP112debugPrint(x)  Serial.println(x)
//#define TMP112debugPrint(x)  log_d(x)   //using the ESP32 debug printing

#ifndef TMP112debugPrint
  #define TMP112debugPrint(x)  ;
#endif


// TMP112 address lookup table:
// #define TMP112_ADDR_A0_to_GND  0x48
// #define TMP112_ADDR_A0_to_VCC  0x49
// #define TMP112_ADDR_A0_to_SDA  0x4A
// #define TMP112_ADDR_A0_to_SCL  0x4B
enum TMP112_ADDR_ENUM : uint8_t {
  TMP112_ADDR_A0_to_GND = 0x48,
  TMP112_ADDR_A0_to_VCC = 0x49,
  TMP112_ADDR_A0_to_SDA = 0x4A,
  TMP112_ADDR_A0_to_SCL = 0x4B
};

//// TMP112 constants:
// output registers:
#define TMP112_TEMP 0x00    // (R) temperature register
// configuration registers:
#define TMP112_CONF 0x01    // (R/W) configuration register     (0x60A0 on reset)
#define TMP112_TLOW 0x02    // (R/W) low temperature threshold register    (+75deg on reset)
#define TMP112_THIGH 0x03   // (R/W) high temperature threshold register   (+80deg on reset)
// notes: registers hold 16 bits. The first byte transmitter in a slave-receiver interaction is the register pointer byte.

// parts of the CONF register:
// in MSByte
#define TMP112_SD_bits  0b00000001 // (in MSByte) Shutdown mode
#define TMP112_TM_bits  0b00000010 // (in MSByte) Thermostat mode
#define TMP112_POL_bits 0b00000100 // (in MSByte) Polarity
#define TMP112_FQ_bits  0b00011000 // (in MSByte) Fault Queue (alert threshold)
#define TMP112_RES_bits 0b01100000 // (in MSByte) Resolution bits (READ ONLY)
#define TMP112_OS_bits  0b10000000 // (in MSByte) One-Shot
// in LSByte
#define TMP112_EM_bits  0b00010000 // (in LSByte) Extended mode (13bit mode)
#define TMP112_AL_bits  0b00100000 // (in LSByte) Alert (READ ONLY)
#define TMP112_CR_bits  0b11000000 // (in LSByte) Conversion Rate
// default values (according to the datasheet) 


#include "_TMP112_thijs_base.h" // this file holds all the nitty-gritty low-level stuff (I2C implementations (platform optimizations))
/**
 * An I2C interfacing library for the TMP112 temperature sensor from Texas Instruments
 * 
 * features the option to use the Wire.h library, or optimized code for the following platforms:
 * atmega328p (direct register manipulation)
 * ESP32 (below HAL layer, but not lowest level)
 * MSP430 (through Energia(?) middle layer)
 * STM32 (through twi->HAL layers)
 */
class TMP112_thijs : public _TMP112_thijs_base
{
  public:
  using _TMP112_thijs_base::_TMP112_thijs_base;
  /*
  This class only contains the higher level functions.
   for the base functions, please refer to _TMP112_thijs_base.h
  here is a brief list of all the lower-level functions:
  - init()
  - requestReadBytes()
  - onlyReadBytes()
  - writeBytes()
  */
  //// the following functions are abstract enough that they'll work for either architecture

  private:
  inline static int16_t _rawToInt(uint8_t readBuff[], bool is13bit=false) { // used to convert the 2-byte registers (not CONF) to usable signed integers
    int16_t returnData = readBuff[0] << 8; // shift MSByte all the way to the left (to get sign information)
    returnData |= readBuff[1]; // add LSByte
    returnData = returnData >> (is13bit ? 3 : 4); // shift to the right (while retaining sign bit)
    return(returnData); //(i know you could turn some of the lines of code into 1 big line, but that will not make the program faster
  }

  public:

  /**
   * (just a macro) check whether an TMP112_ERR_RETURN_TYPE (which may be one of several different types) is fine or not 
   * @param err (bool or esp_err_t or i2c_status_e, see on defines at top)
   * @return whether the error is fine
   */
  bool _errGood(TMP112_ERR_RETURN_TYPE err) {
    #if defined(TMP112_return_esp_err_t)
      return(err == ESP_OK);
    #elif defined(TMP112_return_i2c_status_e)
      return(err == I2C_OK);
    #else
      return(err);
    #endif
  }

  /**
   * request a 2byte value from a register and assemble it into a regular signed interger
   * @param registerToRead register byte (see list of defines at top)
   * @param is13bit whether the sensor is in Extended Mode (13bit mode). Only measurement data has an identifier bit
   * @return post-conversion 12/13bit integer
   */
  int16_t requestReadInt(uint8_t registerToRead, bool is13bit=false) { //note: by int i mean 12bit/13bit signed value  (NOT to be used on CONF register!)
    uint8_t readBuff[2];
    requestReadBytes(registerToRead, readBuff, 2);
    if((readBuff[1] & 0x01) && !is13bit) { TMP112debugPrint("requestReadInt() is13bit should be true!"); is13bit = (readBuff[1] & 0x01); } // recognize a 13bit temperature measurement (Thigh/low does not have this luxury)
    return(_rawToInt(readBuff, is13bit));
  }

  /**
   * read a 2byte value (from the last-accessed register) and assemble it into a regular signed interger
   * @param is13bit whether the sensor is in Extended Mode (13bit mode). Only measurement data has an identifier bit
   * @return post-conversion 12/13bit integer
   */
  int16_t onlyReadInt(bool is13bit=false) { //note: by int i mean 12bit/13bit signed value  (NOT to be used on CONF register!)
    uint8_t readBuff[2];
    onlyReadBytes(readBuff, 2);
    if((readBuff[1] & 0x01) && !is13bit) { TMP112debugPrint("onlyReadInt() is13bit should be true!"); is13bit = (readBuff[1] & 0x01); } // recognize a 13bit temperature measurement (Thigh/low does not have this luxury)
    return(_rawToInt(readBuff, is13bit));
  }

  /**
   * convert a regular signed integer to the TMP112's internal format and write to a register
   * @param registerToWrite register byte (see list of defines at top)
   * @param intToWrite regular signed integer
   * @param is13bit whether the sensor is in Extended Mode (13bit mode). Only measurement data has an identifier bit
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
   */
  TMP112_ERR_RETURN_TYPE writeInt(uint8_t registerToWrite, int16_t intToWrite, bool is13bit=false) { //note: by int i mean 12bit/13bit signed value  (NOT to be used on CONF register!)
    uint8_t writeBuff[sizeof(intToWrite)];
    intToWrite = intToWrite << (is13bit ? 3 : 4); // one shift to resolve the 12/13bit dilemma
    writeBuff[0] = intToWrite >> 8;
    writeBuff[1] = intToWrite & 0xFF;
    return(writeBytes(registerToWrite, writeBuff, sizeof(intToWrite)));
  }

  // measurement retrieval (the main function of the library) boils down to just 2 macros:
  /**
   * retrieve a temperature measurement (but don't convert to a float)
   * @param is13bit (optional) whether the sensor is in Extended Mode (13bit mode). Measurement data has an identifier bit, so no strict need to specify
   * @return temperature measurement (as 12/13bit integer)
   */
  int16_t getTempInt(bool is13bit=false) { return(requestReadInt(TMP112_TEMP, is13bit)); } // just a macro
  /**
   * retrieve a temperature measurement
   * @param is13bit (optional) whether the sensor is in Extended Mode (13bit mode). Measurement data has an identifier bit, so no strict need to specify
   * @return temperature measurement in degrees Celsius
   */
  float getTemp(bool is13bit=false) { return((float)getTempInt(is13bit) * 0.0625); }

  // thresholds (for Alert (and thermostat) purposes):
  TMP112_ERR_RETURN_TYPE setThighInt(int16_t newValue, bool is13bit=false) { return(writeInt(TMP112_THIGH, newValue, is13bit)); }
  TMP112_ERR_RETURN_TYPE setTlowInt(int16_t newValue, bool is13bit=false) { return(writeInt(TMP112_TLOW, newValue, is13bit)); }
  /**
   * set the High temperature threshold (from a float)
   * @param newValue the desired temperature threshold (no safety checks performed in this function)
   * @param is13bit whether the sensor is in Extended Mode (13bit mode). Measurement data has an identifier bit, so no strict need to specify
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
   */
  TMP112_ERR_RETURN_TYPE setThigh(float newValue, bool is13bit=false) { return(setThighInt(newValue * 16, is13bit)); } // temp in celsius divide by 0.0625 (a.k.a. multiply by 16)
  /**
   * set the Low temperature threshold (from a float)
   * @param newValue the desired temperature threshold (no safety checks performed in this function)
   * @param is13bit whether the sensor is in Extended Mode (13bit mode). Measurement data has an identifier bit, so no strict need to specify
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
   */
  TMP112_ERR_RETURN_TYPE setTlow(float newValue, bool is13bit=false) { return(setTlowInt(newValue * 16, is13bit)); } // temp in celsius divide by 0.0625 (a.k.a. multiply by 16)
  uint16_t getThighInt(bool is13bit=false) { return(requestReadInt(TMP112_THIGH, is13bit)); } // just a macro
  uint16_t getTlowInt(bool is13bit=false) { return(requestReadInt(TMP112_TLOW, is13bit)); } // just a macro
  /**
   * retrieve the High temperature threshold
   * @param is13bit whether the sensor is in Extended Mode (13bit mode). Measurement data has an identifier bit, so no strict need to specify
   * @return temperature threshold value in degrees Celsius
   */
  float getThigh(bool is13bit=false) { return((float)getThighInt(is13bit) * 0.0625); } // just a macro
  /**
   * retrieve the Low temperature threshold
   * @param is13bit whether the sensor is in Extended Mode (13bit mode). Measurement data has an identifier bit, so no strict need to specify
   * @return temperature threshold value in degrees Celsius
   */
  float getTlow(bool is13bit=false) { return((float)getTlowInt(is13bit) * 0.0625); } // just a macro

  // editing the conf register can be done using the functions below, or you can do something manually like: "writeBytes(TMP112_CONF, confRawBytes, 2)"
  // MSByte of CONF register:
  TMP112_ERR_RETURN_TYPE setSD(bool newValue) { // ShutDown mode    default=0
    uint8_t currentReg;
    requestReadBytes(TMP112_CONF, &currentReg, 1); // bit of a hack, only read the MSByte
    currentReg &= ~TMP112_SD_bits; //erase old value
    currentReg |= (newValue * TMP112_SD_bits); //insert new value
    return(writeBytes(TMP112_CONF, &currentReg, 1));
  }
  TMP112_ERR_RETURN_TYPE setTM(bool newValue) { // Thermostat Mode    default=0
    uint8_t currentReg;
    requestReadBytes(TMP112_CONF, &currentReg, 1); // bit of a hack, only read the MSByte
    currentReg &= ~TMP112_TM_bits; //erase old value
    currentReg |= (newValue * TMP112_TM_bits); //insert new value
    return(writeBytes(TMP112_CONF, &currentReg, 1));
  }
  TMP112_ERR_RETURN_TYPE setPOL(bool newValue) { // POLarity    default=0
    uint8_t currentReg;
    requestReadBytes(TMP112_CONF, &currentReg, 1); // bit of a hack, only read the MSByte
    currentReg &= ~TMP112_POL_bits; //erase old value
    currentReg |= (newValue * TMP112_POL_bits); //insert new value
    return(writeBytes(TMP112_CONF, &currentReg, 1));
  }
  /**
   * set the Fault Queue threshold
   * @param newValue 2-bit value, results in [1,2,4,6] fault threshhold respectively 
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
   */
  TMP112_ERR_RETURN_TYPE setFQ(uint8_t newValue) { // Fault Queue    default=0 (= 1 consecutive faults)
    uint8_t currentReg;
    requestReadBytes(TMP112_CONF, &currentReg, 1); // bit of a hack, only read the MSByte
    currentReg &= ~TMP112_FQ_bits; //erase old value
    currentReg |= ((newValue<<3) & TMP112_FQ_bits); //insert new value
    return(writeBytes(TMP112_CONF, &currentReg, 1));
  }
  /**
   * set the One-Shot bit. Writing a 1 to it while in ShutDown mode triggers a single conversion.
   *  During the conversion it reads as 0, once it's done it reads as 1. See datasheet for details
   * @param newValue (boolean) bit 
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
   */
  TMP112_ERR_RETURN_TYPE setOS(bool newValue) { // One-Shot bit    default=0
    uint8_t currentReg;
    requestReadBytes(TMP112_CONF, &currentReg, 1); // bit of a hack, only read the MSByte
    currentReg &= ~TMP112_OS_bits; //erase old value
    currentReg |= (newValue * TMP112_OS_bits); //insert new value
    return(writeBytes(TMP112_CONF, &currentReg, 1));
  }
  // LSByte of CONF register:
  TMP112_ERR_RETURN_TYPE setEM(bool newValue) { // Extended Mode (13bit)    default=0
    uint8_t currentReg[2];
    requestReadBytes(TMP112_CONF, currentReg, 2); // to get to the LSByte you have to read the whole register
    currentReg[1] &= ~TMP112_EM_bits; //erase old value
    currentReg[1] |= (newValue * TMP112_EM_bits); //insert new value
    return(writeBytes(TMP112_CONF, currentReg, 2));
  }
  /**
   * set the Conversion Rate
   * @param newValue 2-bit value, results in [0.25,1,4,8] conversions/sec respectively
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
   */
  TMP112_ERR_RETURN_TYPE setCR(uint8_t newValue) { // Conversion Rate    default=2 (= 4 conv/sec)
    uint8_t currentReg[2];
    requestReadBytes(TMP112_CONF, currentReg, 2); // to get to the LSByte you have to read the whole register
    currentReg[1] &= ~TMP112_CR_bits; //erase old value
    currentReg[1] |= ((newValue<<6) & TMP112_CR_bits); //insert new value
    return(writeBytes(TMP112_CONF, currentReg, 2));
  }

  // MSByte of CONF register:
  bool getSD() { uint8_t currentReg; requestReadBytes(TMP112_CONF, &currentReg, 1); return(currentReg & TMP112_SD_bits); } // ShutDown mode
  bool getTM() { uint8_t currentReg; requestReadBytes(TMP112_CONF, &currentReg, 1); return(currentReg & TMP112_TM_bits); } // Thermostat Mode
  bool getPOL() { uint8_t currentReg; requestReadBytes(TMP112_CONF, &currentReg, 1); return(currentReg & TMP112_POL_bits); } // POLarity
  /**
   * retrieve the Fault Queue threshold
   * @return 2-bit value, results in [1,2,4,6] fault threshhold respectively 
   */
  uint8_t getFQ() { uint8_t currentReg; requestReadBytes(TMP112_CONF, &currentReg, 1); return((currentReg & TMP112_FQ_bits) >> 3); } // Fault Queue
  uint8_t getRES() { uint8_t currentReg; requestReadBytes(TMP112_CONF, &currentReg, 1); return((currentReg & TMP112_RES_bits) >> 5); } // RESolution (READ ONLY). Should always return 3 (0x03 0b00000011) for TMP112
  bool getOS() { uint8_t currentReg; requestReadBytes(TMP112_CONF, &currentReg, 1); return(currentReg & TMP112_OS_bits); } // One-Shot
  // LSByte of CONF register:
  bool getEM() { uint8_t currentReg[2]; requestReadBytes(TMP112_CONF, currentReg, 2); return(currentReg[1] & TMP112_EM_bits); } // Extended Mode (13bit)
  bool getAL() { uint8_t currentReg[2]; requestReadBytes(TMP112_CONF, currentReg, 2); return(currentReg[1] & TMP112_AL_bits); } // Alert (READ ONLY)
  /**
   * retrieve the Conversion Rate
   * @return 2-bit value, results in [0.25,1,4,8] conversions/sec respectively
   */
  uint8_t getCR() { uint8_t currentReg[2]; requestReadBytes(TMP112_CONF, currentReg, 2); return((currentReg[1] & TMP112_CR_bits) >> 6); } // Conversion Rate 2-bit value, results in [0.25,1,4,8] conversions/sec respectively

  /**
   * retrieve the configuration register and check the resolution bits (Read-Only) to see if the connection is working properly
   * @return true if reading was successful and resolution bits were as expected
   */
  bool connectionCheck() { // returns true if the read was successfull and the RES matches 3 (like the datasheet claims it always should be for the TMP112)
    uint8_t readBuff[2];
    TMP112_ERR_RETURN_TYPE readSuccess = requestReadBytes(TMP112_CONF, readBuff, 2); // first, check if the read was successfull at all
    if(!_errGood(readSuccess)) { return(false); }
    uint8_t RES = (readBuff[0] & TMP112_RES_bits)>>5;
    return(RES == 3); // should always be 3 (according to the TMP112 datasheet)
  }

  /**
   * print out all the configuration values of the sensor (just for debugging)
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
   */
  TMP112_ERR_RETURN_TYPE printConfig() { // prints contents of CONF register (somewhat efficiently)
    uint8_t readBuff[2]; // instead of calling all the functions above (which may actually take hundreds of millis at the lowest I2C freq this things supports), just get the CONF register once
    TMP112_ERR_RETURN_TYPE readSuccess = requestReadBytes(TMP112_CONF, readBuff, 2);
    if(_errGood(readSuccess))
     {
      //Serial.println("printing config: ");
      Serial.print("ShutDown mode: "); Serial.println(readBuff[0] & TMP112_SD_bits);
      Serial.print("Thermostat Mode: "); Serial.println((readBuff[0] & TMP112_TM_bits)>>1);
      Serial.print("POLatiry: "); Serial.println((readBuff[0] & TMP112_POL_bits)>>2);
      static const uint8_t FQbitsToThreshold[4] = {1,2,4,6};
      uint8_t FQbits = (readBuff[0] & TMP112_FQ_bits)>>3 ; // note: should also prevent array index overflow, by its very nature
      Serial.print("Fault Queue: "); Serial.print(FQbits); Serial.print(" = "); Serial.println(FQbitsToThreshold[FQbits]);
      Serial.print("RESolution (should==3): "); Serial.println((readBuff[0] & TMP112_RES_bits)>>5);
      Serial.print("One-Shot status: "); Serial.println((readBuff[0] & TMP112_OS_bits)>>1);
      bool is13bit = readBuff[1] & TMP112_EM_bits;
      Serial.print("Thermostat Mode: "); Serial.println(is13bit);
      Serial.print("Alert (active==!POL): "); Serial.println((readBuff[1] & TMP112_AL_bits)>>5);
      static const float CRbitsToThreshold[4] = {0.25,1,4,8};
      uint8_t CRbits = (readBuff[1] & TMP112_CR_bits)>>6; // note: should also prevent array index overflow, by its very nature
      Serial.print("Convertsion Rate: "); Serial.print(CRbits); Serial.print(" = "); Serial.println(CRbitsToThreshold[CRbits]);
      // also print Thigh and Tlow (assuming 'is13bit' is working correctly):
      int16_t ThighInt = getThighInt(is13bit); // assume that, since the first reading went successfully, this one will as well
      Serial.print("Thigh threshold: "); Serial.print(ThighInt); Serial.print(" = "); Serial.println((float)ThighInt * 0.0625);
      int16_t TlowInt = getTlowInt(is13bit); // assume that, since the first reading went successfully, this one will as well
      Serial.print("Tlow threshold: "); Serial.print(TlowInt); Serial.print(" = "); Serial.println((float)TlowInt * 0.0625);
    } else {
      Serial.println("TMP112 failed to read CONF register!");
    }
    return(readSuccess);
  }

  /**
   * write the defualt value (according to the datasheet) to the CONF register.
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
   */
  TMP112_ERR_RETURN_TYPE resetConfig() { static uint8_t CONF_default[2]={0x60,0xA0}; return(writeBytes(TMP112_CONF, CONF_default, 2)); } // write the default value (according to datasheet) to CONF register

  /**
   * (intended as example) start a One-Shot measurement (while in ShutDown mode), wait for- and retrieve the measurement
   * @return temperature measurement in degrees Celsius
   */
  float getOneShotWait() { // request and wait for a one-shot measurement (only intended for use while device is in shutdown mode)
    setOS(true); // start a One-Shot measurement
    // technically, the OS bit turns 0 during the conversion, and 1 once it is done. But since the minimum I2C freq is so low that communication might actually take more than 26ms... just waiting works enough
    // while(!getOS()) {} // wait for the OS bit to turn to 1, indicating the conversion is done
    delay(26+1); // according to the datasheet, "A single conversion typically occurs for 26 ms". I added 1ms just to be sure
    //bool is13bit = getEM(); // whether a measurement is 13bit could be decided by using getEM() (or by getting the LSByte when checking the OS bit earlier), or.... 
    // TODO: see if LSBit of LSByte of measurement indicates res
    return(getTemp(/* is13bit */)); // retrieve the temperature measurement
  }

  /**
   * request data from the sensor without specifying a register. It will use the last-addressed register, which is hopefully NOT TMP112_CONF
   *  note: the register pointer of the TMP112 does not change on its own, so after a requestRead call, you can use onlyRead calls for a slight speed increase
   *  DO NOTE HOWEVER, that the burden of proper usage is now really on the user; No more handholding
   *  I'm not sure why anyone would need a temperature measurement to be read slightly quicker (at more risk), but whatever, that's not my business
   * @param is13bit whether the sensor is in Extended Mode (13bit mode). Only measurement data has an identifier bit
   * @return temperature measurement in degrees Celsius
   */
  float onlyReadFloat(bool is13bit=false) { return((float)onlyReadInt(is13bit) * 0.0625); }
};

#endif  // TMP112_thijs_h