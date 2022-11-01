
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
STM32 alternate pins testing (& little table in a comment to help remind me which ones i can use...)
put on github
add arduino example sketch
add example sketch filenames to library.json
test ESP32 with useWireLib   also, test if 'static' vars in the ESP32 code actually are static (connect 2 sensors?)
test 328p (both optimized and wireLib)
*/

#ifndef TMP112_thijs_h
#define TMP112_thijs_h

#include "Arduino.h" // always import Arduino.h
#ifndef TMP112_useWireLib // (note: ifNdef!) if this has been defined by the user, then don't do all this manual stuff
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) // TODO: test 328p processor defines! (also, this code may be functional on other AVR hw as well?)
    // nothing to import, all ATMega328P registers are imported by default
  #elif defined(ARDUINO_ARCH_ESP32)
    #include "driver/i2c.h"
  #elif defined(__MSP430FR2355__) //TBD: determine other MSP430 compatibility: || defined(ENERGIA_ARCH_MSP430) || defined(__MSP430__)
    #include <msp430.h>
    extern "C" {
      #include "twi.h"
    }
  #elif defined(ARDUINO_ARCH_STM32)
    extern "C" {
      #include "utility/twi.h"
    }
  #else // if the platform does not have optimized code
    #warning("using Wire library for TMP112 (no platform optimized code available)")
    #define TMP112_useWireLib  // use Wire as a backup
  #endif
#endif

#ifdef TMP112_useWireLib // note: this ifdef check is done after the ifndef, so the compiler gets a chance to define it anyway
  #include <Wire.h>
  // note: check the defined BUFFER_LENGTH in Wire.h for the max transmission length (on many platforms)
#endif

//#define TMP112debugPrint(x)  Serial.println(x)
//#define TMP112debugPrint(x)  log_d(x)   //using the ESP32 debug printing

#ifndef TMP112debugPrint
  #define TMP112debugPrint(x)  ;
#endif

#ifndef TMP112_ERR_RETURN_TYPE  // unless the user already defined it manually
  #define TMP112_ERR_RETURN_TYPE_default  bool
  #ifdef TMP112_useWireLib
    #define TMP112_ERR_RETURN_TYPE  TMP112_ERR_RETURN_TYPE_default
  #elif defined(ARDUINO_ARCH_ESP32) // the ESP32 likes to spit out esp_err_t for most things
    #define TMP112_ERR_RETURN_TYPE  esp_err_t
    #define TMP112_return_esp_err_t   // to let the code below know that the return type is an esp_err_t
  #elif defined(ARDUINO_ARCH_STM32)
    #define TMP112_ERR_RETURN_TYPE  i2c_status_e
    #define TMP112_return_i2c_status_e // to let the code below know that the return type is an esp_err_t
  #else
    #define TMP112_ERR_RETURN_TYPE  TMP112_ERR_RETURN_TYPE_default
  #endif
#endif

// TMP112 address lookup table:
#define TMP112_ADDR_A0_to_GND  0x48
#define TMP112_ADDR_A0_to_VCC  0x49
#define TMP112_ADDR_A0_to_SDA  0x4A
#define TMP112_ADDR_A0_to_SCL  0x4B
const uint8_t TMP112_ADDR_TABLE[4] = {TMP112_ADDR_A0_to_GND, TMP112_ADDR_A0_to_VCC, TMP112_ADDR_A0_to_SDA, TMP112_ADDR_A0_to_SCL}; // A0 connected to GND, V+, SDA or SCL respectively

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

#define TW_WRITE 0 //https://en.wikipedia.org/wiki/I%C2%B2C  under "Addressing structure"
#define TW_READ  1

#define ACK_CHECK_EN 1  //only for ESP32
#define ACK_CHECK_DIS 0 //ack check disable does not seem to work??? (it always checks for an ack and spits out )

#define SIZEOF_I2C_CMD_DESC_T  20  //a terrible fix for a silly problem. The actual struct/typedef code is too far down the ESP32 dependancy rabbithole.
#define SIZEOF_I2C_CMD_LINK_T  20  //if the ESP32 didn't have such a fragmented filestructure this wouldn't be a problem maybe...
/* the reason why i need those 2 defines:   //this code was derived from https://github.com/espressif/esp-idf/blob/master/components/driver/i2c.c
uint8_t buffer[sizeof(i2c_cmd_desc_t) + sizeof(i2c_cmd_link_t) * numberOfOperations] = { 0 };
i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
*/

/**
 * An I2C interfacing library for the TMP112 temperature sensor from Texas Instruments
 * 
 * features the option to use the Wire.h library, or optimized code for the following platforms:
 * atmega328p (direct register manipulation)
 * ESP32 (below HAL layer, but not lowest level)
 * MSP430 (through Energia(?) middle layer)
 * STM32 (through twi->HAL layers)
 */
class TMP112_thijs
{
  public:
  //// I2C constants:
  const uint8_t slaveAddress; //7-bit address
  const uint8_t SLA_W; // used by several platform-optimized sections.
  const uint8_t SLA_R; // might as well define as constant and avoid repeated caclulation (honestly insignificant, but whatever)
  TMP112_thijs(const uint8_t address) : slaveAddress(address), SLA_W(( address <<1) | TW_WRITE), SLA_R(( address <<1) | TW_READ) {}
  
  #ifdef TMP112_useWireLib // higher level generalized (arduino wire library):

    public:

    /**
     * initialize I2C peripheral through the Wire.h library
     * @param frequency SCL clock freq in Hz
     */
    void init(uint32_t frequency) {
      Wire.begin(); // init I2C as master
      Wire.setClock(frequency); // set the (approximate) desired clock frequency. Note, may be affected by pullup resistor strength (on some microcontrollers)
    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToRead register byte (see list of defines at top)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it wrote/read successfully
     */
    bool requestReadBytes(uint8_t registerToRead, uint8_t readBuff[], uint16_t bytesToRead) {
      // ideally, i'd use the Wire function: requestFrom(address, quantity, iaddress, isize, sendStop), which lets you send the register through iaddress
      // HOWEVER, this function is not implemented on all platforms (looking at you, MSP430!), and it's not that hard to do manually anyway, so:
      Wire.beginTransmission(slaveAddress);
      Wire.write(registerToRead);
      Wire.endTransmission(); // the generalized Wire library is not always capable of repeated starts (on all platforms)
      return(onlyReadBytes(readBuff, bytesToRead));
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it read successfully
     */
    bool onlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
      Wire.requestFrom(slaveAddress, bytesToRead);
      if(Wire.available() != bytesToRead) { TMP112debugPrint("onlyReadBytes() received insufficient data"); return(false); }
      for(uint16_t i=0; i<bytesToRead; i++) { readBuff[i] = Wire.read(); } // dumb byte-by-byte copy
      // unfortunately, TwoWire.rxBuffer is a private member, so we cant just memcpy. Then again, this implementation is not meant to be efficient
      return(true);
    }
    
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToWrite register byte (see list of defines at top)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return whether it wrote successfully
     */
    bool writeBytes(uint8_t registerToWrite, uint8_t writeBuff[], uint16_t bytesToWrite) {
      Wire.beginTransmission(slaveAddress);
      Wire.write(registerToWrite);
      Wire.write(writeBuff, bytesToWrite); // (usually) just calls a forloop that calls .write(byte) for every byte.
      Wire.endTransmission(); // this implementation does not really handle repeated starts (on all platforms)
      return(true);
    }

  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) // TODO: test 328p processor defines! (also, this code may be functional on other AVR hw as well?)
    private:
    //// I2C constants:
    static const uint8_t twi_basic = (1<<TWINT) | (1<<TWEN); //any action will feature these 2 things (note TWEA is 0)
    static const uint8_t twi_START = twi_basic | (1<<TWSTA);
    static const uint8_t twi_STOP  = twi_basic | (1<<TWSTO);
    static const uint8_t twi_basic_ACK = twi_basic | (1<<TWEA); //(for master receiver mode) basic action, repond with ACK (if appropriate)
    
    static const uint8_t twi_SR_noPres = 0b11111000; //TWSR (stas register) without prescalebits
    // status register contents (master mode)
    static const uint8_t twi_SR_M_START = 0x08;      //start condition has been transmitted
    static const uint8_t twi_SR_M_RESTART = 0x10;    //repeated start condition has been transmitted
    static const uint8_t twi_SR_M_SLA_W_ACK = 0x18;  //SLA+W has been transmitted, ACK received
    static const uint8_t twi_SR_M_SLA_W_NACK = 0x20; //SLA+W has been transmitted, NOT ACK received
    static const uint8_t twi_SR_M_DAT_T_ACK = 0x28;  //data has been transmitted, ACK received
    static const uint8_t twi_SR_M_DAT_T_NACK = 0x30; //data has been transmitted, NOT ACK received
    static const uint8_t twi_SR_M_arbit = 0x38;      //arbitration
    static const uint8_t twi_SR_M_SLA_R_ACK = 0x40;  //SLA+R has been transmitted, ACK received
    static const uint8_t twi_SR_M_SLA_R_NACK = 0x48; //SLA+R has been transmitted, NOT ACK received
    static const uint8_t twi_SR_M_DAT_R_ACK = 0x50;  //data has been received, ACK returned
    static const uint8_t twi_SR_M_DAT_R_NACK = 0x58; //data has been received, NOT ACK returned
    // status register contents (slave mode)
    static const uint8_t twi_SR_S_SLA_W_ACK = 0x60;  //own address + W has been received, ACK returned
    static const uint8_t twi_SR_S_arbit_SLA_W = 0x68;//arbitration
    static const uint8_t twi_SR_S_GEN_ACK = 0x70;    //general call + W has been received, ACK returned
    static const uint8_t twi_SR_S_arbit_GEN = 0x78;  //arbitration
    static const uint8_t twi_SR_S_DAT_SR_ACK = 0x80; //data has been received after SLA+W, ACK returned
    static const uint8_t twi_SR_S_DAT_SR_NACK = 0x88;//data has been received after SLA+W, NOT ACK returned
    static const uint8_t twi_SR_S_DAT_GR_ACK = 0x90; //data has been received after GEN+W, ACK returned
    static const uint8_t twi_SR_S_DAT_GR_NACK = 0x98;//data has been received after GEN+W, NOT ACK returned
    static const uint8_t twi_SR_S_prem_STOP_RE =0xA0;//a STOP or repeated_START condition has been received prematurely (page 193)
    static const uint8_t twi_SR_S_SLA_R_ACK = 0xA8;  //own address + R has been received, ACK returned
    static const uint8_t twi_SR_S_arbit_SLA_R = 0xB0;//arbitration
    static const uint8_t twi_SR_S_DAT_ST_ACK = 0xB8; //data has been transmitted, ACK received     (master receiver wants more data)
    static const uint8_t twi_SR_S_DAT_ST_NACK = 0xC0;//data has been transmitted, NOT ACK received (master receiver doesnt want any more)
    static const uint8_t twi_SR_S_DAT_STL_ACK = 0xC8;//last (TWEA==0) data has been transmitted, ACK received (data length misconception)
    // status register contents (miscellaneous states)
    static const uint8_t twi_SR_nothing = twi_SR_noPres; //(0xF8) no relevant state info, TWINT=0
    static const uint8_t twi_SR_bus_err = 0; //bus error due to an illigal start/stop condition (if this happens, set TWCR to STOP condition)

    /*  what the ACK bit does (and what the status registers read if ACK is used wrong/unexpectedly):
    after a START, in response to an address byte, the slave uses ACK if (TWEA=1) it accepts the communication in general
    during data transferrence (either direction) the ACK/NOT-ACK is used to let the other side know whether or not they want more data
    if the recipient sends an ACK, it expects more data
    if the master transmitter ran out of data to send to the slave receiver, the slave status register will read 0xA0 (twi_SR_S_STOP_RESTART)
    if the slave transmitter ran out of data to send to the master receiver, the slave status register will read 0xC8 (twi_SR_S_DAT_STL_ACK)
        in that case, the slave transmitter will send all 1's untill STOP (or RESTART)
    in cases where there is too much data (from either side), the NOT-ACK will just be received earlier than expected
        if the slave sends NOT-ACK early, the master should STOP/RESTART the transmission (or the slave should ignore the overflowing data)
        if the master sends NOT-ACK early, the slave doesnt have to do anything (except maybe raise an error internally)
    
    in general, the TWEA (Enable Ack) bit should be synchronized in both devices (except for master transmitter, which doesnt use it).
    in master receiver, TWEA signals to the slave that the last byte is received, and the transmission will end
    in both slave modes, if TWEA==0, the slave expects for there to be a STOP/RESTART next 'tick', if not, the status register will read 0 (twi_SR_bus_err)
    */
    
    inline void twoWireTransferWait() { while(!(TWCR & (1<<TWINT))); }
    #define twoWireStatusReg      (TWSR & twi_SR_noPres)

    inline void twiWrite(uint8_t byteToWrite) {
      TWDR = byteToWrite;
      TWCR = twi_basic; //initiate transfer
      twoWireTransferWait();
    }
    
    inline bool startWrite() {
      TWCR = twi_START; //send start
      twoWireTransferWait();
      twiWrite(SLA_W);
      if(twoWireStatusReg != twi_SR_M_SLA_W_ACK) { TMP112debugPrint("SLA_W ack error"); TWCR = twi_STOP; return(false); }
      return(true);
    }

    inline bool startRead() {
      TWCR = twi_START; //repeated start
      twoWireTransferWait();
      twiWrite(SLA_R);
      if(twoWireStatusReg != twi_SR_M_SLA_R_ACK) { TMP112debugPrint("SLA_R ack error"); TWCR = twi_STOP; return(false); }
      return(true);
    }

    public:

    /**
     * initialize I2C peripheral
     * @param frequency SCL clock freq in Hz
     * @return frequency it was able to set
     */
    uint32_t init(uint32_t frequency) {
      // set frequency (SCL freq = F_CPU / (16 + 2*TWBR*prescaler) , where prescaler is 1,8,16 or 64x, see page 200)
      TWSR &= 0b11111000; //set prescaler to 1x
      //TWBR  = 12; //set clock reducer to 400kHz (i recommend external pullups at this point)
      #define prescaler 1
      TWBR = ((F_CPU / frequency) - 16) / (2*prescaler);
      uint32_t reconstFreq = F_CPU / (16 + (2*TWBR*prescaler));
      //Serial.print("freq: "); Serial.print(frequency); Serial.print(" TWBR:"); Serial.print(TWBR); Serial.print(" freq: "); Serial.println(reconstFreq);
      // the fastest i could get I2C to work is 800kHz (with another arduino as slave at least), which is TWBR=2 (with some 1K pullups)
      // any faster and i get SLA_ACK errors.
      return(reconstFreq);
    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToRead register byte (see list of defines at top)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it wrote/read successfully
     */
    bool requestReadBytes(uint8_t registerToRead, uint8_t readBuff[], uint16_t bytesToRead) {
      if(!startWrite()) { return(false); }
      twiWrite(registerToRead);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      if(!startRead()) { return(false); }
      for(uint16_t i=0; i<(bytesToRead-1); i++) {
        TWCR = twi_basic_ACK; //request several bytes
        twoWireTransferWait();
        //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { TMP112debugPrint("DAT_R Ack error"); return(false); }
        readBuff[i] = TWDR;
      }
      TWCR = twi_basic; //request 1 more byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { TMP112debugPrint("DAT_R Nack error"); return(false); }
      readBuff[bytesToRead-1] = TWDR;
      TWCR = twi_STOP;
      return(true);
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it read successfully
     */
    bool onlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
      if(!startRead()) { return(false); }
      for(uint16_t i=0; i<(bytesToRead-1); i++) {
        TWCR = twi_basic_ACK; //request several bytes
        twoWireTransferWait();
        //if(twoWireStatusReg != twi_SR_M_DAT_R_ACK) { TMP112debugPrint("DAT_R Ack error"); return(false); }
        readBuff[i] = TWDR;
      }
      TWCR = twi_basic; //request 1 more byte
      twoWireTransferWait();
      //if(twoWireStatusReg != twi_SR_M_DAT_R_NACK) { TMP112debugPrint("DAT_R Nack error"); return(false); }
      readBuff[bytesToRead-1] = TWDR;
      TWCR = twi_STOP;
      return(true);
    }
    
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToWrite register byte (see list of defines at top)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return whether it wrote successfully
     */
    bool writeBytes(uint8_t registerToWrite, uint8_t writeBuff[], uint16_t bytesToWrite) {
      if(!startWrite()) { return(false); }
      twiWrite(registerToWrite);  //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      for(uint8_t i=0; i<bytesToWrite; i++) {
        twiWrite(writeBuff[i]);
        //if(twoWireStatusReg != twi_SR_M_DAT_T_ACK) { return(false); } //should be ACK(?)
      }
      TWCR = twi_STOP;
      return(true);
    }
  
  #elif defined(ARDUINO_ARCH_ESP32)
    // see my AS5600 library for notes on the ESP32's mediocre I2C peripheral
    
    public:
    //// I2C constants:
    i2c_port_t I2Cport = 0;
    uint32_t I2Ctimeout = 10; //in millis
    //const TickType_t I2CtimeoutTicks = 100 / portTICK_RATE_MS; //timeout (divide by portTICK_RATE_MS to convert millis to the right format)
    //uint8_t constWriteBuff[1]; //i2c_master_write_read_device() requires a const uint8_t* writeBuffer. You can make this array bigger if you want, shouldnt really matter
    
    /**
     * initialize I2C peripheral
     * @param frequency SCL clock freq in Hz
     * @param SDApin GPIO pin to use as SDA
     * @param SCLpin GPIO pin to use as SCL
     * @param I2CportToUse which of the ESP32's I2C peripherals to use
     * @return (esp_err_t) whether it was able to establish the peripheral
     */
    esp_err_t init(uint32_t frequency, int SDApin=21, int SCLpin=22, i2c_port_t I2CportToUse = 0) {
      if(I2CportToUse < I2C_NUM_MAX) { I2Cport = I2CportToUse; } else { TMP112debugPrint("can't init(), invalid I2Cport!"); return(ESP_ERR_INVALID_ARG); }
      i2c_config_t conf;
      conf.mode = I2C_MODE_MASTER;
      conf.sda_io_num = SDApin;
      conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
      conf.scl_io_num = SCLpin;
      conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
      conf.master.clk_speed = frequency;
      //conf.clk_flags = 0;          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
      esp_err_t err = i2c_param_config(I2Cport, &conf);
      if (err != ESP_OK) { TMP112debugPrint("can't init(), i2c_param_config error!"); TMP112debugPrint(esp_err_to_name(err)); return(err); }
      return(i2c_driver_install(I2Cport, conf.mode, 0, 0, 0));
    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToRead register byte (see list of defines at top)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return (esp_err_t or bool) whether it wrote/read successfully
     */
    TMP112_ERR_RETURN_TYPE requestReadBytes(uint8_t registerToRead, uint8_t readBuff[], uint16_t bytesToRead) {
//      const uint8_t numberOfCommands = 8; //start, write, write, start, write, read_ACK, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
//      i2c_master_write_byte(cmd, registerToRead, ACK_CHECK_DIS);
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read(cmd, readBuff, bytesToRead, I2C_MASTER_LAST_NACK);
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);

      //constWriteBuff[0] = registerToRead;
      esp_err_t err = i2c_master_write_read_device(I2Cport, slaveAddress, &registerToRead, 1, readBuff, bytesToRead, I2Ctimeout / portTICK_RATE_MS); //faster (seems to work fine)
      if(err != ESP_OK) { TMP112debugPrint(esp_err_to_name(err)); }
      #ifdef TMP112_return_esp_err_t
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return (esp_err_t or bool) whether it read successfully
     */
    TMP112_ERR_RETURN_TYPE onlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
//      const uint8_t numberOfCommands = 5; //start, write, write, start, write, read_ACK, read_NACK, stop
//      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
//      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
//      i2c_master_start(cmd);
//      i2c_master_write_byte(cmd, SLA_R, ACK_CHECK_EN);
//      i2c_master_read(cmd, readBuff, bytesToRead, I2C_MASTER_LAST_NACK);
//      i2c_master_stop(cmd);
//      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
//      i2c_cmd_link_delete_static(cmd);

      esp_err_t err = i2c_master_read_from_device(I2Cport, slaveAddress, readBuff, bytesToRead, I2Ctimeout / portTICK_RATE_MS);  //faster?
      if(err != ESP_OK) { TMP112debugPrint(esp_err_to_name(err)); }
      #ifdef TMP112_return_esp_err_t
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }
    
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToWrite register byte (see list of defines at top)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return (esp_err_t or bool) whether it wrote successfully
     */
    TMP112_ERR_RETURN_TYPE writeBytes(uint8_t registerToWrite, uint8_t writeBuff[], uint16_t bytesToWrite) {
      const uint8_t numberOfCommands = 5; //start, write, write, write, stop
      uint8_t CMDbuffer[SIZEOF_I2C_CMD_DESC_T + SIZEOF_I2C_CMD_LINK_T * numberOfCommands] = { 0 };
      i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(CMDbuffer, sizeof(CMDbuffer)); //create a CMD sequence
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, SLA_W, ACK_CHECK_EN);
      i2c_master_write_byte(cmd, registerToWrite, ACK_CHECK_DIS);
      i2c_master_write(cmd, writeBuff, bytesToWrite, ACK_CHECK_DIS);
      i2c_master_stop(cmd);
      esp_err_t err = i2c_master_cmd_begin(I2Cport, cmd, I2Ctimeout / portTICK_RATE_MS);
      i2c_cmd_link_delete_static(cmd);
      // probably slightly slower:
//      uint8_t copiedArray[bytesToWrite+1]; copiedArray[0]=registerToWrite; for(uint8_t i=0;i<bytesToWrite;i++) { copiedArray[i+1]=writeBuff[i]; }
//      esp_err_t err = i2c_master_write_to_device(I2Cport, slaveAddress, copiedArray, bytesToWrite+1, I2Ctimeout / portTICK_RATE_MS);
      if(err != ESP_OK) { TMP112debugPrint(esp_err_to_name(err)); }
      #ifdef TMP112_return_esp_err_t
        return(err);
      #else
        return(err == ESP_OK);
      #endif
    }

  #elif defined(__MSP430FR2355__) //TBD: determine other MSP430 compatibility: || defined(ENERGIA_ARCH_MSP430) || defined(__MSP430__)

    public:

    /**
     * initialize I2C peripheral
     * @param frequency SCL clock freq in Hz
     */
    void init(uint32_t frequency) {
      //twi_setModule(module); // the MSP430 implementation of I2C is slightly janky. Instead of different classes for different I2C interfaces, they have a global variable indicating which module is targeted
      // the default module is all i'm going to need for my uses, but if you wanted to use multiple I2C peripherals, please uncomment all the twi_setModule() things and add a module constant to each TMP112_thijs obj
      twi_init();
      twi_setClock(frequency);
    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToRead register byte (see list of defines at top)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it wrote/read successfully
     */
    bool requestReadBytes(uint8_t registerToRead, uint8_t readBuff[], uint16_t bytesToRead) {
      //twi_setModule(module);  // see init() for explenation
      int8_t ret = twi_writeTo(slaveAddress, &registerToRead, 1, 1, true); // transmit 1 byte, wait for the transmission to complete and send a STOP command
      if(ret != 0) { TMP112debugPrint("requestReadBytes() twi_writeTo error!"); return(false); }
      return(onlyReadBytes(readBuff, bytesToRead));
    }
    
    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return whether it read successfully
     */
    bool onlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
      uint8_t readQuantity = twi_readFrom(slaveAddress, readBuff, bytesToRead, true); // note: sendstop=true
      if(readQuantity != bytesToRead) { TMP112debugPrint("onlyReadBytes() received insufficient data"); return(false); }
      return(true);
    }
    
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToWrite register byte (see list of defines at top)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return  whether it wrote successfully
     */
    bool writeBytes(uint8_t registerToWrite, uint8_t writeBuff[], uint16_t bytesToWrite) {
      uint8_t bufferCopyWithReg[bytesToWrite+1];   bufferCopyWithReg[0] = registerToWrite;
      for(uint8_t i=0;i<bytesToWrite; i++) { bufferCopyWithReg[i+1] = writeBuff[i]; } // manually copy all bytes
      int8_t ret = twi_writeTo(slaveAddress, bufferCopyWithReg, bytesToWrite+1, 1, true); // transmit some bytes, wait for the transmission to complete and send a STOP command
      if(ret != 0) { TMP112debugPrint("writeBytes() twi_writeTo error!"); return(false); }
      return(true);
      // NOTE; i'd love to just send one byte, then send the writeBuff, but the MSP430 twi library is not made for that.
      // calling twi_writeTo always calls a start condition, and a repeated start causes the AS5600 to look for a register again i think.
      // underwater, all the MSP430 twi library does is fill a buffer and start an operation which calls an ISR,
      //  but i can't even insert one byte in the buffer before the rest, because there is no function for that (also, twi_writeTo clears the buffer before adding to it).
      // so, I'm just stuck copying the writeBuff to yet another buffer. Luckily, the TMP112 only accepts 2-byte data anyway, so it's a small buffer...
    }

  #elif defined(ARDUINO_ARCH_STM32)
  
    /* Notes on the STM32 I2C perihperal (specifically that of the STM32WB55):
    Much like the ESP32, the STM32 libraries are built on several layers of abstraction.
    The twi.h library goes to some HAL library, and i have no intention of finding out where it goes from there.
    Since this particular implementation does not need to be terribly fast, i'll just stick with twi.h,
     which does have the advantage of working with the whole STM32 family (whereas a lower implementation would target specific subfamilies)
    The STM32 can map the I2C pins to a limited (but at least more than one) selection of pins,
     see PeripheralPins.c for the PinMap_I2C_SDA and PinMap_I2C_SCL (or just look at the datasheet)
     (for my purposes, that's: .platformio\packages\framework-arduinoststm32\variants\STM32WBxx\WB55R(C-E-G)V\PeripheralPins.c )
     Here is a handy little table:
      I2C1: SDA: PA10, PB7, PB9
            SCL: PA9, PB6, PB8
      I2C3: SDA: PB4, PB11, PB14, PC1
            SCL: PA7 PB10, PB13, PC0
    
    */

    public:

    i2c_t _i2c; // handler thingy (presumably)
    static const uint8_t STM32_MASTER_ADDRESS = 0x01; // a reserved address which tells the peripheral it's a master, not a slave
    
    /**
     * initialize I2C peripheral on STM32
     * @param frequency SCL clock freq in Hz
     * @param SDApin pin (arduino naming) to use as SDA (select few possible)
     * @param SCLpin pin (arduino naming) to use as SCL (select few possible)
     * @param generalCall i'm honestly not sure, the STM32 twi library is not documented very well...
     */
    void init(uint32_t frequency, uint32_t SDApin=PIN_WIRE_SDA, uint32_t SCLpin=PIN_WIRE_SCL, bool generalCall = false) {
      _i2c.sda = digitalPinToPinName(SDApin);
      _i2c.scl = digitalPinToPinName(SCLpin);
      _i2c.__this = (void *)this; // i truly do not understand the stucture of the STM32 i2c_t, but whatever, i guess the i2c_t class needs to know where this higher level class is or something
      _i2c.isMaster = true;
      _i2c.generalCall = (generalCall == true) ? 1 : 0; // 'generalCall' is just a uint8_t instead of a bool
      i2c_custom_init(&_i2c, frequency, I2C_ADDRESSINGMODE_7BIT, (STM32_MASTER_ADDRESS << 1)); // this selects which I2C peripheral is used based on what pins you entered
      // note: use i2c_setTiming(&_i2c, frequency) if you want to change the frequency later
    }
    
    /**
     * request a specific register and read bytes into a buffer
     * @param registerToRead register byte (see list of defines at top)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return (i2c_status_e or bool) whether it wrote/read successfully
     */
    TMP112_ERR_RETURN_TYPE requestReadBytes(uint8_t registerToRead, uint8_t readBuff[], uint16_t bytesToRead) {
      #if defined(I2C_OTHER_FRAME) // not on all STM32 variants
        _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // (this one i don't understand, but the Wire.h library does it, and without it i get HAL_I2C_ERROR_SIZE~~64 (-> I2C_ERROR~~4))
      #endif
      i2c_status_e err = i2c_master_write(&_i2c, (slaveAddress << 1), &registerToRead, 1);
      if(err != I2C_OK) {
        TMP112debugPrint("requestReadBytes() i2c_master_write error!");
        #ifdef TMP112_return_i2c_status_e
          return(err);
        #else
          return(false);
        #endif
      }
      return(onlyReadBytes(readBuff, bytesToRead));
    }

    /**
     * read bytes into a buffer (without first writing a register value!)
     * @param readBuff a buffer to store the read values in
     * @param bytesToRead how many bytes to read
     * @return (i2c_status_e or bool) whether it read successfully
     */
    TMP112_ERR_RETURN_TYPE onlyReadBytes(uint8_t readBuff[], uint16_t bytesToRead) {
      #if defined(I2C_OTHER_FRAME) // if the STM32 subfamily is capable of writing without sending a stop
        _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // tell the peripheral it should send a STOP at the end
      #endif
      i2c_status_e err = i2c_master_read(&_i2c, (slaveAddress << 1), readBuff, bytesToRead);
      if(err != I2C_OK) { TMP112debugPrint("onlyReadBytes() i2c_master_read error!"); }
      #ifdef TMP112_return_i2c_status_e
        return(err);
      #else
        return(err == I2C_OK);
      #endif
    }
    
    /**
     * request a specific register and write bytes from a buffer
     * @param registerToWrite register byte (see list of defines at top)
     * @param writeBuff a buffer of bytes to write to the device
     * @param bytesToWrite how many bytes to write
     * @return (i2c_status_e or bool) whether it wrote successfully
     */
    TMP112_ERR_RETURN_TYPE writeBytes(uint8_t registerToWrite, uint8_t writeBuff[], uint16_t bytesToWrite) {
      //// NOTE: the code below is commented out because it concerns a potential optimization (repeated start instead of buffer copying), but it's untested! (and not really needed anyway)
      // #if defined(I2C_OTHER_FRAME) // if the STM32 subfamily is capable of writing without sending a stop
      //   _i2c.handle.XferOptions = I2C_OTHER_FRAME; // tell the peripheral it should NOT send a STOP at the end
      //   i2c_status_e err = i2c_master_write(&_i2c, (slaveAddress << 1), &registerToWrite, 1);
      //   if(err != I2C_OK) {
      //     TMP112debugPrint("requestReadBytes() first i2c_master_write error!");
      //     #ifdef TMP112_return_i2c_status_e
      //       return(err);
      //     #else
      //       return(false);
      //     #endif
      //   }
      //   _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // tell the peripheral it should send a STOP at the end
      //   err = i2c_master_write(&_i2c, (slaveAddress << 1), writeBuff, bytesToWrite);
      // #else // if the STM32 subfamily can't handle repeated starts anyways, just do it the hard way:
      //// if the code above is uncommented, be sure to remove this next part:
      
      #if defined(I2C_OTHER_FRAME) // if the STM32 subfamily is capable of writing without sending a stop
        _i2c.handle.XferOptions = I2C_OTHER_AND_LAST_FRAME; // tell the peripheral it should send a STOP at the end
      #endif

        uint8_t bufferCopyWithReg[bytesToWrite+1];   bufferCopyWithReg[0] = registerToWrite;
        for(uint8_t i=0;i<bytesToWrite; i++) { bufferCopyWithReg[i+1] = writeBuff[i]; } // manually copy all bytes
        i2c_status_e err = i2c_master_write(&_i2c, (slaveAddress << 1), bufferCopyWithReg, bytesToWrite+1);

      // #endif // related to the commented optimization code above

      if(err != I2C_OK) { TMP112debugPrint("writeBytes() i2c_master_write error!"); }
      #ifdef TMP112_return_i2c_status_e
        return(err);
      #else
        return(err == I2C_OK);
      #endif
    }

  #else
    #error("should never happen, platform optimization code has issue (probably at the top there)")
  #endif // platform-optimized code end

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
    #if defined(TMP112_return_esp_err_t)
      if(readSuccess != ESP_OK)
    #elif defined(TMP112_return_i2c_status_e)
      if(readSuccess != I2C_OK)
    #else
      if(!readSuccess)
    #endif
     { return(false); }
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
    #if defined(TMP112_return_esp_err_t)
      if(readSuccess == ESP_OK)
    #elif defined(TMP112_return_i2c_status_e)
      if(readSuccess == I2C_OK)
    #else
      if(readSuccess)
    #endif
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