/* RheoLink.h
   Define RheoLink Class
   https://www.idex-hs.com/docs/default-source/product-manuals/rheolink-i2c-communication-protocol-for-titanex.pdf.zip
   Control and IDEX selector valve using I2C

   This class implements the folowing features:
        - Initialization
        - Set new I2C address
        - Set position (with direction of rotation)
        - Get current position
        - Report status

   Class Members:
    Variables:
        uint8_t address_ - I2C address of the device
        TwoWire *w_ - Pointer to Wire object
    Functions:
        RheoLink: set the private variables
        bool begin: Initialize the device. Return false if failed
        uint8_t send_command: Send a command to the selector valve and return an error
        uint8_t read_register: Read a register in the selector valve and return its value or an error

       Created on: 8/10/2023
         Author: Kevin Marx
*/
#ifndef RHEOLINK_H_
#define RHEOLINK_H_

#include <Wire.h>
#include <Arduino.h>

/* Error codes:
              99 – valve failure (valve can not be homed)
              88 – non-volatile memory error
              77 – valve configuration error or command mode error
              66 – valve positioning error
              55 – data integrity error
              44 – data CRC error
              3x – Problem connecting over I2C
                  31 – Data too long for buffer
                  32 – NACK on address tx
                  33 – NACK on data tx
                  34 – other error
                  35 – timeout
                  36 – other other error
              22 – Not initialized
              current valve position (1 to N) otherwise
*/

// Commands - accessible outside this file
enum RheoLinkCommand_t {
  RheoLink_POS = 'P',
  RheoLink_CW = '-',
  RheoLink_CCW = '+',
  RheoLink_NEW_ADDR = 'N',
  RheoLink_STATUS = 'S'
};
// Definitions for this file only
#define RheoLink_DUMMY_DATA 'x'
#define RheoLink_TIMEOUT 2000

class RheoLink {
  public:
    RheoLink();
    uint8_t begin(TwoWire &w, uint8_t address);
    uint8_t send_command(RheoLinkCommand_t cmd, uint8_t data = RheoLink_DUMMY_DATA);
    uint8_t read_register(RheoLinkCommand_t target);
    uint8_t block_until_done(uint32_t timeout = RheoLink_TIMEOUT);
  private:
    uint8_t address_;
    TwoWire *w_;
    bool init_;
};

#undef RheoLink_DUMMY_DATA
#undef RheoLink_TIMEOUT
#endif /* RHEOLINK_H_ */
