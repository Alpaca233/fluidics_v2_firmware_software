/* RheoLink.h
   Define RheoLink Class
   https://www.idex-hs.com/docs/default-source/product-manuals/rheolink-i2c-communication-protocol-for-titanex.pdf.zip
   Control and IDEX selector valve using I2C

    Functions:
        RheoLink: Set up the class
            Arguments: uint8_t address
            Returns: None
        bool begin: Initialize the device. Return false if failed
            Arguments: TwoWire *w_
            Returns: If begun successfully, return true
        bool send_command: Send a command to the 
            Arguments: uint8_t pos - the new position
            Returns: Return true if successful
        uint8_t get_position: Get current position and error status
            Arguments: None
            Returns:
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

       Created on: 8/10/2023
         Author: Kevin Marx
*/
#include "RheoLink.h"

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: RheoLink() initializes the selector valve class

  OPERATION:   None

  ARGUMENTS: None

  RETURNS: NONE

  INPUTS / OUTPUTS: NONE

  LOCAL VARIABLES: NONE

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
RheoLink::RheoLink()
{
  init_ = false;
  return;
}


/*
  -----------------------------------------------------------------------------
  DESCRIPTION: begin() initializes the I2C bus

  OPERATION:   We set the I2C address. The Wire library handles setting the low bit for read/write so we shift the address down a bit. Then, we store the I2C bus object pointer to a local variable

  ARGUMENTS:
      TwoWire *w:      Pointer to TwoWire object for I2C
      uint8_t address:      I2C write address of the pump

  RETURNS: 
      uint8_t err: I2C transmission error code

  INPUTS / OUTPUTS: NONE

  LOCAL VARIABLES: NONE

  SHARED VARIABLES:
     TwoWire *w_: pointer to stream object, written to and read from
     uint8_t address_: I2C address, written to and read from
     uint8_t pos_min, pos_max: written to

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint8_t RheoLink::begin(TwoWire &w, uint8_t address, uint8_t p_min, uint8_t p_max) {

  address_ = address >> 1;
  
  w_ = &w;
  w_->begin();
  w_->beginTransmission(address_);

  init_ = true;

  pos_min = p_min;
  pos_max = p_max;

  return w_->endTransmission(address_);
}
/*
  -----------------------------------------------------------------------------
  DESCRIPTION: send_command() sends a command with retry logic for robust communication

  OPERATION:   We initialize the checksum with the "write" address, then send the command and data, updating the checksum. We send the checksum and check for transmission errors. If an error occurs, we retry up to RheoLink_MAX_RETRIES times with a delay between attempts.

  ARGUMENTS:
      uint8_t cmd: The command
      int8_t data: The data

  RETURNS:
      uint8_t err: The I2C error code (0 = success, non-zero = error after all retries exhausted).

  INPUTS / OUTPUTS: Data is sent over I2C.

  LOCAL VARIABLES:
      uint8_t checksum: Stores the checksum
      uint8_t err:      Stores the error value
      uint8_t retry_count: Tracks number of retry attempts

  SHARED VARIABLES:
     TwoWire *w_: pointer to stream object

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint8_t RheoLink::send_command(RheoLinkCommand_t cmd, uint8_t data ) {
  uint8_t checksum;
  uint8_t err;
  uint8_t retry_count = 0;

  if (!init_){
    return 22;
  }
  
  // Retry loop for robust communication
  do {
    checksum = address_ << 1;

    w_->beginTransmission(address_);
    w_->write(cmd);
    checksum = checksum ^ cmd;
    w_->write(data);
    checksum = checksum ^ data;
    w_->write(checksum);

    err = w_->endTransmission(true);

    // there's a problem with the firmware on the selector valve - open and close a dummy transmission to terminate the command
    w_->beginTransmission(0);
    w_->endTransmission();

    // If successful, break out of retry loop
    if (err == 0) {
      break;
    }

    // If we have retries left, wait and try again
    if (retry_count < RheoLink_MAX_RETRIES) {
      delay(RheoLink_RETRY_DELAY);
      retry_count++;
    }

  } while (retry_count <= RheoLink_MAX_RETRIES);
  
  return err;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: read_register() reads a value from the selector valve with retry logic.

  OPERATION:   We send the I2C command to request the register, then attempt to read the response. Both the command transmission and data request operations include retry logic for robust communication.

  ARGUMENTS:
      RheoLinkCommand_t target: The register/command to read

  RETURNS:
      uint8_t err: The register value or error code (after all retries exhausted).

  INPUTS / OUTPUTS: Data is sent and received over I2C.

  LOCAL VARIABLES:
      uint8_t err: Stores the error values
      uint8_t retry_count: Tracks number of retry attempts for requestFrom
      uint8_t bytes_received: Number of bytes received from requestFrom

  SHARED VARIABLES:
     TwoWire *w_: pointer to stream object

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint8_t RheoLink::read_register(RheoLinkCommand_t target){
  if (!init_){
    return 22;
  }
  
  uint8_t err = this->send_command(target);

  // If there was a problem connecting, return an error
  if(err != 0){
    return 30 + err;
  }

  // Retry logic for requestFrom operation
  uint8_t retry_count = 0;
  uint8_t bytes_received = 0;

  do {
    // Request the data
    bytes_received = w_->requestFrom(address_, 3, true);

    // If we got data, break out of retry loop
    if(bytes_received > 0){
      break;
    }

    // If we have retries left, wait and try again
    if (retry_count < RheoLink_MAX_RETRIES) {
      delay(RheoLink_RETRY_DELAY);
      retry_count++;
    }

  } while (retry_count <= RheoLink_MAX_RETRIES);

  // If we didn't get any data after all retries, return an error
  if(bytes_received == 0){
    return 36;
  }

  // Parse the data RXed - we only care about the first byte
  err = w_->read();
  // Clear the buffer
  while(w_->available())
    w_->read();

  return err;
}


/*
  -----------------------------------------------------------------------------
  DESCRIPTION: block_until_done() blocks operation until the selector valve is done moving, then returns the error code.

  OPERATION:   We first record the initial time. Then, we read the the status register until we stop seeing a data read error or we time out. We then report the error.

  ARGUMENTS: NONE

  RETURNS:
      uint8_t err: The value or error code.

  INPUTS / OUTPUTS: Data is sent over I2C.

  LOCAL VARIABLES:
      uint8_t err: Stores the error values
      uint32_t t0: Stores the initial time

  SHARED VARIABLES:
     TwoWire *w_: pointer to stream object

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint8_t RheoLink::block_until_done(uint32_t timeout){
  if (!init_){
    return 22;
  }
  
  uint8_t err = this->read_register(RheoLink_STATUS);
  uint32_t t0 = millis();
  while((err < 40) and (err >=30) and ((millis() - t0) < timeout)){
    // If error is 30 - 39 AND we didn't time out yet, wait a bit and get the error again
    delay(5);
    err = this->read_register(RheoLink_STATUS);
  }
  return err;
}


/*
  -----------------------------------------------------------------------------
  DESCRIPTION: block_until_position_reached() blocks operation until the selector valve reaches a specific position, then returns the error code.

  OPERATION:   We first record the initial time and validate the target position. Then, we read the status register until we get the target position, encounter an error, or time out. We return the final status.

  ARGUMENTS:
      uint8_t pos: The target position to wait for (must be between pos_min and pos_max)
      uint32_t timeout: Maximum time to wait in milliseconds (default: RheoLink_TIMEOUT)

  RETURNS:
      uint8_t err: The final status or error code
                   pos - Success, valve reached target position
                   22 - Not initialized
                   11 - Position out of range
                   35 - Timeout waiting for position
                   Other error codes from read_register()

  INPUTS / OUTPUTS: Data is sent over I2C.

  LOCAL VARIABLES:
      uint8_t current_pos: Stores the current position readings
      uint32_t t0: Stores the initial time

  SHARED VARIABLES:
     TwoWire *w_: pointer to stream object
     pos_min, pos_max: minimum and maximum valid positions

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint8_t RheoLink::block_until_position_reached(uint8_t pos, uint32_t timeout){

  uint8_t current_pos = this->read_register(RheoLink_STATUS);
  uint32_t t0 = millis();

  while((millis() - t0) < timeout){
    // If we got the target position, return success
    if(current_pos == pos){
      return 0;
    }

    // Wait a bit and check position again
    delay(5);
    current_pos = this->read_register(RheoLink_STATUS);
  }

  // If we timed out, return timeout error
  return 35;
}


/*
  -----------------------------------------------------------------------------
  DESCRIPTION: set_position() sets the valve to a specific position with error handling

  OPERATION:   We first check if the device is initialized and if the position is within valid range.
               Then we send the position command and optionally wait for completion or specific position.

  ARGUMENTS:
      uint8_t pos: The target position (must be between pos_min and pos_max)
      bool wait_for_completion: Whether to block until operation completes (default: true)
      bool wait_for_position: If true, wait for exact position; if false, wait for movement to stop (default: false)
      uint32_t timeout: Maximum time to wait for completion in milliseconds (default: RheoLink_TIMEOUT)

  RETURNS:
      uint8_t err: Error code or success
                   0 - Success (or target position if wait_for_position=true)
                   22 - Not initialized
                   11 - Position out of range
                   35 - Timeout (if wait_for_position=true)
                   Other codes from send_command(), block_until_done(), or block_until_position_reached()

  INPUTS / OUTPUTS: Data is sent over I2C.

  LOCAL VARIABLES:
      uint8_t err: Stores the error values

  SHARED VARIABLES:
     pos_min, pos_max: minimum and maximum valid positions

  GLOBAL VARIABLES: None

  DEPENDENCIES: Wire.h
  -----------------------------------------------------------------------------
*/
uint8_t RheoLink::set_position(uint8_t pos, bool wait_for_completion, uint32_t timeout) {
  // Check if device is initialized
  if (!init_) {
    return 22;
  }

  // Validate position is within range
  if (pos < pos_min || pos > pos_max) {
    return 11; // Position out of range error
  }

  // Send the position command
  uint8_t err = this->send_command(RheoLink_POS, pos);

  // If there was an error sending the command, return it
  if (err != 0) {
    return err;
  }

  // If requested, wait for the valve to reach the specific position or just complete movement
  if (wait_for_completion)
      err = this->block_until_position_reached(pos, timeout);

  return err;
}