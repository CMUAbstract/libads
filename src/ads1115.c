#include <msp430.h>
#include <stdint.h>

#include <libio/console.h>

#include "ads1115.h"

// This is slave address b/c ADDR pin is connected to ground on capybara
#define ADS_SLAVE_ADDR 0x48 // 100 1000

// Only use this gain for now, currently the range of the banks we're measuring
// on capy is 0V - .475V Make sure this is never set such that the voltage is
// allowed to exceed VDD (+ some margin of error)
#define ADS_GAIN ADS1015_REG_CONFIG_PGA_0_512V



void restartTransmit(void){
	EUSCI_B_I2C_disable(EUSCI_B0_BASE);
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, ADS_SLAVE_ADDR);
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
//  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

	while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
}

/*
 *@brief writes a single value over i2c with appropriate waiting etc
 */
void writeSingleByte(uint8_t val){
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, val);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return;
}

/*
 *@brief reads a byte over i2c
 */
uint8_t readDataByte(){
	uint8_t val;
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
  val = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return val;
}

uint16_t ads_se_read(uint8_t channel) {
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, MAGNETOMETER_SLAVE_ADDRESS);

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  uint16_t config;
  config =  ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
            ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
            ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
            ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
            ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
            ADS1015_REG_CONFIG_MODE_SINGLE  | // Single-shot mode (default)
            ADS_GAIN                        | // ADS gain value
            ADS1015_REG_CONFIG_OS_SINGLE ;    // get single shot read now

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Write config register
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  /* 8 samples averaged (MA1 MA0 = 11), */
  /* 15hz data output (DO2 DO1 DO0 = 100), */
  /* normal measurement mode (MS0 MS1 = 00) */
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, ADS1015_REG_POINTER_CONFIG);
  // Send MSB of config
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, config >> 8);
  // Send LSB of config
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, config & 0xFF);
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  // Adding delay_cycles in here to handle the conversion delay
  __delay_cycles(8000);

  EUSCI_B_I2C_disable(EUSCI_B0_BASE);

  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, ADS_SLAVE_ADDR);

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, ADS1015_REG_POINTER_CONVERT);

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);

  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
  // Receive MSB then LSB
  uint16_t output;
  output = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE) << 8;
  output |= EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
  // Now stop
  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  return output;
}



