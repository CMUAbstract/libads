#include <msp430.h>
#include <stdint.h>

#include <libio/console.h>
#include "libmspware/driverlib.h"
#include <libmspware/eusci_b_i2c.h>

#include "ads1115.h"

// This is slave address b/c ADDR pin is connected to ground on capybara
#define ADS_SLAVE_ADDR 0x48 // 100 1000

// Only use this gain for now, currently the range of the banks we're measuring
// on capy is 0V - .475V Make sure this is never set such that the voltage is
// allowed to exceed VDD (+ some margin of error)
#define ADS_GAIN ADS1015_REG_CONFIG_PGA_0_512V



void restartTransmit(void){
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = ADS_SLAVE_ADDR; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable
  while (UCB0STATW & UCBBUSY); // is bus busy? then wait!
}

static void writeSingle(uint8_t val){
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, val);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return;
}

static uint8_t readByte(){
	uint8_t val;
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
  val = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return val;
}


uint16_t ads_temp(uint8_t channel) {
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
  restartTransmit();
  PRINTF("Writing!\r\n");
  writeSingle(ADS1015_REG_POINTER_CONFIG);
  PRINTF("Writing1!\r\n");
  writeSingle(config >> 8);
  PRINTF("Writing2!\r\n");
  writeSingle(config & 0xFF);
  __delay_cycles(72000);
  PRINTF("Wrote!\r\n");
  restartTransmit();
  writeSingle(ADS1015_REG_POINTER_CONVERT);
  uint8_t val1, val0;
  val1 = readByte();
  val0 = readByte();
  uint16_t temp;
  temp = val1 << 8;
  temp += val0;
  return temp;
}


/*
 *@brief writes a single value over i2c with appropriate waiting etc
 */
void writeSingleByte(uint8_t reg, uint8_t val){
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  while((UCB0CTLW0 & UCTXSTT)); // wait for addr transmission to finish

  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0TXBUF = reg;

  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0TXBUF = val;

  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0CTLW0 |= UCTXSTP; // stop

  while (UCB0STATW & UCBBUSY); // wait until bus is quiet
  //EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  //EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, val);
  //while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return;
}

/*
 *@brief writes 2 bytes to a register over i2c
 */
void writeDataWord (uint8_t reg, uint8_t val1, uint8_t val2) {
  PRINTF("Writing word!\r\n");
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  //while((UCB0CTLW0 & UCTXSTT)); // wait for addr transmission to finish
  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty

  //Write reg
  UCB0TXBUF = reg;
  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  //Pause
  //while (UCB0STATW & UCBBUSY); // is bus busy? then wait!
  //PRINTF("Alive1\r\n");
  //while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0TXBUF = val1;

  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0TXBUF = val2;

  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0CTLW0 |= UCTXSTP; // stop

  //while (UCB0STATW & UCBBUSY); // wait until bus is quiet

  return;
}

/*
 *@brief reads a byte over i2c
 */
uint8_t readDataByte(uint8_t reg){
  while (UCB0STATW & UCBBUSY); // is bus busy? then wait!
  // Query gyro reg
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty

  UCB0TXBUF = reg; // fill txbuf with reg address

  while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty

  UCB0CTLW0 &= ~UCTR; // receive mode
  UCB0CTLW0 |= UCTXSTT; // repeated start

  // wait for addr transmission to finish, data transfer to start
  while(UCB0CTLW0 & UCTXSTT);

  UCB0CTLW0 |= UCTXSTP; // stop

  while(!(UCB0IFG & UCRXIFG)); // wait until txbuf is empty
  uint8_t val = UCB0RXBUF; // read out of rx buf

  while (UCB0STATW & UCBBUSY); // hang out until bus is quiet

	//uint8_t val;
	//EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  //EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
  //val = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
  //EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  //while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return val;
}


/*
 *@brief read data word
 */
uint16_t readDataWord(uint8_t reg) {
  while (UCB0STATW & UCBBUSY); // is bus busy? then wait!
  PRINTF("Alive0.1\r\n");
  // Query gyro reg
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  //while(UCB0CTLW0 & UCTXSTT);
  while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty
  PRINTF("Alive0\r\n");
  UCB0TXBUF = reg; // fill txbuf with reg address

  while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty
  PRINTF("Alive1\r\n");

  UCB0CTLW0 &= ~UCTR; // receive mode
  UCB0CTLW0 |= UCTXSTT; // repeated start

  // wait for addr transmission to finish, data transfer to start
  while(UCB0CTLW0 & UCTXSTT);
  PRINTF("Alive2\r\n");
  UCB0CTLW0 |= UCTXSTP; // stop

  while(!(UCB0IFG & UCRXIFG)); // wait until rxbuf is empty
  uint8_t val0 = UCB0RXBUF; // read out of rx buf
  while(!(UCB0IFG & UCRXIFG)); // wait until rxbuf is empty
  uint8_t val1 = UCB0RXBUF; // read out of rx buf
  UCB0CTLW0 |= UCTXSTP; // Stop!
  PRINTF("Alive3!\r\n");
  while (UCB0STATW & UCBBUSY); // hang out until bus is quiet
   
  // TODO figure out the byte ordering
	uint16_t word = val1 << 8;
  word += val0;
  return word;

}

uint16_t ads_se_read_new(uint8_t channel) {
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
  PRINTF("Config ready\r\n");
  //Yell i2c address
  restartTransmit();
  // Write config word
  writeDataWord(ADS1015_REG_POINTER_CONFIG, config >> 8, config & 0xFF);
  PRINTF("Wrote word!\r\n");
  __delay_cycles(72000);
  // Yell i2c address
  restartTransmit();
  while (UCB0STATW & UCBBUSY); // hang out until bus is quiet
  PRINTF("done wait\r\n");
  // Write read data word
  uint8_t reg = ADS1015_REG_POINTER_CONVERT;
  uint16_t ads_out;
  //PRINTF("Alive0.1\r\n");
  // Query gyro reg
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  //while(UCB0CTLW0 & UCTXSTT);
  while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty
  PRINTF("Alive0\r\n");
  UCB0TXBUF = reg; // fill txbuf with reg address

  while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty
  PRINTF("Alive1\r\n");

  UCB0CTLW0 &= ~UCTR; // receive mode
  UCB0CTLW0 |= UCTXSTT; // repeated start

  // wait for addr transmission to finish, data transfer to start
  while(UCB0CTLW0 & UCTXSTT);
  PRINTF("Alive2\r\n");
  UCB0CTLW0 |= UCTXSTP; // stop

  while(!(UCB0IFG & UCRXIFG)); // wait until rxbuf is empty
  uint8_t val0 = UCB0RXBUF; // read out of rx buf
  while(!(UCB0IFG & UCRXIFG)); // wait until rxbuf is empty
  uint8_t val1 = UCB0RXBUF; // read out of rx buf
  UCB0CTLW0 |= UCTXSTP; // Stop!
  PRINTF("Alive3!\r\n");
  while (UCB0STATW & UCBBUSY); // hang out until bus is quiet
   
  // TODO figure out the byte ordering
	uint16_t word = val1 << 8;
  word += val0;
  //ads_out = readDataWord(ADS1015_REG_POINTER_CONVERT);
  return ads_out; 
}


uint16_t ads_se_read(uint8_t channel) {
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
  printf("CONFIG: %x\r\n",config); 
	restartTransmit();

  // Write config register
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  /* 8 samples averaged (MA1 MA0 = 11), */
  /* 15hz data output (DO2 DO1 DO0 = 100), */
  /* normal measurement mode (MS0 MS1 = 00) */
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  PRINTF("Sent start\r\n");
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, ADS1015_REG_POINTER_CONFIG);
  PRINTF("wrote reg\r\n");
  // Send MSB of config
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, config >> 8);
  PRINTF("wrote config\r\n");
  // Send LSB of config
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, config & 0xFF);
  PRINTF("wrote config2\r\n");
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  PRINTF("Sent stop\r\n");
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  PRINTF("Waiting...\r\n");
  // Adding delay_cycles in here to handle the conversion delay
  // TODO figure out if we can sustain this...
  __delay_cycles(80000);
  PRINTF("dONE write\r\n");
  restartTransmit();
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, ADS1015_REG_POINTER_CONVERT);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

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



