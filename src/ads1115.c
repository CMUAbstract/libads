#include <msp430.h>
#include <stdint.h>

#include <libio/console.h>
#include "libmspware/driverlib.h"
#include <libmspware/eusci_b_i2c.h>

#include "ads1115.h"

// This is slave address b/c ADDR pin is connected to ground on capybara
#define ADS_SLAVE_ADDR 0x48 // 100 1000

// Note-- we can't measure about 3.3V because that's the input voltage
// 125uV per division
#define ADS_GAIN ADS1015_REG_CONFIG_PGA_4_096V 


void restartTransmit(void){
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = ADS_SLAVE_ADDR; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable
  while (UCB0STATW & UCBBUSY); // is bus busy? then wait!
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
  P1OUT |= BIT0;
  P1DIR |= BIT0;
  P1OUT &= ~BIT0;
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, ADS1015_REG_POINTER_CONFIG);
  P1OUT |= BIT1;
  P1DIR |= BIT1;
  P1OUT &= ~BIT1;
  PRINTF("wrote reg\r\n");
  // Send MSB of config
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, config >> 8);
  P1OUT |= BIT1;
  P1DIR |= BIT1;
  P1OUT &= ~BIT1;
  PRINTF("wrote config\r\n");
  // Send LSB of config
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, config & 0xFF);
  P1OUT |= BIT1;
  P1DIR |= BIT1;
  P1OUT &= ~BIT1;
  PRINTF("wrote config2\r\n");
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  P1OUT |= BIT1;
  P1DIR |= BIT1;
  P1OUT &= ~BIT1;
  PRINTF("Sent stop\r\n");
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  P1OUT |= BIT0;
  P1DIR |= BIT0;
  P1OUT &= ~BIT0;
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


void ads_se_comp_setup(uint8_t channel) {
  uint16_t config;
  config = ADS1015_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1015_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode


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
  // Write low thresh
  restartTransmit();
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE,
    ADS1015_REG_POINTER_HITHRESH);
  // Set lower threshold
  // TODO make this configurable from makefile
  // Voltage divider (4M + 6M)
  // .125uV per div
  // Low thresh voltage = 2.2V
  // Code = (2.2*6/10)/(125*10^-6) = 0x2940
  uint16_t lowThresh = 0x2940;
  // Send MSB
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE,lowThresh >> 8);
  // Send LSB
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE,lowThresh & 0xFF);
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  
	restartTransmit();

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
  return;
}

// Useful in conjunction with comp_setup
// This reads the conversion that triggered an interrupt and in doing so, clears
// the interrupt
// TODO figure out how this works if we have multiple input voltages...
uint16_t ads_get_last_conversion() {
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
