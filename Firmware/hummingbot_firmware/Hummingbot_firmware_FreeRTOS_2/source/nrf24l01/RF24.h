/*
 * RF24.h
 *
 *  Created on: Apr 28, 2019
 *      Author: jackxu
 */

#ifndef RF24_H_
  #define RF24_H_

  /************* Macro Preference ***************/
  #define USE_SPI_BUFFER        0
  #define RF24_SPI_TRANSACTIONS 1

  /************* Macro GPIO Configurations ***************/
  //#define RF24_CE_PIN; /**< "Chip Enable" pin, activates the RX or TX role */
  //#define RF24_CSN_PIN; /**< SPI Chip select */

  /************* ENUM List ***************/
  /**
   * Power Amplifier level.
   *
   * For use with setPALevel()
   */
  typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

  /**
   * Data rate.  How fast data moves through the air.
   *
   * For use with setDataRate()
   */
  typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

  /**
   * CRC Length.  How big (if any) of a CRC is included.
   *
   * For use with setCRCLength()
   */
  typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;


  /************* Public Functions ***************/
  void RF24_onDestroy(void);
  /**
   * @param _cepin The pin attached to Chip Enable on the RF module
   * @param _cspin The pin attached to Chip Select
   * @param spispeed For RPi, the SPI speed in MHZ ie: BCM2835_SPI_SPEED_8MHZ
   */
  void RF24_config(pin_t* _cepin, pin_t* _cspin);

  uint8_t RF24_getChannel(void);
  bool RF24_isChipConnected(void);
  bool RF24_setDataRate(rf24_datarate_e speed);
  void RF24_setRetries(uint8_t delay, uint8_t count);
  void RF24_powerDown(void);
  void RF24_powerUp(void);

  void RF24_enableAckPayload(void);
  void RF24_openReadingPipe(uint8_t child, const uint8_t *address);
  void RF24_setPALevel(uint8_t level);
  void RF24_startListening(void);

  bool RF24_available(void);
  bool RF24_init(void);
  
  void RF24_read( void* buf, uint8_t len );
#endif /* RF24_H_ */
