#include "battery.h"

#include <driver/gpio.h>

#include "ember_taskglue.h"
#include "opencan_tx.h"
#include "opencan_rx.h"


#include <SPI.h> //idk how to import this
SPIClass SPI1(HSPI); //need to declare which SPI instance you want to use for ESP32


// ######        DEFINES        ###### //

//HSPI	MOSI-GPIO 13	MISO-GPIO 12	SCLK-GPIO 14	CS-GPIO 15
#define SCLK 14
#define CS 15
#define SDI 13 //serial data input
#define SDO 12 //serial data output

uint16_t Burst_reg1;
uint16_t Burst_reg2;
uint16_t Burst_reg3;
uint16_t Burst_reg4;


// ######      PROTOTYPES       ###### //
 
static void burst_read_register(); 



// ######     PRIVATE DATA      ###### //

// ######    RATE FUNCTIONS     ###### //

static void battery_init();
static void battery_100Hz(); 


ember_rate_funcs_S module_rf = {
    .call_init  = battery_init,
    .call_100Hz = battery_100Hz,
};



static void battery_init() {

  SPI1.begin(SCLK, SDI, SDO, CS);
}


static void battery_100Hz() {

    bool dbw_active = base_dbw_active();

    if (dbw_active && !CANRX_is_node_CTRL_ok()) {
        base_set_state_estop(0 /* dummy value, API will change */);
    }

}


// ######   PRIVATE FUNCTIONS   ###### //

/* transfers 8 bits from SPI MOSI to 8 bit int*/
uint8_t transfer_bits8(uint8_t starting_address) {

  uint8_t result = 0; 
  for(int i = 0; i < sizeof(uint8_t); i++) {

    uint8_t current_byte = SPI.transfer(starting_address + i); 
    result |= (current_byte << i); 
  }

  return result; 
}



/* transfers 16 bits from SPI MOSI to 16 bit int*/
uint16_t transfer_bits16(uint8_t starting_address) {

  uint8_t result = 0; 
  for(int i = 0; i < sizeof(uint16_t); i++) {

    uint8_t current_byte = SPI.transfer(starting_address + i); 
    result |= (current_byte << i); 
  }

  return result; 
}


void burst_read_registrar(uint8_t address) {

  uint8_t initial_info = transfer_bits8(address);

  Burst_reg1 = transfer_bits16(address + 8);
  Burst_reg2 = transfer_bits16(address + 24);
  Burst_reg3 = transfer_bits16(address + 40); 
  Burst_reg4 = transfer_bits16(address + 56); 
}


// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //















