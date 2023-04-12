#include "battery.h"
#include <stdio.h>
#include <stdint.h>

#include <driver/gpio.h>

#include "ember_taskglue.h"
#include "opencan_tx.h"
#include "opencan_rx.h"

//spi libraries
#include <driver/spi_common.h> 
#include <driver/spi_master.h>


// ######        DEFINES        ###### //

#define BATTERY_CLK_FRQ 100 //100 Hz


#define PROD_ID     0x00
#define MAX22530_ID 0x81

#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */

#define SCLK 14
#define CS 15
#define SDI 13 //serial data input
#define SDO 12 //serial data output

uint16_t Burst_reg1;
uint16_t Burst_reg2;
uint16_t Burst_reg3;
uint16_t Burst_reg4;

// ######      PROTOTYPES       ###### //

//writing bits
esp_err_t writeBit(spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t data);
esp_err_t writeBits(spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
esp_err_t writeByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data);
esp_err_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);
    

//reading 
esp_err_t readBit(spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
esp_err_t readBits(spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
esp_err_t readByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t *data);
esp_err_t readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data);

// ######        CAN TX         ###### //

uint16_t read_register(uint8_t regAddress);
uint16_t write_register(uint8_t regAddress, uint16_t regValue);
uint8_t sensor_init(void);
void MAX22530_Reset(void);
float Convert_to_Voltage(uint8_t regAddress);

// ######     PRIVATE DATA      ###### //

// ######    RATE FUNCTIONS     ###### //

static void battery_init();
static void battery_100Hz(); 


ember_rate_funcs_S module_rf = {
    .call_init  = battery_init,
    .call_100Hz = battery_100Hz,
};



typedef struct battery_context_s { 
  
  spi_bus_config_t btry_cfg; 
  spi_host_device_t host_id; 
  spi_device_handle_t handle; 
} battery_context_t; 



static void battery_init() {
  
  esp_err_t err = ESP_OK; 

  battery_context_t *ctx = (battery_context_t *) malloc(sizeof(battery_context_t)); 
  //if (!ctx) return ESP_ERR_NO_MEM; 

  spi_bus_config_t spi_bus = {
  
    .mosi_io_num = -1, //master out slave in (output from master)
    .miso_io_num = -1, //master in slave out (output from slave)
    .sclk_io_num = -1, //clock
    //.data0_io_num = -1,
    //.data1_io_num = -1, 
    .data2_io_num = -1, 
    .data3_io_num = -1, 
    .data4_io_num = -1, 
    .data5_io_num = -1, 
    .data6_io_num = -1, 
    .data7_io_num = -1,
    //.quadwp_io_num = -1, //write protect signal
    //.quadhd_io_num = -1, //hold signal
    .max_transfer_sz = -1, //max trasnfer size in bytes, 4092 when DMA enabled or SOC_SPI_MAXIMUM_BUFFER_SIZE if DMA disabled
    .flags = SPICOMMON_BUSFLAG_MASTER,
    .intr_flags = -1 
  };

  ctx->btry_cfg = spi_bus; 
  ctx->host_id = SPI2_HOST;

  //intialize with SPI1 and disabling DMA for SPI (what is DMA)
  err = spi_bus_initialize(ctx->host_id, &ctx->btry_cfg, SPI_DMA_DISABLED);
  if(err != ESP_OK) {
    
    goto cleanup; 
  }

  spi_device_interface_config_t dev_cfg = {
    .address_bits = 8,
    .command_bits = 0,
    .dummy_bits = 0,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .duty_cycle_pos = 128,              
    .clock_speed_hz = BATTERY_CLK_FRQ,    //clock speed
    .mode = 0,                            //sampling occurs during rising edge of clock
    .spics_io_num = -1,                   //chip select pin
    .queue_size = 1,
    .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS,
    .pre_cb = NULL,
    .post_cb = NULL,
  };


  err = spi_bus_add_device(ctx->host_id, &dev_cfg, &ctx->handle); 
  if(err != ESP_OK) {

    goto cleanup; 
  }

  //sensor initialization
  sensor_init(); 


  cleanup:
    printf("cleaning up stuff\n"); 
}



void sensor_init() {

  uint8_t status = 0x1;

  //checking if device is present
  if ((MAX22530_read_register(PROD_ID) != MAX22530_ID)) {
    status = 0x0;
  }

  printf("MAX2253x status = ");
  printf(answer); /* Answer: 1 when the device is initialized and the ID is read and recognized */
  
  if (answer == 1) {
      
    printf("Device Recognized. Device Configuration ongoing");
    // Configuring the Digital Comparators
    /* Digital Input Mode with Unfiltered ADC results, setting upper threshold to 50% of range
     *  and lower threshold to 10% of range*/
    write_register(COUTHI1,0x0800);
    write_register(COUTLO1,0x019A);
    /* Digital Status Mode with filtered ADC results, setting upper threshold to 70% of range
     *  and lower threshold set to 40%*/
    write_register(COUTHI2,0xcb32);
    write_register(COUTLO2,0x0667);
    /* Writing default upper threshold values for COUTHI3 and COUTHI4
     * in Digital Status mode and using default lower threshold values*/
    write_register(COUTHI3,0x8b32);
    write_register(COUTHI3,0x8b32);
  }
}


static void battery_100Hz() {

    /*bool dbw_active = base_dbw_active();

    if (dbw_active && !CANRX_is_node_DBW_ok()) {
        base_set_state;
    }*/

}


// ######   PRIVATE FUNCTIONS   ###### //

uint16_t read_register(uint8_t regAddress) {
  
  uint32_t frame = (uint32_t)(regAddress << 2); 
  uint8_t buffer; 
  uint16_t result; 
  
  esp_err_t err = readByte(handle, frame, &buffer); 
  if (err) return -1; 
  
  err = readBytes(handle, 0, &result); 
  if (err) return -1; 

  return result; 
}


uint16_t write_register(uint8_t regAddress, uint16_t regValue) {
// Register_address address = COUTH1, data = 8B32h or 35634d
  uint32_t data_frame1 = 0x0000;

  data_frame1 = (uint32_t)((regAddress << 2) + (1 << 1)); 
  
  SPI.transfer8(data_frame1);
  SPI.transfer16(regValue);
}




// ######   PUBLIC FUNCTIONS    ###### //

//writing bits logic


esp_err_t writeBit(spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    
    uint8_t buffer;
    esp_err_t err = readByte(handle, regAddr, &buffer);
    if (err) return err;
    buffer = data ? (buffer | (1 << bitNum)) : (buffer & ~(1 << bitNum));
    return writeByte(handle, regAddr, buffer);
}

esp_err_t writeBits(spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    
    uint8_t buffer;
    esp_err_t err = readByte(handle, regAddr, &buffer);
    if (err) return err;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;
    return writeByte(handle, regAddr, buffer);
}

esp_err_t writeByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data) {
    
    return writeBytes(handle, regAddr, 1, &data);
}

esp_err_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data) {
    
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr & SPIBUS_WRITE;
    transaction.length = length * 8;
    transaction.rxlength = 0;
    transaction.user = NULL;
    transaction.tx_buffer = data;
    transaction.rx_buffer = NULL;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    
    return err;
}



//reading bits logic

esp_err_t readBit(spi_device_handle_t handle, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    
    return readBits(handle, regAddr, bitNum, 1, data);
}

esp_err_t readBits(spi_device_handle_t handle, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    
    uint8_t buffer;
    esp_err_t err = readByte(handle, regAddr, &buffer);
    if(!err) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        buffer &= mask;
        buffer >>= (bitStart - length + 1);
        *data = buffer;
    }
    return err;
}

esp_err_t readByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t *data) {
    
    return readBytes(handle, regAddr, 1, data);
}

esp_err_t readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) {
    
    if(length == 0) return ESP_ERR_INVALID_SIZE;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr | SPIBUS_READ;
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = NULL;
    transaction.rx_buffer = data;
    esp_err_t err = spi_device_transmit(handle, &transaction);    
    
    return err;
}



// ######        CAN TX         ###### //














