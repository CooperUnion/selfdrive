#include <stdio.h>
#include <stdint.h>
#include <esp_attr.h>

#include "bat.h"
#include "ember_common.h"
#include "cuber_base.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"

//spi libraries
#include <driver/gpio.h>
#include <driver/spi_common.h> 
#include <driver/spi_master.h>

// ######        DEFINES        ###### //
#define BATTERY_CLK_FRQ 100 //100 Hz

#define COUTHI1   0x09
#define COUTHI2   0x0a
#define COUTHI3   0x0b
#define COUTHI4   0x0c
#define COUTLO1   0x0d
#define COUTLO2   0x0e
#define COUTLO3   0x0f
#define COUTLO4   0x10

#define PROD_ID     0x00
#define MAX22530_ID 0x81

#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */

#define SCLK 14
#define CS 15
#define SDI 13 //serial data input
#define SDO 12 //serial data output

#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */



// ######      PROTOTYPES       ###### //

static void bat_init();
static void burst_read_register(uint8_t regAddress); 
uint16_t read_register(uint8_t regAddress);
uint16_t write_register(uint8_t regAddress, uint16_t regValue);

esp_err_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);
esp_err_t readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data);

// ######     PRIVATE DATA      ###### //

spi_device_handle_t handle;

// ######    RATE FUNCTIONS     ###### //

ember_rate_funcs_S module_rf = {
    .call_init  = bat_init,
};


static void bat_init() {

    //spi bus initiliaztion
    esp_err_t err = ESP_OK; 

    spi_bus_config_t spi_bus = {

        .mosi_io_num = SDI, //master out slave in (output from master)
        .miso_io_num = SDO, //master in slave out (output from slave)
        .sclk_io_num = SCLK, //clock
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

    //intialize with SPI1 and disabling DMA for SPI (what is DMA)
    err = spi_bus_initialize(SPI2_HOST, &spi_bus, SPI_DMA_DISABLED);
    if(err != ESP_OK) { goto cleanup; }

    //initialize device
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


    err = spi_bus_add_device(SPI2_HOST, &dev_cfg, &handle); 
    if(err != ESP_OK) { goto cleanup; }


    //sensor initialization
    uint8_t status = 0x1;

    //checking if device is present
    if ((read_register(PROD_ID) != MAX22530_ID)) { status = 0x0; }

    printf("MAX2253x status = ");
    printf("%d", (int)status); /* Answer: 1 when the device is initialized and the ID is read and recognized */

    if (status == 1) {

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

cleanup:
    printf("cleaning up stuff\n"); 
}


void burst_read_register(uint8_t regAddress) {

	uint8_t Burst_reg1, Burst_reg2, Burst_reg3, Burst_reg4, Burst_INT_status;
	uint32_t frame = (uint32_t)((regAddress << 2) + (1 << 0)) ;

	readBytes(handle, frame, 2, &Burst_reg1); 
	readBytes(handle, frame + 0xF, 2, &Burst_reg2); 
	readBytes(handle, frame + 0x1F, 2, &Burst_reg3); 
	readBytes(handle, frame + 0x2F, 2, &Burst_reg4); 
	
	/*
	Burst_reg1 = SPI.transfer16(0);
	Burst_reg2 = SPI.transfer16(0);
	Burst_reg3 = SPI.transfer16(0);
	Burst_reg4 = SPI.transfer16(0);
	Burst_INT_status = SPI.transfer16(0);*/
}


//function to read from a register
uint16_t read_register(uint8_t regAddress) {
  
  uint32_t frame = (uint32_t)(regAddress << 2); 
  uint8_t buffer, result;  
  
  esp_err_t err = readBytes(handle, frame, 1, &buffer); 
  if (err) return -1; 
  
  err = readBytes(handle, 0, 2, &result); 
  if (err) return -1; 

  return result; 
}

//function to write to a register
uint16_t write_register(uint8_t regAddress, uint16_t regValue) {
// Register_address address = COUTH1, data = 8B32h or 35634d
  uint32_t data_frame1 = 0x0000;

  data_frame1 = (uint32_t)((regAddress << 2) + (1 << 1)); 
  
  //write_byte(data_frame1);
  //SPI.transfer16(regValue);

  return 1; 
}

// ######   PRIVATE FUNCTIONS   ###### //
esp_err_t writeBytes(spi_device_handle_t handle_, uint8_t regAddr, size_t length, const uint8_t *data) {

	if(length == 0) return ESP_ERR_INVALID_SIZE;
	
	spi_transaction_t transaction = {
		.flags = 0,
		.cmd = 0,
		.addr = regAddr & SPIBUS_WRITE,
		.length = length * 8,
		.rxlength = 0,
		.user = NULL,
		.tx_buffer = data,
		.rx_buffer = NULL,
	};
	esp_err_t err = spi_device_transmit(handle_, &transaction);    
	return err;
}

//reading bits logic
esp_err_t readBytes(spi_device_handle_t handle_, uint8_t regAddr, size_t length, uint8_t *data) {

	if(length == 0) return ESP_ERR_INVALID_SIZE;

	spi_transaction_t transaction = {
		.flags = 0,
		.cmd = 0,
		.addr = regAddr | SPIBUS_READ,
		.length = length * 8,
		.rxlength = length * 8,
		.user = NULL,
		.tx_buffer = NULL,
		.rx_buffer = data,
	};
	esp_err_t err = spi_device_transmit(handle_, &transaction);    

	return err;
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //
