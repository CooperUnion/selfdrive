#include "eeprom.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include "node_pins.h"
#include <stdint.h>

// ######        DEFINES        ###### //

#define EEPROM_PAGESIZ  64
#define EEPROM_ADDR_I2C (0xa0 | (0x00 << 1))
#define EEPROM_ADDR_MAX 0x7fff
#define EEPROM_SCL_GPIO NODE_BOARD_PIN_EEPROM_SCL
#define EEPROM_SDA_GPIO NODE_BOARD_PIN_EEPROM_SDA
#define EEPROM_WP_GPIO  NODE_BOARD_PIN_EEPROM_WP
#define EEPROM_WP(ENABLE) gpio_set_level(EEPROM_WP_GPIO, ENABLE)

#define I2C_ESP_INTR_FLAGS 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM     0
#define I2C_MASTER_RX_BUF  0
#define I2C_MASTER_TX_BUF  0
#define I2C_TICK_TIMEOUT   0
#define I2C_WRITE          0
#define I2C_READ           1

// ######      PROTOTYPES       ###### //

static esp_err_t sel_read(uint16_t addr);

// ######     PRIVATE DATA      ###### //

static uint16_t internal_addr;

// ######    RATE FUNCTIONS     ###### //

void eeprom_init()
{
    gpio_set_direction(EEPROM_WP_GPIO, GPIO_MODE_OUTPUT);
    EEPROM_WP(true);

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = EEPROM_SDA_GPIO,
        .scl_io_num       = EEPROM_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(
        I2C_MASTER_NUM,
        I2C_MODE_MASTER,
        I2C_MASTER_RX_BUF,
        I2C_MASTER_TX_BUF,
        I2C_ESP_INTR_FLAGS
    );

    // we want to know where we are in EEPROM
    // in case of an immediate address read
    // the following sets internal_addr to 0
    uint8_t tmp;
    eeprom_read(EEPROM_ADDR_MAX, &tmp, 1);
}

// ######   PRIVATE FUNCTIONS   ###### //

// sel_read attempts a selective read at the given address.
// can be used for acknowledge polling (to ensure EEPROM is not busy)
static esp_err_t sel_read(uint16_t addr)
{
    i2c_cmd_handle_t cmd;
    uint8_t buf[2];
    uint8_t data[1];
    esp_err_t val = ESP_OK;

    buf[0] = addr >> 8;
    buf[1] = addr & 0xff;

    cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR_I2C | I2C_WRITE, true);
    i2c_master_write(cmd, buf, sizeof(buf), true);
    i2c_master_start(cmd); // Missing end signal is intentional
    i2c_master_write_byte(cmd, EEPROM_ADDR_I2C | I2C_READ, true);
    i2c_master_read(cmd, data, 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    val = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TICK_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    return val;
}

// ######   PUBLIC FUNCTIONS    ###### //

esp_err_t eeprom_read(uint16_t addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd;
    uint8_t buf[2];
    esp_err_t val = ESP_OK;

    // mask MSB of the address to ensure
    // we can compare w/ internal_addr
    addr &= EEPROM_ADDR_MAX;

    buf[0] = addr >> 8;
    buf[1] = addr & 0xff;

    // ensure we can read
    while (true) {
        val = sel_read(internal_addr);
        if (val == ESP_OK) break;

        // when the EEPROM is busy writing it will
        // not send an ACK, returning ESP_FAIL
        if (val != ESP_FAIL) goto error;
    }

    cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;

    if (internal_addr != addr) { // selective read at the given address
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, EEPROM_ADDR_I2C | I2C_WRITE, true);
        i2c_master_write(cmd, buf, sizeof(buf), true);
        // missing end signal is intentional
    } // otherwise, do an immediate access read (address not required)

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR_I2C | I2C_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    val = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TICK_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    internal_addr = addr + len;
    internal_addr &= EEPROM_ADDR_MAX;

error:
    return val;
}

esp_err_t eeprom_write(uint16_t addr, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd;
    uint16_t page;
    uint8_t offset, write_len;
    uint8_t buf[2];
    esp_err_t val = ESP_OK;

    while (len) {
        // mask MSB of the address to ensure correct calculations
        addr &= EEPROM_ADDR_MAX;

        page   = addr / EEPROM_PAGESIZ;
        offset = addr % EEPROM_PAGESIZ;
        buf[0] = page >> 2;
        buf[1] = ((page & 0x03) << 6) | offset;

        // we don't want to write too much
        // since we can only write maximum one page at at time
        write_len = EEPROM_PAGESIZ - offset;
        if (write_len > len) write_len = len;

        // ensure we can write
        while (true) {
            val = sel_read(internal_addr);
            if (val == ESP_OK) break;

            // when the EEPROM is busy writing it will
            // not send an ACK, returning ESP_FAIL
            if (val != ESP_FAIL) goto error;
        }

        cmd = i2c_cmd_link_create();
        if (!cmd) return ESP_ERR_NO_MEM;

        // WP is always enabled after sel_read() call
        EEPROM_WP(false);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, EEPROM_ADDR_I2C | I2C_WRITE, true);
        i2c_master_write(cmd, buf, sizeof(buf), true);
        i2c_master_write(cmd, data, write_len, true);
        i2c_master_stop(cmd);

        val = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TICK_TIMEOUT);
        i2c_cmd_link_delete(cmd);
        if (val != ESP_OK) goto error;

        addr += write_len;
        internal_addr = addr;
        internal_addr &= EEPROM_ADDR_MAX;

        data += write_len;
        len -= write_len;
    }

error:
    EEPROM_WP(true);

    return val;
}

esp_err_t eeprom_write_byte(uint16_t addr, const uint8_t data)
{
    i2c_cmd_handle_t cmd;
    uint8_t buf[3];
    esp_err_t val = ESP_OK;

    // ensure we can write
    while (true) {
        val = sel_read(internal_addr);
        if (val == ESP_OK) break;

        // when the EEPROM is busy writing it will
        // not send an ACK, returning ESP_FAIL
        if (val != ESP_FAIL) goto error;
    }

    buf[0] = addr >> 8;
    buf[1] = addr & 0xff;
    buf[2] = data;

    cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;

    // WP is always enabled after sel_read() call
    EEPROM_WP(false);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_ADDR_I2C | I2C_WRITE, true);
    i2c_master_write(cmd, buf, sizeof(buf), true);
    i2c_master_stop(cmd);

    val = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TICK_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    EEPROM_WP(true);

    internal_addr = addr + 1;
    internal_addr &= EEPROM_ADDR_MAX;

error:
    return val;
}
