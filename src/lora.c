#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/subghz.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "lora.h"
#include "common.h"

#define RF_SW_CTRL_GPIO_PORT                     GPIOA
#define RF_SW_CTRL_TXEN_PIN                      GPIO6
#define RF_SW_CTRL_RXEN_PIN                      GPIO7

#define HF_PA_CTRL1_PORT GPIOC
#define HF_PA_CTRL1_PIN  GPIO3
#define HF_PA_CTRL2_PORT GPIOC
#define HF_PA_CTRL2_PIN  GPIO4
#define HF_PA_CTRL3_PORT GPIOC
#define HF_PA_CTRL3_PIN  GPIO5

void print_reg_hex(char *prefix, uint8_t value);
void print_reg16_hex(char *prefix, uint16_t value);

#define SUBGHZ_NSS_LOOP_TIME ((24000000 * 24U) >> 16U)

bool g_deepsleep_enable = true;
subghz_t g_subghz_state;

/* Tx, RX, timeout flags (used in synchronous mode)*/
volatile bool g_flag_tx_done = false;
volatile bool g_flag_rx_done = false;
volatile bool g_flag_timeout = false;

void subghz_spi_init(int baudrate_prescaler);

static void lora_service(void *);
void lora_rf_switch_cb(bool);
extern void print_hex(uint8_t *src, int len);
BaseType_t woken = pdTRUE;

QueueHandle_t lora_rx_queue = NULL, lora_tx_queue = NULL;
/**
 * @brief Initialize SubGHZ peripheral.
 * @retval SUBGHZ_SUCCESS on success, SUBGHZ_ERROR otherwise.
 */

int
subghz_init(void)
{
	subghz_result_t res;

	/* Initialize subghz_t structure. */
	memset(&g_subghz_state, 0, sizeof(subghz_t));
	g_subghz_state.current_mode = SUBGHZ_MODE_UNDEF;          /* Set current mode to undefined. */
	g_subghz_state.current_state = SUBGHZ_STATE_STARTUP;      /* Current state: startup. */
	g_subghz_state.ramp_time = SUBGHZ_TXPARAMS_RAMPTIME_40US; /* Default ramp time of 40us. */
	g_flag_rx_done = false;
	g_flag_tx_done = false;
	g_flag_timeout = false;

	/* Enable SUBGHZSPI clock. */
	rcc_periph_clock_enable(RCC_SUBGHZ);

	/* Configure NVIC to enable RADIO IRQ. */
	nvic_set_priority(NVIC_RADIO_IRQ, IRQ2NVIC_PRIOR(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY));
	nvic_enable_irq(NVIC_RADIO_IRQ);

	/* Disable SUBGHZ reset through RCC. */
	rcc_subghz_reset_release();

	/* Wait for SUBGHZ to be reset. */
	while (rcc_subghz_running() != 0) ;

	/* Select SUBGHZSPI NSS. */
	pwr_subghzspi_select();

	exti_enable_request(EXTI44);
	pwr_enable_rfbusy_wakeup();

	/* Clear pending flag for EXTI45 (RFBUSY) */
	exti_reset_request(EXTI45);

	/* Initialize SUBGHZ SPI bus. */
	subghz_spi_init(SPI_CR1_BAUDRATE_FPCLK_DIV_32);
	subghz_check_device_ready();
	pwr_subghzspi_unselect();

#if 0
	/* Configure TCXO. */
	subghz_set_tcxo_mode(SUBGHZ_TCXO_TRIM_1V7, 10 << 6);
#endif

	/* PLL calibration. */
	res = subghz_calibrate(SUBGHZ_CALIB_ALL);
	if (SUBGHZ_CMD_SUCCESS(res)) {
		/* Switch to standby mode. */
		subghz_set_standby_mode(SUBGHZ_STDBY_HSE32);

		/* Success. */
		return SUBGHZ_SUCCESS;
	} else {
		/* An error occured during calibration. */
		return SUBGHZ_ERROR;
	}
}

/***********************************************
 * SPI core routines
 **********************************************/

/**
 * @brief Initialize SUBGHZ SPI bus.
 *
 **/

void
subghz_spi_init(int baudrate_prescaler)
{
	/* Disable SUBGHZ peripheral. */
	spi_disable(SUBGHZSPI_BASE);

	/***** Set SUBGHZ CR1 register ******/
	SPI_CR1(SUBGHZSPI_BASE) = (SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM);
	SPI_CR1(SUBGHZSPI_BASE) |= baudrate_prescaler;

	/***** Set SUBGHZ CR2 register ******/
	SPI_CR2(SUBGHZSPI_BASE) = SPI_CR2_FRXTH | SPI_CR2_DS_8BIT;

	/* Re-enable SUBGHZ peripheral. */
	spi_enable(SUBGHZSPI_BASE);
}


/**
 * @brief Start SPI transaction
 *
 * This routine ensures the transceiver is ready to accept
 * an SPI transaction and asserts NSS.
 */

static void
spi_start_transaction(void)
{
	/* Ensure SUBGHZ is ready. */
	subghz_check_device_ready();

	/* Assert NSS. */
	pwr_subghzspi_select();
}


/**
 * @brief End SPI transaction.
 *
 * This routine deasserts NSS and wait until the transceiver
 * is no more busy.
 */

static void
spi_end_transaction(void)
{
	/* Deassert NSS. */
	pwr_subghzspi_unselect();

	/* Wait until SUBGHZ is no more busy. */
	subghz_wait_on_busy();
}

#define SPI_WAIT_LOOP	10000

/* return number of spi data transfered */

int
bulk_spi_transfer(unsigned char *data, int len)
{
	uint16_t v;
	uint32_t spi = SUBGHZSPI_BASE;
	int pos = 0, i = 0;

	if (data == NULL || len <= 0 || spi == 0)
		return 0;

	while (pos < len) {
		/* Wait for previous tx transfer finished. */
		i = 0;
		while ((i < SPI_WAIT_LOOP) && !(SPI_SR(spi) & SPI_SR_TXE))
			i ++;

		if (i == SPI_WAIT_LOOP)
			break;

		SPI_DR8(spi) = data[pos];

		/* wait for rx fifo ready */
		i = 0;
		while ((i < SPI_WAIT_LOOP) && !(SPI_SR(spi) & SPI_SR_RXNE))
			i ++;

		if (i == SPI_WAIT_LOOP)
			break;

		v = SPI_DR8(spi);
		data[pos ++] = v & 0xff;
	}

	return pos;
}

/**
 * @brief  Transmit a byte over SUBGHZSPI.
 * @param  data  Byte to transmit
 * @retval HAL status
 */

static subghz_result_t
spi_transmit(uint8_t data)
{
	spi_send8(SUBGHZSPI_BASE, data);
	return spi_read8(SUBGHZSPI_BASE);
}


/**
 * @brief  Read a byte from SUBGHZSPI.
 * @retval Byte read.
 */

static uint8_t
spi_read_byte(void)
{
	return spi_transmit(0xFF); /* 0x42 */
}

/*******************************************/

/**
 * @brief  Read data registers at an address in the peripheral
 * @param  address register to configurate
 * @param  p_buffer pointer to a data buffer
 * @param  size    amount of data to be sent
 * @retval HAL status
 */

subghz_result_t
subghz_read_regs(uint16_t address, uint8_t *p_buffer, uint16_t size)
{
	subghz_result_t status;
	uint8_t d[4];

	d[0] = SUBGHZ_READ_REGISTER;
	d[1] = ((address & 0xF00) >> 8);
	d[2] = ((address & 0xFF));
	d[3] = 0xFF;

	/* Start transaction. */
	spi_start_transaction();

	bulk_spi_transfer(d, 4);
	bulk_spi_transfer(p_buffer, size);

	/* End transaction. */
	spi_end_transaction();

	status = d[3];
	/* Success. */
	return status;
}

/**
 * @brief  Write data registers at an address in the peripheral
 * @param  address register to configurate
 * @param  p_buffer pointer to a data buffer
 * @param  size    amount of data to be sent
 * @retval HAL status
 */

subghz_result_t
subghz_write_regs(uint16_t address, uint8_t *p_buffer, uint16_t size)
{
	uint8_t d[4];

	d[0] = SUBGHZ_WRITE_REGISTER;
	d[1] = ((address & 0xF00) >> 8);
	d[2] = ((address & 0xFF));

	/* Start SPI transaction. */
	spi_start_transaction();

	bulk_spi_transfer(d, 3);
	bulk_spi_transfer(p_buffer, size);

	/* End of transaction. */
	spi_end_transaction();

	/* Success. */
	return SUBGHZ_STATUS_CMD_SUCCESS;
}


/**
 * @brief  Read data registers at an address in the peripheral
 * @param  address register to configurate
 * @retval HAL status
 */

subghz_result_t
subghz_read_reg(uint16_t address, uint8_t *p_reg)
{
	return subghz_read_regs(address, p_reg, 1);
}


/**
 * @brief  Write data registers at an address in the peripheral
 * @param  address register to configurate
 * @retval HAL status
 */

subghz_result_t
subghz_write_reg(uint16_t address, uint8_t value)
{
	return subghz_write_regs(address, &value, 1);
}


/**
 * @brief Write TX buffer
 *
 * @param   offset  Offset in the TX exchange memory (circular buffer)
 * @param   p_data  Pointer to the data to write
 * @param   length  Number of bytes to write
 * @retval  HAL status
 */

subghz_result_t
subghz_write_buffer(uint8_t offset, uint8_t *p_data, int length)
{
	uint8_t d[4];

	d[0] = SUBGHZ_WRITE_BUFFER;
	d[1] = offset;

	spi_start_transaction();

	bulk_spi_transfer(d, 2);
	bulk_spi_transfer(p_data, length);

	spi_end_transaction();

	/* Success. */
	return SUBGHZ_STATUS_CMD_SUCCESS;
}


/**
 * @brief Read RX buffer
 *
 * @param   offset  Offset in the RX exchange memory (circular buffer)
 * @param   p_data  Pointer to the data to write
 * @param   length  Number of bytes to write
 * @retval  HAL status
 */

subghz_result_t
subghz_read_buffer(uint8_t offset, uint8_t *p_data, int length)
{
	subghz_result_t status = 0;
	uint8_t d[4];

	d[0] = SUBGHZ_READ_BUFFER;
	d[1] = offset;
	d[2] = 0xFF;

	spi_start_transaction();

	bulk_spi_transfer(d, 3);
	bulk_spi_transfer(p_data, length);

	spi_end_transaction();

	status = d[2];
	/* Success. */
	return status;
}


/**
 * @brief Send a command to the SUBGHZ transceiver.
 *
 * @param command       Command to send
 * @param p_parameters  Pointer to an array containing the different parameters to pass to
 *                      the command
 * @param params_size   Size of the parameters array (number of parameters)
 * @retval HAL status
 */

subghz_result_t
subghz_write_command(uint8_t command, uint8_t *p_parameters, int params_size)
{
	uint8_t d[4];

	d[0] = command;

	spi_start_transaction();

	bulk_spi_transfer(d, 1);
	bulk_spi_transfer(p_parameters, params_size);

	spi_end_transaction();

	/* Success. */
	return SUBGHZ_STATUS_CMD_SUCCESS;
}

/**
 * @brief Check SUBGHZ device.
 *
 * @retval 0 on success, 1 otherwise.
 **/

int
subghz_check_device_ready(void)
{
	return subghz_wait_on_busy();
}

/**
 * @brief Wait while device is busy.
 *
 * @retval 0 on success, 1 fail with timeout.
 **/

int
subghz_wait_on_busy(void)
{
	int i = 0;

#define SUBGHZ_BUSY_LOOP 100000

	while ((pwr_is_rfbusys() == 1) && (i < SUBGHZ_BUSY_LOOP)) i++ ;

	return (i == SUBGHZ_BUSY_LOOP);
}

/************************************************
 * SUBGHZ Commands API
 ***********************************************/

/**
 * Retrieve the status of the SubGHz transceiver.
 */

subghz_result_t
subghz_get_status(void)
{
	subghz_result_t result;
	uint8_t params[] = {0x00};

	/* Send command, receive response in params. */
	subghz_write_command(SUBGHZ_GET_STATUS, params, 1);
	result = params[0];

	/* Success. */
	return result;
}


/**
 * @brief Get RX buffer status (payload length and offset)
 *
 * @param   p_rxbuf_status   pointer to a `subghz_rxbuf_status_t` structure that will
 *                           be filled by this function
 * @retval  HAL status
 */

static subghz_result_t
subghz_get_rxbuf_status(subghz_rxbuf_status_t *p_rxbuf_status)
{
	subghz_result_t result;
	uint8_t params[] = {0x00, 0x00, 0x00};

	/* Send command, receive response in params. */
	subghz_write_command(SUBGHZ_GET_RX_BUFFER_STATUS, params, 3);

	/* Copy response into structure. */
	result = params[0];
	p_rxbuf_status->payload_length = params[1];
	p_rxbuf_status->buffer_offset = params[2];

	/* Return status. */
	return result;
}

/**
 * @brief Retrieve the current error byte from transceiver.
 *
 * @param  error pointer to a uint16_t error code.
 * @retval current status
 */

subghz_result_t
subghz_get_error(subghz_err_t *error)
{
	subghz_result_t status = 0;
	uint8_t params[] = {0x00, 0x00, 0x00};

	status = subghz_write_command(SUBGHZ_GET_ERROR, params, 3);
	*error = (params[1] << 8) | params[2];

	/* Success. */
	return status;
}


/**
 * @brief Set Temperature-Compensated Crystal Oscillator (TCXO) parameters
 *
 * @param   trim    trimming value
 * @param   timeout timeout in ms after which the oscillator should be considered unstable
 * @retval  status
 */
subghz_result_t
subghz_set_tcxo_mode(subghz_tcxo_trim_t trim, uint32_t timeout)
{
	subghz_result_t status;
	uint8_t params[] = { trim, (timeout >> 16) & 0xFF,(timeout >> 8) & 0xFF, timeout & 0xFF};

	status = subghz_write_command(SUBGHZ_SET_TCXO_MODE, params, 4);

	/* Success. */
	return status;
}


/**
 * @brief Calibrate oscillators and PLLs
 *
 * @param   calib_cfg  Calibration configuration byte
 * @retval  status
 */

subghz_result_t
subghz_calibrate(uint8_t calib_cfg)
{
	subghz_result_t status = 0;
	uint8_t params[] = {calib_cfg};

	status = subghz_write_command(SUBGHZ_CALIBRATE, params, 1);

	/* Success. */
	return status;
}


/**
 * @brief Set sleep mode
 *
 * @param   sleep_cfg   Sleep mode configuration bits
 * @retval  HAL status
 */

subghz_result_t
subghz_set_sleep(uint8_t sleep_cfg)
{
	subghz_result_t status = 0;
	uint8_t params[] = {sleep_cfg};

	status = subghz_write_command(SUBGHZ_SET_SLEEP, params, 1);
	if (SUBGHZ_CMD_SUCCESS(status)) {
		g_subghz_state.current_state = SUBGHZ_STATE_SLEEP;
	}

	/* Success. */
	return status;
}


/**
 * @brief Put transceiver in standby mode
 *
 * @param mode specify the oscillator to use in standby mode (RC 13MHz or HSE32)
 * @retval HAL status
 */

subghz_result_t
subghz_set_standby_mode(subghz_standby_mode_t mode)
{
	subghz_result_t status = 0;
	uint8_t params[] = {mode};

	status = subghz_write_command(SUBGHZ_SET_STANDBY, params, 1);
	if (SUBGHZ_CMD_SUCCESS(status)) {
		g_subghz_state.current_state = SUBGHZ_STATE_STDBY;
	}

	/* Success. */
	return status;
}


/**
 * @brief Set transceiver regulator mode
 *
 * @param mode Transceiver regulator mode
 */

subghz_result_t
subghz_set_regulator_mode(subghz_regulator_mode_t mode)
{
	subghz_result_t status = 0;
	uint8_t params[] = {mode};

	status = subghz_write_command(SUBGHZ_SET_REGULATOR_MODE, params, 1);

	/* Success. */
	return status;
}


/**
 * @brief Set transceiver in FS mode
 *
 * @retval HAL status
 */

subghz_result_t
subghz_set_fs_mode(void)
{
	subghz_result_t result;

	result = subghz_write_command(SUBGHZ_SET_FS, NULL, 0);
	if (SUBGHZ_CMD_SUCCESS(result)) {
		g_subghz_state.current_state = SUBGHZ_STATE_FS;
	}

	return result;
}


/**
 * @brief Set transceiver in TX mode
 *
 * @param   timeout     TX Timeout
 * @retval  HAL status
 */

subghz_result_t
subghz_set_tx_mode(uint32_t timeout)
{
	subghz_result_t result;

	uint8_t params[] = { (timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF};

	lora_rf_switch_cb(true);

	result = subghz_write_command(SUBGHZ_SET_TX, params, 3);
	if (SUBGHZ_CMD_SUCCESS(result)) {
		/* Update current state. */
		g_subghz_state.current_state = SUBGHZ_STATE_TX;
	}

	return result;
}


/**
 * @brief Set transceiver in RX mode
 *
 * @param   timeout     RX Timeout
 * @retval  HAL status
 */

subghz_result_t
subghz_set_rx_mode(uint32_t timeout)
{
	subghz_result_t result;

	uint8_t params[] = { (timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF};

	/* Request an RF switch (RX mode). */
	lora_rf_switch_cb(false);

	result = subghz_write_command(SUBGHZ_SET_RX, params, 3);
	if (SUBGHZ_CMD_SUCCESS(result)) {
		g_subghz_state.current_state = SUBGHZ_STATE_RX;
	}

	return result;
}


/**
 * @brief Set stop RX timer on preamble
 *
 * @param   config  RX timer stop condition
 * @retval  HAL status
 */

subghz_result_t
subghz_set_stop_rxtimer_on_preamble(subghz_rxtimer_stop_t config)
{
	uint8_t params[] = {config};

	return subghz_write_command(SUBGHZ_SET_STOP_RX_TIMER, params, 1);
}


/**
 * @brief Set RX duty cycle (see section 5.8.3 from RM0453)
 *
 * @param   rx_period    RX period
 * @param   sleep_period Sleep period
 * @retval  HAL status
 */

subghz_result_t
subghz_set_duty_cycle(uint32_t rx_period, uint32_t sleep_period)
{
	uint8_t params[] = {
		(rx_period >> 16) & 0xff,
		(rx_period >> 8) & 0xff,
		(rx_period >> 0) & 0xff,
		(sleep_period >> 16) & 0xff,
		(sleep_period >> 8) & 0xff,
		(sleep_period >> 0) & 0xff,
	};

	return subghz_write_command(SUBGHZ_SET_RX, params, 6);
}


/**
 * @brief Enable Channel Activity Detection (only in LoRa mode)
 *
 * @retval  HAL status
 */

subghz_result_t
subghz_set_cad(void)
{
	return subghz_write_command(SUBGHZ_SET_CAD, NULL, 0);
}


/**
 * @brief Set TX in continuous wave mode
 *
 * @retval  HAL status
 */

subghz_result_t
subghz_set_tx_continuous_wave(void)
{
	return subghz_write_command(SUBGHZ_SET_TX_CONT_WAVE, NULL, 0);
}


/**
 * @brief Set TX in continuous preamble mode
 *
 * @retval  HAL status
 */

subghz_result_t
subghz_set_tx_continuous_preamble(void)
{
	return subghz_write_command(SUBGHZ_SET_TX_CONT_PREAMBLE, NULL, 0);
}


/**
 * @brief Set packet type
 *
 * @param   packet_type   Packet type to set
 * @retval  HAL status
 */

subghz_result_t
subghz_set_packet_type(subghz_packet_type_t packet_type)
{
	uint8_t params[] = {packet_type};
	return subghz_write_command(SUBGHZ_SET_PACKET_TYPE, params, 1);
}


/**
 * @brief Get current packet type
 *
 * @param   p_packet_type   pointer to the current packet type
 * @retval  HAL status
 */

subghz_result_t
subghz_get_packet_type(subghz_packet_type_t *packet_type)
{
	subghz_result_t status;
	uint8_t params[] = {0x00, 0x00};

	subghz_write_command(SUBGHZ_GET_PACKET_TYPE, params, 2);
	status = params[0];
	*packet_type = params[1];

	return status;
}


/**
 * @brief Set RF frequency
 *
 * @param   freq        RF frequency parameter (see section 5.8.4 from RM0453
 *                      for frequency computation)
 * @retval  HAL status
 */

subghz_result_t
subghz_set_rf_freq(uint32_t freq)
{
	uint8_t params[] = {
		(freq >> 24) & 0xff,
		(freq >> 16) & 0xff,
		(freq >> 8) & 0xff,
		(freq >> 0) & 0xff
	};

	return subghz_write_command(SUBGHZ_SET_RF_FREQ, params, 4);
}


/**
 * @brief Set TX parameters
 *
 * @param   power     Set transmit power in DB
 * @param   ramp_time Set PA Ramp-up time
 * @retval  HAL status
 */

subghz_result_t
subghz_set_tx_params(int8_t power, subghz_txparams_ramptime_t ramp_time)
{
	uint8_t params[] = {power, ramp_time};

	return subghz_write_command(SUBGHZ_SET_TX_PARAMS, params, 2);
}


/**
 * @brief Set power amplifier configuration
 *
 * @param    duty_cycle   Set PA duty cycle (Caution, see section 5.8.4)
 * @param    hp_max       Set HP PA maximum value
 * @param    pa_sel       HP/LP selection (default, HP:1)
 */

subghz_result_t
subghz_set_pa_config(uint8_t duty_cycle, uint8_t hp_max, uint8_t pa_sel)
{
	uint8_t params[] = {duty_cycle, hp_max, pa_sel, 0x01};

	return subghz_write_command(SUBGHZ_SET_PA_CONFIG, params, 4);
}


/**
 * @brief   Set TX/RX fallback mode
 *
 * @param   mode   Mode to enter when a packet is succesfully sent/received
 * @retval  HAL status
 */

subghz_result_t
subghz_set_tx_rx_fallback_mode(subghz_fallback_mode_t mode)
{
	uint8_t params[] = {mode};

	return subghz_write_command(SUBGHZ_SET_TXRX_FALLBACK, params, 1);
}


/**
 * @brief   Set CAD parameters
 *
 * @param   nb_symbols    Number of CAD symbols
 * @param   det_peak      Detection peak value
 * @param   det_min       Detection min value
 * @param   exit_mode     Exit mode
 * @param   timeout       CAD timeout
 * @retval  HAL status
 */

subghz_result_t
subghz_set_cad_params(subghz_cad_symbols_t nb_symbols, uint8_t det_peak,
                      uint8_t det_min, subghz_cad_exit_mode_t exit_mode,
                      uint32_t timeout)
{
	uint8_t params[] = {
		nb_symbols,
		det_peak,
		det_min,
		exit_mode,
		(timeout >> 16) & 0xFF,
		(timeout >> 8) & 0xFF,
		timeout & 0xFF
	};

	return subghz_write_command(SUBGHZ_SET_CAD_PARAMS, params, 7);
}


/**
 * @brief Set buffer base address for RX and TX packets
 *
 * @param   tx_base_addr    TX base address offset relative to SUBGHZ RAM base address
 * @param   rx_base_addr    RX base address offset relative to SUBGHZ RAM base address
 * @retval  HAL status
 */

subghz_result_t
subghz_set_buffer_base_address(uint8_t tx_base_addr, uint8_t rx_base_addr)
{
	uint8_t params[] = {tx_base_addr, rx_base_addr};

	return subghz_write_command(SUBGHZ_SET_BUFFER_BA, params, 2);
}


/**
 * @brief Set FSK modulation parameters.
 *
 * @param   bitrate     Modulation bitrate (bit/s)
 * @param   gaussian    Pulse shape (gaussian filter)
 * @param   bandwidth   RF bandwidth
 * @param   deviation   Frequency deviation
 * @retval  HAL status
 */

subghz_result_t
subghz_set_fsk_modulation_params(uint32_t bitrate, subghz_fsk_gaussian_t gaussian,
	                         subghz_fsk_bandwidth_t bandwidth, uint32_t deviation)
{
	uint32_t channel;
	uint32_t br = ( uint32_t )(( 32 * XTAL_FREQ ) / bitrate);
	SX_FREQ_TO_CHANNEL(channel, deviation);

	uint8_t params[] = {
		(br >> 16) & 0xff,
		(br >> 8) & 0xff,
		(br >> 0) & 0xff,
		gaussian,
		bandwidth,
		(channel >> 16) & 0xff,
		(channel >> 8) & 0xff,
		(channel >> 0) & 0xff,
	};

	return subghz_write_command(SUBGHZ_SET_MOD_PARAMS, params, 8);
}


/**
 * @brief Set LoRa modulation parameters.
 *
 * @param   sf          Spreading factor
 * @param   bandwidth   RF bandwidth
 * @param   cr          Coding rate
 * @param   ldro        Low data rate optimization
 * @retval  HAL status
 */

subghz_result_t
subghz_set_lora_modulation_params(subghz_lora_sf_t sf, subghz_lora_bandwidth_t bandwidth,
                                  subghz_lora_cr_t cr, subghz_lora_ldro_t ldro)
{
	uint8_t params[] = {sf, bandwidth, cr, ldro};

	return subghz_write_command(SUBGHZ_SET_MOD_PARAMS, params, 4);
}


/**
 * @brief Set BSPK modulation parameters
 *
 * @param   bitrate Bitrate
 * @param   gaussian Gaussian filter setting
 * @retval  HAL status
 */

subghz_result_t
subghz_set_bpsk_modulation_params(uint32_t bitrate, subghz_bpsk_gaussian_t gaussian)
{
	uint8_t params[] = {
		(bitrate >> 16) & 0xff,
		(bitrate >> 8) & 0xff,
		(bitrate >> 0) & 0xff,
		gaussian
	};

	return subghz_write_command(SUBGHZ_SET_MOD_PARAMS, params, 4);
}


/**
 * @brief Set generic packet parameters
 *
 * @param   preamble_length     Size of preamble, in number of symbols
 * @param   det_length          Number of bit symbols used to detect preamble
 * @param   syncword_length     Size of synchronization word, in number of bit symbols
 * @param   addr_comp           Address comparison method
 * @param   packet_length       Type of packet (variable or fixed length)
 * @param   payload_length      Size of payload in bytes
 * @param   crc_type            CRC Size and status
 * @param   whitening           Enable/disable whitening
 */

subghz_result_t
subghz_set_packet_params(uint16_t preamble_length, subghz_det_length_t det_length,
                         uint8_t syncword_length, subghz_addr_comp_t addr_comp,
                         subghz_packet_length_t packet_length, uint8_t payload_length,
                         subghz_packet_crc_t crc_type, bool whitening)
{
	uint8_t params[] = {
		(preamble_length >> 8) & 0xff,
		(preamble_length >> 0) & 0xff,
		det_length,
		syncword_length,
		addr_comp,
		packet_length,
		payload_length,
		crc_type,
		whitening ? 1 : 0
	};

	return subghz_write_command(SUBGHZ_SET_PACKET_PARAMS, params, 9);
}


/**
 * @brief Set LoRa packet parameters
 *
 * @param   preamble_length     Preamble length in number of symbols
 * @param   header_type         variable/fixed length payload
 * @param   payload_length      Length of payload in bytes
 * @param   crc_enabled         CRC enabled if true, disabled otherwise
 * @param   invert_iq           standard IQ setup if false, inverted IQ if true
 * @retval  HAL status
 */

subghz_result_t
subghz_set_lora_packet_params(uint16_t preamble_length, subghz_lora_packet_hdr_t header_type,
                              uint8_t payload_length, bool crc_enabled, bool invert_iq)
{
	uint8_t params[] = {
		(preamble_length >> 8) & 0xff,
		(preamble_length >> 0) & 0xff,
		header_type,
		payload_length,
		crc_enabled ? 1 : 0,
		invert_iq ? 1 : 0
	};

	return subghz_write_command(SUBGHZ_SET_PACKET_PARAMS, params, 6);
}


/**
 * @brief Set BPSK packet parameters (payload length)
 *
 * @param   payload_length maximum length for BSPK payload
 * @retval  HAL status
 */

subghz_result_t
subghz_set_bpsk_packet_params(uint8_t payload_length)
{
	uint8_t params[] = {payload_length};

	return subghz_write_command(SUBGHZ_SET_PACKET_PARAMS, params, 1);
}


/**
 * @brief Set LoRa symbol timeout
 *
 * Set number of LoRa symbols to be received before starting the reception of a
 * LoRa packet
 *
 * @param   symb_num  Number of LoRa symbols (0-255)
 * @retval  HAL status
 */

subghz_result_t
subghz_set_lora_symb_timeout(uint8_t symb_num)
{
	uint8_t params[] = {symb_num};

	return subghz_write_command(SUBGHZ_SET_LORA_SYMB_TO, params, 1);
}


/**
 * @brief Get RX buffer status (payload length and offset)
 *
 * @param   p_rxbuf_status   pointer to a `subghz_rxbuf_status_t` structure that will
 *                              be filled by this function
 * @retval  HAL status
 */

subghz_result_t
subghz_get_fsk_packet_status(subghz_fsk_packet_status_t *p_packet_status)
{
	subghz_result_t result;
	uint8_t params[] = {0x00, 0x00, 0x00, 0x00};

	/* Send command, receive response in params. */
	subghz_write_command(SUBGHZ_GET_PACKET_STATUS, params, 4);

	/* Copy response into structure. */
	result = params[0];
	p_packet_status->rx_status = params[1];
	p_packet_status->rssi_sync = params[2];
	p_packet_status->rssi_avg = params[3];

	/* Return status. */
	return result;
}


/**
 * @brief Get LoRa packet status.
 *
 * @param   p_packet_status  pointer to a `subghz_lora_packet_status_t` structure that will be
 *                           filled by this function.
 * @retval  HAL status
 */

subghz_result_t
subghz_get_lora_packet_status(subghz_lora_packet_status_t *p_packet_status)
{
	subghz_result_t result;
	uint8_t params[] = {0x00, 0x00, 0x00, 0x00};

	/* Send command, receive response in params. */
	subghz_write_command(SUBGHZ_GET_PACKET_STATUS, params, 4);

	/* Copy response into structure. */
	result = params[0];
	p_packet_status->rssi = params[1];
	p_packet_status->snr = params[2];
	p_packet_status->signal_rssi = params[3];

	/* Return status. */
	return result;
}


/**
 * @brief Get instantaneous RSSI
 *
 * @param   p_rssi_inst pointer to a `uint8_t` that will receive the instantaneous RSSI value
 * @retval  HAL status
 */

subghz_result_t
subghz_get_rssi_inst(uint8_t *p_rssi_inst)
{
	subghz_result_t result;
	uint8_t params[] = {0x00, 0x00};

	/* Send command, receive response in params. */
	subghz_write_command(SUBGHZ_GET_RSSI_INST, params, 2);

	result = params[0];
	*p_rssi_inst = params[1];

	/* Return status. */
	return result;
}


/**
 * @brief Get FSK status
 *
 * @param   p_fsk_stats pointer to a `subghz_fsk_stats_t` struct that will be filled
 *                       by this function.
 * @retval  HAL status
 */

subghz_result_t
subghz_get_fsk_stats(subghz_fsk_stats_t *p_fsk_stats)
{
	subghz_result_t result;
	uint8_t params[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	/* Send command, receive response in params. */
	subghz_write_command(SUBGHZ_GET_STATS, params, 7);

	/* Extract result and data. */
	result = params[0];
	p_fsk_stats->nb_packets_recvd = (params[1] << 8) | params[2];
	p_fsk_stats->nb_packets_crcerr = (params[3] << 8) | params[4];
	p_fsk_stats->nb_packets_lenerr = (params[5] << 8) | params[6];

	/* Return status. */
	return result;
}


/**
 * @brief Get LoRa status
 *
 * @param   p_lora_stats pointer to a `subghz_lora_stats_t` struct that will be filled
 *                       by this function.
 * @retval  HAL status
 */

subghz_result_t
subghz_get_lora_stats(subghz_lora_stats_t *p_lora_stats)
{
	subghz_result_t result;
	uint8_t params[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	/* Send command, receive response in params. */
	subghz_write_command(SUBGHZ_GET_STATS, params, 7);

	/* Extract result and data. */
	result = params[0];
	p_lora_stats->nb_packets_recvd = (params[1] << 8) | params[2];
	p_lora_stats->nb_packets_crcerr = (params[3] << 8) | params[4];
	p_lora_stats->nb_packets_headerr = (params[5] << 8) | params[6];

	/* Return status. */
	return result;
}


/**
 * @brief Reset stats counters for FSK & LoRa
 *
 * @retval  HAL status
 */

subghz_result_t
subghz_reset_stats(void)
{
	uint8_t params[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	/* Send command. */
	return subghz_write_command(SUBGHZ_RESET_STATS, params, 6);
}


/**
 * @brief Configure IRQ sources
 *
 * @param   irq_mask  Global IRQ interrupt mask
 * @param   irq1_mask IRQ1 interrupt mask
 * @param   irq2_mask IRQ2 interrupt mask
 * @param   irq3_mask IRQ3 interrupt mask
 * @retval  HAL status
 */

subghz_result_t
subghz_config_dio_irq(uint16_t irq_mask, uint16_t irq1_mask,
                      uint16_t irq2_mask, uint16_t irq3_mask)
{
	subghz_result_t res;
	uint8_t params[] = {
		(irq_mask >> 8) & 0xff,
		(irq_mask >> 0) & 0xff,
		(irq1_mask >> 8) & 0xff,
		(irq1_mask >> 0) & 0xff,
		(irq2_mask >> 8) & 0xff,
		(irq2_mask >> 0) & 0xff,
		(irq3_mask >> 8) & 0xff,
		(irq3_mask >> 0) & 0xff
	};

	/* Send command. */
	res = subghz_write_command(SUBGHZ_CFG_DIO_IRQ, params, 8);
	print_reg_hex("dio_irq_res", res);

	/* Always success. */
	return (0x06 << 1);
}


/**
 * @brief Get IRQ status
 *
 * @param   p_irq_status  Pointer to a `subghz_irq_status_t` structure that
 *                        will be filled by this function.
 * @retval  HAL status
 */

static subghz_result_t
subghz_get_irq_status(subghz_irq_status_t *p_irq_status)
{
	subghz_result_t result;
	uint8_t params[] = {0x00, 0x00, 0x00};

	/* Send command. */
	subghz_write_command(SUBGHZ_GET_IRQ_STATUS, params, 3);

	result = params[0];
	p_irq_status->word = (params[1] << 8) | params[2];

	/* Return result. */
	return result;
}


/**
 * @brief Clear IRQ
 *
 * @param   p_irq_status  Pointer to a `subghz_irq_status_t` structure that
 *                        will be filled by this function.
 * @retval  HAL status
 */

static subghz_result_t
subghz_clear_irq_status(subghz_irq_status_t *p_irq_status)
{
	subghz_result_t result;
	uint8_t params[] = {
	    (p_irq_status->word >> 8) & 0xff,
	    (p_irq_status->word >> 0) & 0xff,
	};

	/* Send command. */
	return subghz_write_command(SUBGHZ_CLR_IRQ_STATUS, params, 3);

	/* Return result. */
	return result;
}


/**
 * @brief Set TX payload
 *
 * @param   payload   Pointer to the payload to send
 * @param   size      Payload size
 * @retval  HAL status
 */

subghz_result_t
subghz_set_payload(uint8_t *payload, uint8_t size)
{
	/* Use offset 0 by default. */
	return subghz_write_buffer(0x00, payload, size);
}

/*********************************************
 * SubGHZ Driver API
 ********************************************/

/**
 * @brief   Enable LoRa mode
 * @param   p_lora_config pointer to a `subghz_lora_config_t` structure.
 * @return  SUBGHZ_SUCCESS on success, SUBGHZ_ERROR on error.
 **/

int
subghz_lora_mode(subghz_lora_config_t *p_lora_config)
{
	uint32_t channel;

	/* Switch to LoRa mode. */
	g_subghz_state.current_mode = SUBGHZ_MODE_LORA;

	/* Backup our LoRa parameters. */
	memcpy(&g_subghz_state.lora_params, p_lora_config, sizeof(subghz_lora_config_t));

	/* Set packet type to LoRa. */
	if (SUBGHZ_CMD_FAILED(subghz_set_packet_type(SUBGHZ_PACKET_LORA))) {
		return SUBGHZ_ERROR;
	}

	/* Configure LoRa packet parameters. */
	if (SUBGHZ_CMD_FAILED(subghz_set_lora_packet_params(p_lora_config->preamble_length, p_lora_config->header_type,
	                                                    p_lora_config->payload_length, p_lora_config->crc_enabled,
	                                                    p_lora_config->invert_iq)))
	{
		return SUBGHZ_ERROR;
	}

	/* Set RF frequency. */
	SX_FREQ_TO_CHANNEL(channel, p_lora_config->freq);
	if (SUBGHZ_CMD_FAILED(subghz_set_rf_freq(channel))) {
		return SUBGHZ_ERROR;
	}

	/* Save freq and derivated channel values. */
	g_subghz_state.current_freq = p_lora_config->freq;
	g_subghz_state.current_channel = channel;

	mini_printf("Lora FREQ -> %d HZ, Channel #%d\n", g_subghz_state.current_freq, g_subghz_state.current_channel);

	if (subghz_config_pa(p_lora_config->pa_mode, p_lora_config->pa_power) == SUBGHZ_ERROR) {
		return SUBGHZ_ERROR;
	}

	/* Configure transceiver in LoRa mode. */
	if (SUBGHZ_CMD_FAILED(subghz_set_lora_modulation_params(p_lora_config->sf, p_lora_config->bw,
	                                                        p_lora_config->cr, p_lora_config->ldro)))
	{
		return SUBGHZ_ERROR;
	}

	return SUBGHZ_SUCCESS;
}


/**
 * @brief   Enable FSK mode
 * @param   p_fsk_config pointer to a `subghz_fsk_config_t` structure
 * @return  SUBGHZ_SUCCESS on success, SUBGHZ_ERROR on error.
 */

int
subghz_fsk_mode(subghz_fsk_config_t *p_fsk_config)
{
	int i;
	uint16_t syncword_addr;
	uint32_t channel;

	/* Switch to FSK mode. */
	g_subghz_state.current_mode = SUBGHZ_MODE_FSK;

	/* Backup our FSK parameters. */
	memcpy(&g_subghz_state.fsk_params, p_fsk_config, sizeof(subghz_fsk_config_t));

	/* Set packet type to FSK. */
	if (SUBGHZ_CMD_FAILED(subghz_set_packet_type(SUBGHZ_PACKET_FSK)))
	{
		mini_printf("Set packet type failed\n");
		return SUBGHZ_ERROR;
	}

	/* Configure FSK packet parameters. */
	if (SUBGHZ_CMD_FAILED(subghz_set_packet_params(p_fsk_config->preamble_length, p_fsk_config->preamble_detect,
	                                               p_fsk_config->sync_word_length*8, p_fsk_config->addr_comp,
	                                               p_fsk_config->packet_type, p_fsk_config->payload_length,
	                                               p_fsk_config->crc, p_fsk_config->whitening)))
	{
		mini_printf("Set packet params failed\n");
		return SUBGHZ_ERROR;
	}

	/* Set Synchronization Word (SYNC_WORD). */
	if (p_fsk_config->sync_word_length > 0) {
		syncword_addr = SUBGHZ_GSYNCR0;
		for (i = 0; i < p_fsk_config->sync_word_length; i ++) {
			subghz_write_reg(syncword_addr ++, p_fsk_config->sync_word[i]);
		}
	}

	/* Set RF frequency. */
	SX_FREQ_TO_CHANNEL(channel, p_fsk_config->freq);
	if (SUBGHZ_CMD_FAILED(subghz_set_rf_freq(channel))) {
		mini_printf("Set RF freq failed\n");
		return SUBGHZ_ERROR;
	}

	/* Save freq and derivated channel values. */
	g_subghz_state.current_freq = p_fsk_config->freq;
	g_subghz_state.current_channel = channel;

	if (subghz_config_pa(p_fsk_config->pa_mode, p_fsk_config->pa_power) == SUBGHZ_ERROR) {
		mini_printf("Set PA config failed\n");
		return SUBGHZ_ERROR;
	}

	/* Configure transceiver in LoRa mode. */
	if (SUBGHZ_CMD_FAILED(subghz_set_fsk_modulation_params(p_fsk_config->bit_rate, p_fsk_config->pulse_shape,
	                                                       p_fsk_config->bandwidth, p_fsk_config->freq_dev)))
	{
		mini_printf("Set FSK mod params failed\n");
		return SUBGHZ_ERROR;
	}

	return SUBGHZ_SUCCESS;
}


/**
 * @brief   Configure the STM32WLxx Power Amplifier.
 * @param   mode  Power amplifier mode (Low Power / High Power)
 * @param   power Requested output power
 * @return  SUBGHZ_ERROR on error, SUBGHZ_SUCCESS otherwise.
 */

int
subghz_config_pa(subghz_pa_mode_t mode, subghz_pa_pwr_t power)
{
	struct pa_params_cfg
	{
	  uint8_t duty_cycle;
	  uint8_t hp_max;
	  uint8_t pa_sel;
	  uint8_t tx_pwr;
	};

	struct pa_params_cfg selected_config;

	static const struct pa_params_cfg pa_params_map[SUBGHZ_PA_MODE_COUNT][SUBGHZ_PA_PWR_COUNT] = {
	    [SUBGHZ_PA_MODE_LP] = {
	        [SUBGHZ_PA_PWR_10DBM] = {.duty_cycle = 0x01, .hp_max = 0x00, .pa_sel = 0x01, .tx_pwr = 0x0D},
	        [SUBGHZ_PA_PWR_14DBM] = {.duty_cycle = 0x04, .hp_max = 0x00, .pa_sel = 0x01, .tx_pwr = 0x0E},
	        [SUBGHZ_PA_PWR_15DBM] = {.duty_cycle = 0x07, .hp_max = 0x00, .pa_sel = 0x01, .tx_pwr = 0x0E},
	    },
	    [SUBGHZ_PA_MODE_HP] = {
	        [SUBGHZ_PA_PWR_14DBM] = {.duty_cycle = 0x02, .hp_max = 0x02, .pa_sel = 0x00, .tx_pwr = 0x16},
	        [SUBGHZ_PA_PWR_17DBM] = {.duty_cycle = 0x02, .hp_max = 0x03, .pa_sel = 0x00, .tx_pwr = 0x16},
	        [SUBGHZ_PA_PWR_20DBM] = {.duty_cycle = 0x03, .hp_max = 0x05, .pa_sel = 0x00, .tx_pwr = 0x16},
	        [SUBGHZ_PA_PWR_22DBM] = {.duty_cycle = 0x04, .hp_max = 0x07, .pa_sel = 0x00, .tx_pwr = 0x16},
	    },
	};

	/* Compute current max. */
	uint8_t current_max = (mode == SUBGHZ_PA_MODE_LP) ? SUBGHZ_DEFAULT_CURMAX_LP : SUBGHZ_DEFAULT_CURMAX_HP;

	/* Get selected PA parameters and apply them. */
	selected_config = pa_params_map[mode][power];
	if (selected_config.duty_cycle > 0) {
		/* Configure PA. */
		subghz_set_pa_config(selected_config.duty_cycle, selected_config.hp_max, selected_config.pa_sel);

		/* Set current max for the whole SoC. */
		if (SUBGHZ_CMD_FAILED(subghz_write_reg(SUBGHZ_PAOCPR, current_max))) {
			return SUBGHZ_ERROR;
		}

		/* Set TX parameters. */
		if (SUBGHZ_CMD_FAILED(subghz_set_tx_params(selected_config.tx_pwr, g_subghz_state.ramp_time))) {
			return SUBGHZ_ERROR;
		}

		/* Success. */
		return SUBGHZ_SUCCESS;
	} else {
		/* Error. */
		return SUBGHZ_ERROR;
	}
}


/**
 * @brief   Write frame into TX buffer and send it.
 * @param   p_frame pointer to a frame (byte array) to send
 * @param   length frame length in bytes
 * @param   timeout Transmission timeout
 * @return  SUBGHZ_SUCCESS if frame has been correctly sent, SUBGHZ_ERROR otherwise.
 */
int
subghz_send_async(uint8_t *p_frame, int length, uint32_t timeout)
{
	/* Configure IRQ flags. */
	if (SUBGHZ_CMD_FAILED(subghz_config_dio_irq(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
	                                            IRQ_RADIO_NONE, IRQ_RADIO_NONE)))
	{
		return SUBGHZ_ERROR;
	}

	/* Set payload buffer. */
	if (SUBGHZ_CMD_FAILED(subghz_set_payload(p_frame, length))) {
		return SUBGHZ_ERROR;
	}

	/* Start transmission. */
	if (SUBGHZ_CMD_FAILED(subghz_set_tx_mode(timeout))) {
		return SUBGHZ_ERROR;
	}

	/* Success. */
	return SUBGHZ_SUCCESS;
}


/**
 * @brief   Write frame into TX buffer and wait for transmission (synchronous)
 * @param   p_frame pointer to a frame (byte array) to send
 * @param   length frame length in bytes
 */
int
subghz_send(uint8_t *p_frame, int length, uint32_t timeout)
{
	/* Configure IRQ flags (no IRQ) */
	if (SUBGHZ_CMD_FAILED(subghz_config_dio_irq(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
	                                            IRQ_RADIO_NONE, IRQ_RADIO_NONE)))
	{
		return SUBGHZ_ERROR;
	}

	/* Set payload buffer. */
	if (SUBGHZ_CMD_FAILED(subghz_set_payload(p_frame, length))) {
		return SUBGHZ_ERROR;
	}

	/* Reset TX and Timeout flags. */
	g_flag_tx_done = false;
	g_flag_timeout = false;

	/* Start transmission. */
	if (SUBGHZ_CMD_FAILED(subghz_set_tx_mode(timeout))) {
		return SUBGHZ_ERROR;
	}

	/* Wait for timeout or packet sent. */
	while ((!g_flag_tx_done) && (!g_flag_timeout)) ;

	/* Return status. */
	return (g_flag_timeout) ? SUBGHZ_TIMEOUT : SUBGHZ_SUCCESS;
}


/**
 * @brief   Enable RX mode and await a packet
 * @param   timeout Reception timeout
 * @return  SUBGHZ_SUCCESS if RX mode has been correctly set, SUBGHZ_ERROR otherwise.
 */

int
subghz_receive_async(uint32_t timeout)
{
	/* Configure IRQ flags (no IRQ) */
	if (SUBGHZ_CMD_FAILED(subghz_config_dio_irq(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
	                                            IRQ_RADIO_NONE, IRQ_RADIO_NONE)))
	{
		return SUBGHZ_ERROR;
	}

	/* Enable reception. */
	if (SUBGHZ_CMD_FAILED(subghz_set_rx_mode(timeout))) {
		return SUBGHZ_ERROR;
	}

	/* Success. */
	return SUBGHZ_SUCCESS;
}


/**
 * @brief   Enable RX mode and await a packet (synchronous)
 * @param   p_frame   Pointer to a RX buffer
 * @param   p_length  Pointer to an uint8_t specifying the size of the provided RX buffer
 *                    and updated to return the frame size
 * @param   timeout   Reception timeout
 * @return  SUBGHZ_SUCCESS if RX mode has been correctly set, SUBGHZ_ERROR otherwise.
 */

int
subghz_receive(uint8_t *p_frame, uint8_t *p_length, uint32_t timeout)
{
	uint8_t frame_length;
	subghz_rxbuf_status_t rxbuf_status;

	/* p_length and p_frame must not be NULL. */
	if ((p_length == NULL) || (p_frame == NULL))
	  return SUBGHZ_ERROR;

	/* Configure IRQ flags (no IRQ) */
	if (SUBGHZ_CMD_FAILED(subghz_config_dio_irq(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
	                                            IRQ_RADIO_NONE, IRQ_RADIO_NONE)))
		return SUBGHZ_ERROR;

	/* Enable reception. */
	if (SUBGHZ_CMD_FAILED(subghz_set_rx_mode(timeout)))
		return SUBGHZ_ERROR;


	/* Wait for packet. */
	while ((!g_flag_rx_done) && (!g_flag_timeout)) ;

	/* If timeout, return SUBGHZ_TIMEOUT. */
	if (g_flag_timeout)
		return SUBGHZ_TIMEOUT;

	/* Retrieve packet information. */
	if (SUBGHZ_CMD_SUCCESS(subghz_get_rxbuf_status(&rxbuf_status))) {
		/* Process received packet. */
		frame_length = *p_length;
		*p_length = rxbuf_status.payload_length;
		if (rxbuf_status.payload_length <= (frame_length)) {
			/* Read packet and store into */
			if (SUBGHZ_CMD_SUCCESS(subghz_read_buffer(rxbuf_status.buffer_offset, p_frame, *p_length)))
				return SUBGHZ_SUCCESS;
			else {
				mini_printf("Cannot read buffer\r\n");
			}
		}
	} else {
		mini_printf("Cannot read rxbuf status\r\n");
	}

	/* An error occured. */
	return SUBGHZ_ERROR;
}

/*********************************************
 * RADIO IRQ Handling
 ********************************************/

void
radio_isr(void)
{
	subghz_irq_status_t irq_status;
	subghz_irq_status_t irq_clr;
	subghz_rxbuf_status_t rxbuf_status;
	uint8_t data[256];

	/* Initialize our IRQ clear value. */
	irq_clr.word = 0;

	/* Get IRQ status */
	subghz_get_irq_status(&irq_status);

	/* Handle packet transmission. */
	if (irq_status.bits.tx_done) {
		/* Save flag for synchronous mode. */
		g_flag_tx_done = true;

		/* Clear TX_DONE bit. */
		irq_clr.bits.tx_done = 1;
		subghz_clear_irq_status(&irq_clr);
	}

	/* Handle packet reception. */
	if (irq_status.bits.rx_done) {
		/* Save flag for synchronous mode. */
		g_flag_rx_done = true;

		if (SUBGHZ_CMD_SUCCESS(subghz_get_rxbuf_status(&rxbuf_status))) {
			/* Display received packet. */
			gpio_set(GPIOB, GPIO4);
			if (rxbuf_status.payload_length < 254) {
				if (SUBGHZ_CMD_SUCCESS(subghz_read_buffer(rxbuf_status.buffer_offset, data + 1, rxbuf_status.payload_length))) {
					data[0] = rxbuf_status.payload_length;
					if (lora_rx_queue)
						xQueueSendToBackFromISR(lora_rx_queue, data, &woken);
				}
			}
			gpio_clear(GPIOB, GPIO4);
		}

		/* Clear RX_DONE bit. */
		irq_clr.bits.rx_done = 1;
	}

	/* Handle timeout. */
	if (irq_status.bits.timeout) {
		/* Save flag for synchronous mode. */
		g_flag_timeout = true;

		/* Clear TIMEOUT bit. */
		irq_clr.bits.timeout = 1;
	}

	/* Clear IRQ. */
	subghz_clear_irq_status(&irq_clr);
}


/**
 * @brief   Set synchronization word (0 - 64 bits)
 * @param   p_syncword    pointer to a buffer containing the syncword value
 * @param   length        syncword size in bytes
 * @retval  SUBGHZ_SUCCESS on success, SUBGHZ_ERROR otherwise
 */

int
subghz_set_syncword(uint8_t *p_syncword, int length)
{
	int i;
	uint16_t syncword_addr;

	/* LoRa mode: synchronization word must be 2 bytes. */
	if (g_subghz_state.current_mode == SUBGHZ_MODE_LORA) {
		if (length == 2) {
			subghz_write_reg(SUBGHZ_LSYNCRL, p_syncword[0]);
			subghz_write_reg(SUBGHZ_LSYNCRH, p_syncword[1]);
		} else {
			/* Error, length does not match. */
			return SUBGHZ_ERROR;
		}
	} else {
		/* Set Synchronization Word (SYNC_WORD). */
		if (length > 0) {
			syncword_addr = SUBGHZ_GSYNCR0;
			for (i = 0; i < length; i ++) {
				subghz_write_reg(syncword_addr ++, p_syncword[i]);
			}
		}
	}

	/* Success. */
	return SUBGHZ_SUCCESS;
}


/**
 * @brief   Set 16-bit sync word
 * @param   syncword  16-bit syncword
 * @retval  SUBGHZ_SUCCESS on success, SUBGHZ_ERROR otherwise
 */

int
subghz_set_syncword16(uint16_t syncword)
{
	/* Forward to our generic syncword setter. */
	return subghz_set_syncword((uint8_t *)&syncword, 2);
}

/**
 * @brief   Set 32-bit sync word
 * @param   syncword  32-bit syncword
 * @retval  SUBGHZ_SUCCESS on success, SUBGHZ_ERROR otherwise
 */

int
subghz_set_syncword32(uint32_t syncword)
{
	/* Forward to our generic syncword setter. */
	return subghz_set_syncword((uint8_t *)&syncword, 4);
}


/**
 * @brief   Set 64-bit sync word
 * @param   syncword_l  Bits 0-31 of sync word
 * @param   syncword_h  Bits 32-63 of sync word
 * @retval  SUBGHZ_SUCCESS on success, SUBGHZ_ERROR otherwise
 */

int
subghz_set_syncword64(uint32_t syncword_l, uint32_t syncword_h)
{
	uint32_t syncword[2];

	/* Fill syncword buffer. */
	syncword[0] = syncword_l;
	syncword[1] = syncword_h;

	/* Forward to our generic syncword setter. */
	return subghz_set_syncword((uint8_t *)&syncword, 8);
}


/**
 * @brief   Enable or disable synchronization detection.
 * @param   enable  True to enable sync word detection, false to disable.
 * @retval  Always SUBGHGZ_SUCCESS.
 */

int
subghz_enable_syncword(bool enable)
{
	uint8_t reg;

	/* First, read register value. */
	subghz_read_reg(SUBGHZ_GPKTCTL1AR, &reg);

	if (enable)
		reg |= SUBGHZ_GPKTCTL1AR_SYNCDETEN_MSK;
	else
		reg &= ~(SUBGHZ_GPKTCTL1AR_SYNCDETEN_MSK);

	/* Write register back. */
	subghz_write_reg(SUBGHZ_GPKTCTL1AR, reg);

	/* Success. */
	return SUBGHZ_SUCCESS;
}


/**
 * @brief   Set CRC initial value (16 bits)
 * @param   crc_init  CRC initial value
 * @retval  Always SUBGHZ_SUCCESS
 */

int
subghz_set_crc_init(uint16_t crc_init)
{
	/* Write CRC initial value in the correct registers. */
	subghz_write_reg(SUBGHZ_GCRCINIRL, crc_init & 0x00ff );
	subghz_write_reg(SUBGHZ_GCRCINIRH, (crc_init >> 8) & 0x00ff);

	/* Success. */
	return SUBGHZ_SUCCESS;
}


/**
 * @brief   Set CRC polynomial
 * @param   crc_poly  CRC polynomial
 * @retval  Always SUBGHZ_SUCCESS
 */

int
subghz_set_crc_poly(uint16_t crc_poly)
{
	/* Write CRC initial value in the correct registers. */
	subghz_write_reg(SUBGHZ_GCRCPOLRL, crc_poly & 0x00ff );
	subghz_write_reg(SUBGHZ_GCRCPOLRH, (crc_poly >> 8) & 0x00ff);

	/* Success. */
	return SUBGHZ_SUCCESS;
}


/**
 * @brief   Set whitening initial value (8 bits)
 * @param   white_init  Whitening initial value
 * @retval  Always SUBGHZ_SUCCESS.
 */

int
subghz_set_whitening_init(uint8_t white_init)
{
	/* Write whitening initial value (8 bits) */
	subghz_write_reg(SUBGHZ_GWHITEINIRL, white_init);

	/* Success. */
	return SUBGHZ_SUCCESS;
}

int lora_started = 0;
int init_lora(void)
{
	subghz_lora_config_t lora_config = {
		.sf = SUBGHZ_LORA_SF7,
		.bw = SUBGHZ_LORA_BW250,
		.cr = SUBGHZ_LORA_CR_48,
		.freq = 865200000,
		.payload_length = 64,
		.preamble_length = 12,
		.header_type = SUBGHZ_PKT_FIXED_LENGTH,
		.crc_enabled = false,
		.invert_iq = false,
		.ldro = SUBGHZ_LORA_LDRO_DISABLED,
		.pa_mode = SUBGHZ_PA_MODE_HP,
		.pa_power = SUBGHZ_PA_PWR_14DBM
	};

#if 0
	subghz_fsk_config_t fsk_config = {
		.freq = 865200000,
		.freq_dev = 50000,
		.bandwidth = SUBGHZ_FSK_BW373,
		.pulse_shape = SUBGHZ_FSK_GAUSSIAN_NONE,
		.bit_rate = 50000,
		.preamble_length = 8,
		.packet_type = SUBGHZ_PKT_FIXED_LENGTH,
		.sync_word_length = 0,
		.payload_length = 13,
		.addr_comp = SUBGHZ_ADDR_COMP_DISABLED,
		.crc = SUBGHZ_PKT_CRC_NONE,
		.whitening = false,
		.pa_mode = SUBGHZ_PA_MODE_HP,
		.pa_power = SUBGHZ_PA_PWR_22DBM
	};

	fsk_config.sync_word[0] = 0xBA;
	fsk_config.sync_word[1] = 0xDC;
	fsk_config.sync_word[2] = 0x0F;
	fsk_config.sync_word[3] = 0xFE;
#endif

	/* PA6 = RF_TXEN, PA7 = RF_RXEN */
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL_RXEN_PIN | RF_SW_CTRL_TXEN_PIN);

	gpio_set(RF_SW_CTRL_GPIO_PORT, RF_SW_CTRL_RXEN_PIN);
	gpio_clear(RF_SW_CTRL_GPIO_PORT, RF_SW_CTRL_TXEN_PIN);

	/* PB3 TXLED, PB4 RXLED, PB5 LINKLED */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO3 | GPIO4 | GPIO5);

#if 0
	/* according to RM0461 V8, P245, 6.2.1
	   The sub-GHz radio enables HSE32 autonomously for its own purpose, independently of the HSEON bit.
	 */
	/* enable HSE */
	rcc_osc_bypass_enable(RCC_HSE); /* ENABLE TXCO */
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
#endif

	/* Initialize SUBGHZ. */
	subghz_init();

	/* Set our payload. */
	subghz_set_buffer_base_address(0, 0);

	/* Enable LoRa mode. */
	mini_printf("HSE32 -> %s\n", rcc_is_osc_ready(RCC_HSE)?"ON":"OFF");
	xTaskCreate(lora_service, "LORA", 2000, &lora_config, 3, NULL);
	return 0;
}

void
lora_enter_rx(void)
{
	uint8_t r;

	r = subghz_get_status();
	r = SUBGHZ_STATUS_MODE(r);

	if ((r == SUBGHZ_STATUS_MODE_RX) || (r == SUBGHZ_STATUS_MODE_TX))
		return;

	/* Configure RX IRQ flags */
	if (SUBGHZ_CMD_FAILED(subghz_config_dio_irq(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
	                                            IRQ_RADIO_NONE, IRQ_RADIO_NONE)))
		return;

	/* Enable reception. */
	subghz_set_rx_mode(0xFFFFFF);
}

static void
lora_service(void *arg)
{
	subghz_lora_config_t *config = NULL;
	int r;
	uint8_t data[256];

	config = (subghz_lora_config_t *) arg;
	if (config == NULL) {
		mini_printf("arg == NULL, exit\n");
		vTaskDelete(NULL);
		return;
	}

	r = subghz_lora_mode(config);
	if (r != SUBGHZ_SUCCESS) {
		mini_printf("LORA MODE FAILED\n");
		vTaskDelete(NULL);
		return;
	}

	lora_rx_queue = xQueueCreate(10, 256);
	lora_tx_queue = xQueueCreate(10, 256);

	mini_printf("Enable LoRa -> OK\n");
	lora_started = 1;

	while (1) {
		r = xQueueReceive(lora_rx_queue, data, pdMS_TO_TICKS(0));
		if (r == pdPASS) {
			mini_printf("RX #%d data -> ", data[0]);
			print_hex(data + 1, data[0]);
		}
		lora_enter_rx();
		/* enter rx mode */
		r = xQueueReceive(lora_tx_queue, data, pdMS_TO_TICKS(1000));
		if (r != pdPASS)
			continue;
		/* Configure IRQ flags (no IRQ) */
		if (SUBGHZ_CMD_FAILED(subghz_config_dio_irq(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
							    IRQ_RADIO_NONE, IRQ_RADIO_NONE)))
		{
			continue;
		}

		/* Set payload buffer. */
		if (SUBGHZ_CMD_FAILED(subghz_set_payload(data + 1, data[0]))) {
			continue;
		}

		/* Reset TX and Timeout flags. */
		g_flag_tx_done = false;
		g_flag_timeout = false;

		/* Start transmission. */
		if (SUBGHZ_CMD_FAILED(subghz_set_tx_mode(0xFFFFFF)))
			continue;

		gpio_set(GPIOB, GPIO3);
		/* Wait for timeout or packet sent. */
		while ((!g_flag_tx_done) && (!g_flag_timeout)) ;
		gpio_clear(GPIOB, GPIO3);
	}
}

#if 0
int
lora_send(char *msg, int len)
{
	if (lora_started == 0) {
		mini_printf("lora not started\n");
		return 1;
	}

	/* Set TX mode (start transmission). */
	mini_printf("Send payload: %s\n", msg);
	if (subghz_send((uint8_t *)msg, len, 0) == SUBGHZ_ERROR)
		mini_printf("Send failed\n");

	print_reg_hex("status", SUBGHZ_STATUS_CMD(subghz_get_status()));
	return 0;
}
#endif

/* handle RF switch config. */
void
lora_rf_switch_cb(bool tx)
{
	  if (tx) {
	      /* TX mode, low power */
	      mini_printf("Enable TX (RF switch)\n");
	      gpio_set(RF_SW_CTRL_GPIO_PORT, RF_SW_CTRL_TXEN_PIN);
	      gpio_clear(RF_SW_CTRL_GPIO_PORT, RF_SW_CTRL_RXEN_PIN);
	  } else {
	      /* RX mode, low power */
	      mini_printf("Enable RX (RF switch)\n");
	      gpio_clear(RF_SW_CTRL_GPIO_PORT, RF_SW_CTRL_TXEN_PIN);
	      gpio_set(RF_SW_CTRL_GPIO_PORT, RF_SW_CTRL_RXEN_PIN);
	  }
}

void
print_reg_hex(char *prefix, uint8_t value)
{
	char digits[] = "0123456789abcdef";
	char digit[3];

	digit[0] = digits[(value & 0xf0)>>4];
	digit[1] = digits[(value & 0x0f)];
	digit[2] = '\0';
	console_puts(prefix);
	console_putc('=');
	console_puts(digit);
	console_puts("\n");
}

void
print_reg16_hex(char *prefix, uint16_t value)
{
	char digits[] = "0123456789abcdef";
	char digit[4];
	digit[0] = digits[(value & 0xf000) >> 12];
	digit[1] = digits[(value & 0x0f00) >> 8];
	digit[2] = digits[(value & 0x00f0) >> 4];
	digit[3] = digits[(value & 0x000f)];
	console_puts(prefix);
	console_putc('=');
	console_puts(digit);
	console_puts("\n");
}

int
lora_cmd(int argc, char **argv)
{
	int len;
	uint8_t data[256];

	/* lora init/send */
	if (argc >= 2 && strcasecmp(argv[1], "init") == 0) {
		/* lora int */
		if (lora_started == 1) {
			mini_printf("lora already started\n");
		} else {
			init_lora();
		}
		return 0;
	} else if (argc >= 3 && strcasecmp(argv[1], "send") == 0) {
		/* lora send data */
		len = strlen(argv[2]);
		if (len > 254)
			len = 254;
		memcpy(data + 1, argv[2], len);
		data[len + 1] = '\0';
		data[0] = len & 0xff;
		if (lora_tx_queue)
			xQueueSendToBack(lora_tx_queue, data, 0);
		return 0;
	} else if (argc >= 2 && strcasecmp(argv[1], "recv") == 0) {
		int r;
		uint8_t rx = 64;

		memset(data, 0, sizeof(data));

		r = subghz_receive((uint8_t *) data, &rx, 0xFFFFFF);
		mini_printf("got packet, rx = %d\r\n", rx);
		data[rx] = '\0';
		if (r == SUBGHZ_SUCCESS) {
			mini_printf("Received packet: %s\r\n", data);
		} else if (r == SUBGHZ_TIMEOUT) {
			mini_printf("Reception timed out\r\n");
		} else {
			mini_printf("An error occured\r\n");
		}
		return 0;
	}
	return 1;
}
