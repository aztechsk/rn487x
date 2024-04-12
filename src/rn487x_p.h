/*
 * rn487x_p.h
 *
 * Copyright (c) 2021 Jan Rusnak <jan@rusnak.sk>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef RN487X_P_H
#define RN487X_P_H

enum rn487x_addr_type {
	RN487X_ADDR_TYPE_PUBLIC,
        RN487X_ADDR_TYPE_RANDOM
};

enum rn487x_uuid_type {
	RN487X_UUID_TYPE_PUBLIC,
        RN487X_UUID_TYPE_PRIVATE
};

struct rn487x_service {
	enum rn487x_uuid_type uuid_type;
	const char *uuid;
        struct rn487x_characteristic *chr_list;
	struct rn487x_service *next;
};

enum rn487x_char_prop {
	RN487X_CHAR_PROP_EXT_PROPERTY  = 0x80, // Currently not supported.
	RN487X_CHAR_PROP_AUTH_WRITE    = 0x40, // Currently not supported.
        RN487X_CHAR_PROP_INDICATE      = 0x20,
        RN487X_CHAR_PROP_NOTIFY        = 0x10,
        RN487X_CHAR_PROP_WRITE         = 0x08,
        RN487X_CHAR_PROP_WRITE_NO_RESP = 0x04,
        RN487X_CHAR_PROP_READ          = 0x02,
	RN487X_CHAR_PROP_BROADCAST     = 0x01  // Currently not supported.
};

struct rn487x_characteristic {
	enum rn487x_uuid_type uuid_type;
	const char *uuid;
	const char *name;
	uint8_t props;
	uint8_t size;
	uint8_t *def_val;
	uint16_t handle;
	uint8_t eepr_props;
	uint16_t cccd_handle;
        uint8_t cccd_props;
        QueueHandle_t update;
        QueueHandle_t write;
        struct rn487x_characteristic *next;
};

enum rn487x_ad_type {
	RN487X_AD_TYPE_FLAGS          = 0x01,
	RN487X_AD_TYPE_UUIDS16_PART   = 0x02,
        RN487X_AD_TYPE_UUIDS16_COMPL  = 0x03,
        RN487X_AD_TYPE_UUIDS128_PART  = 0x06,
        RN487X_AD_TYPE_UUIDS128_COMPL = 0x07,
        RN487X_AD_TYPE_LOC_NAME_SHORT = 0x08,
        RN487X_AD_TYPE_LOC_NAME_COMPL = 0x09,
        RN487X_AD_TYPE_TX_POWER       = 0x0A,
	RN487X_AD_TYPE_DEVICE_CLASS   = 0x0D,
        RN487X_AD_TYPE_MAN_CUST_DATA  = 0xFF
};

struct rn487x_ad_record {
	enum rn487x_ad_type type;
	char *data;
	struct rn487x_ad_record *next;
};

enum rn487x_features {
	RN487X_FEAT_FLOW_CONTROL       = 0x8000,
        RN487X_FEAT_NO_PROMPT          = 0x4000,
        RN487X_FEAT_FAST_MODE          = 0x2000,
	RN487X_FEAT_NO_BEACON_SCAN     = 0x1000,
        RN487X_FEAT_NO_CONNECT_SCAN    = 0x0800,
        RN487X_FEAT_NO_DUP_SCAN_FILTER = 0x0400,
        RN487X_FEAT_PASSIVE_SCAN       = 0x0200,
        RN487X_FEAT_UART_TRANS_NO_ACK  = 0x0100,
        RN487X_FEAT_REBOOT_AFTER_DISC  = 0x0080,
        RN487X_FEAT_POWER_ON_SCRIPT    = 0x0040,
        RN487X_FEAT_MLDP_STREAM_SERV   = 0x0020,
        RN487X_FEAT_DISABLE_DLE        = 0x0010,
        RN487X_FEAT_COMMAND_MODE_GUARD = 0x0008
};

enum rn487x_default_services {
	RN487X_DEFAULT_SERV_DEV_INFO    = 0x80,
        RN487X_DEFAULT_SERV_TRANSP_UART = 0x40,
        RN487X_DEFAULT_SERV_BEACON      = 0x20
};

enum rn487x_io_cap {
	RN487X_IO_CAP_BOND_NO_IN_NO_OUT,
        RN487X_IO_CAP_DISP_YES_NO,
        RN487X_IO_CAP_NO_IN_NO_OUT,
        RN487X_IO_CAP_KEYB_ONLY,
        RN487X_IO_CAP_DISP_ONLY,
        RN487X_IO_CAP_DISP_KEYB
};

enum rn487x_advert_style {
	RN487X_ADVERT_STYLE_CONNECT,
        RN487X_ADVERT_STYLE_BEACON,
        RN487X_ADVERT_STYLE_CONNECT_BEACON
};

enum rn487x_tx_power {
	RN487X_TX_POWER_0_DBM,
	RN487X_TX_POWER_MIN_5_DBM,
	RN487X_TX_POWER_MIN_10_DBM,
        RN487X_TX_POWER_MIN_15_DBM,
        RN487X_TX_POWER_MIN_20_DBM,
        RN487X_TX_POWER_MIN_25_DBM
};

struct rn487x_def_conn_params {
	uint16_t min_interval; // 0x0006 - 0x0C80 (unit 1.25 ms).
	uint16_t max_interval; // 0x0006 - 0x0C80 (unit 1.25 ms).
        uint16_t latency;      // 0x0000 - 0x01F3 (< (timeout * 10 / interval * 1.25 - 1)).
        uint16_t timeout;      // 0x000A - 0x0C80 (unit 10 ms).
};

struct rn487x_def_advert_params {
	uint16_t fast_interval; // Unit 0.625 ms.
	uint16_t fast_timeout;  // Unit 10.24 sec.
        uint16_t slow_interval; // Unit 0.625 ms.
};

struct rn487x_advert_params {
	uint16_t interval;  // Unit 1 ms.
};

struct rn487x_def_beac_advert_params {
	uint16_t interval; // Unit 0.625 ms.
};

enum rn487x_pin {
	RN487X_PIN_P07,
        RN487X_PIN_P10,
        RN487X_PIN_P11,
        RN487X_PIN_P22,
        RN487X_PIN_P24,
        RN487X_PIN_P31,
        RN487X_PIN_P32,
        RN487X_PIN_P33,
        RN487X_PIN_P34,
        RN487X_PIN_P35,
        RN487X_PIN_P12,
        RN487X_PIN_P13,
        RN487X_PIN_P16,
        RN487X_PIN_P17,
	RN487X_PIN_ARY_END
};

enum rn487x_pin_func {
	RN487X_PIN_FUNC_NONE,
        RN487X_PIN_FUNC_LOW_BATT_IND,
        RN487X_PIN_FUNC_RSSI_IND,
        RN487X_PIN_FUNC_LINK_DROP,
        RN487X_PIN_FUNC_UART_RX_IND,
        RN487X_PIN_FUNC_PAIRING,
        RN487X_PIN_FUNC_RF_ACTIVE_IND,
        RN487X_PIN_FUNC_STAT_1,
        RN487X_PIN_FUNC_STAT_2,
        RN487X_PIN_FUNC_TRIG_1,
        RN487X_PIN_FUNC_TRIG_2,
        RN487X_PIN_FUNC_TRIG_3,
        RN487X_PIN_FUNC_UART_MODE_SWITCH
};

struct rn487x_pin_conf {
	enum rn487x_pin pin;
	enum rn487x_pin_func func;
};

enum rn487x_ext_reset_ctl {
	RN487X_EXT_RESET_ACTIVE,
        RN487X_EXT_RESET_OFF
};

struct rn487x_conf {
	void *ser_dev;
        int (*rcv_fn)(void *, void *, TickType_t);
        int (*snd_fn)(void *, void *, int);
        boolean_t (*intr_rx_fn)(void *);
	void (*ext_rst_ctl_fn)(enum rn487x_ext_reset_ctl reset_ctl);
#if RN487X_RX_IND_CTL == 1
	void (*rx_ind_ctl_fn)(boolean_t lev);
#endif
	const char *name;
	uint16_t features;
	uint8_t default_serv;
        struct rn487x_service *serv_list;
        uint16_t gap_appearance;
	const char *dis_model;
	const char *dis_factory;
	const char *dis_sw_version;
        const char *dis_serial;
	enum rn487x_io_cap io_cap;
	enum rn487x_advert_style advert_style;
	enum rn487x_tx_power advert_tx_power;
	enum rn487x_tx_power connect_tx_power;
	const char *pin6;
	boolean_t use_rnd_addr;
	boolean_t req_sec_link;
	struct rn487x_def_conn_params *def_conn_params;               // Optional.
        struct rn487x_def_advert_params *def_advert_params;           // Optional.
	struct rn487x_def_beac_advert_params *def_beac_advert_params; // Optional.
        struct rn487x_advert_params *advert_params; // Optional, override def_advert_params.
        struct rn487x_pin_conf *pins_conf; // Optional.
	struct rn487x_ad_record *advert_list;    // Optional.
	struct rn487x_ad_record *scan_resp_list; // Optional.
	struct rn487x_ad_record *beacon_list;    // Optional.
};

/**
 * init_rn487x
 */
void init_rn487x(const struct rn487x_conf *rn487x_conf);

/**
 * rn487x_update_gatt_local
 *
 * Request for update of characteristic's value on local GATT server.
 *
 * @name: Short characteristic's name.
 * @val: Array of bytes of value.
 *
 * Returns: TRUE - request accepted; FALSE - old update still pending.
 */
boolean_t rn487x_update_gatt_local(const char *name, const uint8_t *val);

/**
 * rn487x_wait_gatt_client_write
 *
 * Wait for characteristic's value update by remote client on local GATT server.
 *
 * @name: Short characteristic's name.
 * @buf: Buffer for saving bytes of value.
 * @tmo: Timeout in tick periods.
 *
 * Returns: TRUE - value saved; FALSE - timeout.
 */
boolean_t rn487x_wait_gatt_client_write(const char *name, uint8_t *buf, TickType_t tmo);

/**
 * rn487x_read_gatt_client_write
 *
 * Read characteristic's updated value on local GATT server.
 * Update of value must be notified by RN487X_EVENT_GATT_CLIENT_WRITE event before
 * function call.
 *
 * @name: Short characteristic name.
 * @buf: Buffer for saving bytes of value.
 */
void rn487x_read_gatt_client_write(const char *name, uint8_t *buf);

enum rn487x_event_type {
	RN487X_EVENT_WAIT_TIMEOUT,
	RN487X_EVENT_CONNECTED,
	RN487X_EVENT_CONN_SECURED,
	RN487X_EVENT_CONN_SEC_ERR,
        RN487X_EVENT_BONDED,
        RN487X_EVENT_DISCONNECTED,
	RN487X_EVENT_GATT_CLIENT_WRITE,
	RN487X_EVENT_CMD_OK,
	RN487X_EVENT_CMD_ERR,
	RN487X_EVENT_SECURITY_KEY
};

struct rn487x_event {
	enum rn487x_event_type event_type;
	union {
		const char *chr_name;
		char key[7];
	} param;
};

/**
 * rn487x_wait_event
 *
 * Wait for event generated by BLE state machine.
 *
 * @tmo: Wait timeout in tick periods.
 *
 * Returns: Event struct.
 */
struct rn487x_event rn487x_wait_event(TickType_t tmo);

/**
 * rn487x_chr_data_size
 *
 * @name: Short characteristic's name.
 *
 * Returns: Size of characteristic's data.
 */
int rn487x_chr_data_size(const char *name);

struct rn487x_bond_tbl_row {
	int idx;
	enum rn487x_addr_type atype;
        uint8_t addr[6];
};

struct rn487x_whl_row {
	enum rn487x_addr_type atype;
        uint8_t addr[6];
};

enum rn487x_cmd_type {
	RN487X_CMD_RESET = 1,
	RN487X_CMD_EXT_RESET,
	RN487X_CMD_DISCONNECT,
	RN487X_CMD_SET_PUB_ADDR,  // RN487X_EVENT_CMD_OK, RN487X_EVENT_CMD_ERR.
	RN487X_CMD_SET_FACT_DEF,  // RN487X_EVENT_CMD_OK, RN487X_EVENT_CMD_ERR.
        RN487X_CMD_REQ_SEC_LINK,
	RN487X_CMD_CLR_ALL_BOND,  // RN487X_EVENT_CMD_OK, RN487X_EVENT_CMD_ERR.
	RN487X_CMD_CLR_WHL,       // RN487X_EVENT_CMD_OK, RN487X_EVENT_CMD_ERR.
	RN487X_CMD_BOND_TO_WHL,   // RN487X_EVENT_CMD_OK, RN487X_EVENT_CMD_ERR.
	RN487X_CMD_READ_ALL_BOND, // RN487X_EVENT_CMD_OK, RN487X_EVENT_CMD_ERR.
	RN487X_CMD_READ_WHL       // RN487X_EVENT_CMD_OK, RN487X_EVENT_CMD_ERR.
};

struct rn487x_cmd_common {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_reset {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_ext_reset {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_disconnect {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_set_pub_addr {
	enum rn487x_cmd_type cmd_type;
	uint8_t bta[6];
};

struct rn487x_cmd_set_fact_def {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_req_sec_link {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_clr_all_bond {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_clr_whl {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_bond_to_whl {
	enum rn487x_cmd_type cmd_type;
};

struct rn487x_cmd_read_all_bond {
	enum rn487x_cmd_type cmd_type;
        struct rn487x_bond_tbl_row (*btbl)[8];
};

struct rn487x_cmd_read_whl {
	enum rn487x_cmd_type cmd_type;
        struct rn487x_whl_row (*whl)[16];
};

union rn487x_cmd {
        struct rn487x_cmd_common common;
	struct rn487x_cmd_reset reset;
	struct rn487x_cmd_ext_reset ext_reset;
        struct rn487x_cmd_disconnect disconnect;
	struct rn487x_cmd_set_pub_addr set_pub_addr;
        struct rn487x_cmd_set_fact_def set_fact_def;
        struct rn487x_cmd_req_sec_link req_sec_link;
        struct rn487x_cmd_clr_all_bond clr_all_bond;
        struct rn487x_cmd_clr_whl clr_whl;
        struct rn487x_cmd_bond_to_whl bond_to_whl;
        struct rn487x_cmd_read_all_bond read_all_bond;
        struct rn487x_cmd_read_whl read_whl;
};

/**
 * rn487x_send_cmd
 *
 * Send RN487X control command.
 *
 * @req: Pointer to command definition structure.
 */
void rn487x_send_cmd(const union rn487x_cmd *cmd);

#if TERMOUT == 1
/**
 * log_rn487x_stats
 */
void log_rn487x_stats(void);
#endif

#endif
