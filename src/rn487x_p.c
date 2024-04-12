/*
 * rn487x_p.c
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

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <gentyp.h>
#include "sysconf.h"
#include "board.h"
#include "atom.h"
#include "msgconf.h"
#include "criterr.h"
#include "hwerr.h"
#include "tools.h"
#include "rn487x_p.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>

#define EXT_RESET_TIME (50 / portTICK_PERIOD_MS)
#define EXT_RESET_AGAIN_TMO (5000 / portTICK_PERIOD_MS)
#define RX_IND_TMO (10 / portTICK_PERIOD_MS)
#define CMD_RESET_TMO (1000 / portTICK_PERIOD_MS)
#define CMD_FACT_DEF_TMO (1000 / portTICK_PERIOD_MS)
#define RESET_STATE_1_DLY (200 / portTICK_PERIOD_MS)
#define RESET_STATE_DLY (800 / portTICK_PERIOD_MS)
#define CMD_TMO (250 / portTICK_PERIOD_MS)
#define CMD_PC_TMO (800 / portTICK_PERIOD_MS)
#define CMD_SHW_TMO (1000 / portTICK_PERIOD_MS)
#define CON_SEC_ERR_D_TMO (3000 / portTICK_PERIOD_MS)
#define CON_SEC_ERR_Y_TMO (2000 / portTICK_PERIOD_MS)
#define DISCON_TMO (10000 / portTICK_PERIOD_MS)
#define PROMPT_TMO (100 / portTICK_PERIOD_MS)
#define INIT_DELAY (150 / portTICK_PERIOD_MS)
#define RX_CHAR_TMO (25 / portTICK_PERIOD_MS)
#define RX_CHAR_EXT_TMO (50 / portTICK_PERIOD_MS)
#define RNFWV_ARY_SIZE 12

enum status_id {
	STATUS_ID_ADV_TIMEOUT,
	STATUS_ID_BONDED,
	STATUS_ID_CONN_PARAM,
        STATUS_ID_CONNECT,
        STATUS_ID_DISCONNECT,
        STATUS_ID_ERR_CONNPARAM,
        STATUS_ID_ERR_MEMORY,
        STATUS_ID_ERR_READ,
        STATUS_ID_ERR_RMT_CMD,
        STATUS_ID_ERR_SEC,
        STATUS_ID_KEY,
        STATUS_ID_KEY_REQ,
        STATUS_ID_INDICATION,
        STATUS_ID_NOTIFICATION,
        STATUS_ID_PIO1H,
        STATUS_ID_PIO1L,
        STATUS_ID_PIO2H,
        STATUS_ID_PIO2L,
        STATUS_ID_PIO3H,
        STATUS_ID_PIO3L,
        STATUS_ID_RE_DISCV,
        STATUS_ID_REBOOT,
        STATUS_ID_RMT_CMD_OFF,
        STATUS_ID_RMT_CMD_ON,
        STATUS_ID_RV,
        STATUS_ID_S_RUN,
        STATUS_ID_SECURED,
        STATUS_ID_STREAM_OPEN,
        STATUS_ID_TMR1,
        STATUS_ID_TMR2,
        STATUS_ID_TMR3,
	STATUS_ID_WC,
        STATUS_ID_WV,
	STATUS_ID_UNKNOWN_DEVICE,
	STATUS_ID_CON_ADVERT,
        STATUS_ID_NO_CON_ADVERT
};

#if TERMOUT == 1
static const struct txt_item status_id_desc_ary[] = {
	{STATUS_ID_ADV_TIMEOUT, "ADV_TIMEOUT"},
	{STATUS_ID_BONDED, "BONDED"},
	{STATUS_ID_CONN_PARAM, "CONN_PARAM"},
        {STATUS_ID_CONNECT, "CONNECT"},
        {STATUS_ID_DISCONNECT, "DISCONNECT"},
        {STATUS_ID_ERR_CONNPARAM, "ERR_CONNPARAM"},
        {STATUS_ID_ERR_MEMORY, "ERR_MEMORY"},
        {STATUS_ID_ERR_READ, "ERR_READ"},
        {STATUS_ID_ERR_RMT_CMD, "ERR_RMT_CMD"},
        {STATUS_ID_ERR_SEC, "ERR_SEC"},
        {STATUS_ID_KEY, "KEY"},
        {STATUS_ID_KEY_REQ, "KEY_REQ"},
        {STATUS_ID_INDICATION, "INDICATION"},
        {STATUS_ID_NOTIFICATION, "NOTIFICATION"},
        {STATUS_ID_PIO1H, "PIO1H"},
        {STATUS_ID_PIO1L, "PIO1L"},
        {STATUS_ID_PIO2H, "PIO2H"},
        {STATUS_ID_PIO2L, "PIO2L"},
        {STATUS_ID_PIO3H, "PIO3H"},
        {STATUS_ID_PIO3L, "PIO3L"},
        {STATUS_ID_RE_DISCV, "RE_DISCV"},
        {STATUS_ID_REBOOT, "REBOOT"},
        {STATUS_ID_RMT_CMD_OFF, "RMT_CMD_OF"},
        {STATUS_ID_RMT_CMD_ON, "RMT_CMD_ON"},
        {STATUS_ID_RV, "RV"},
        {STATUS_ID_S_RUN, "S_RUN"},
        {STATUS_ID_SECURED, "SECURED"},
        {STATUS_ID_STREAM_OPEN, "STREAM_OPEN"},
        {STATUS_ID_TMR1, "TMR1"},
        {STATUS_ID_TMR2, "TMR2"},
        {STATUS_ID_TMR3, "TMR3"},
	{STATUS_ID_WC, "WC"},
        {STATUS_ID_WV, "WV"},
	{STATUS_ID_UNKNOWN_DEVICE, "UNKNOWN_DEVICE"},
	{STATUS_ID_CON_ADVERT, "CON_ADVERT"},
        {STATUS_ID_NO_CON_ADVERT, "NO_CON_ADVERT"},
	{0, NULL}
};
#endif

enum client_notif_req {
	CLIENT_NOTIF_OFF,
	CLIENT_NOTIF_ON = 0x0100,
	CLIENT_INDI_ON = 0x0200
};

struct status_common {
	enum status_id id;
};

struct status_adv_timeout {
	enum status_id id;
};

struct status_bonded {
	enum status_id id;
};

struct status_conn_param {
	enum status_id id;
};

struct status_connect {
	enum status_id id;
	enum rn487x_addr_type addr_type;
	uint8_t addr[6];
};

struct status_disconnect {
	enum status_id id;
};

struct status_err_connparam {
	enum status_id id;
};

struct status_err_memory {
	enum status_id id;
};

struct status_err_read {
	enum status_id id;
};

struct status_err_rmt_cmd {
	enum status_id id;
};

struct status_err_sec {
	enum status_id id;
};

struct status_key {
	enum status_id id;
	char key[7];
};

struct status_key_req {
	enum status_id id;
};

struct status_indication {
	enum status_id id;
};

struct status_notification {
	enum status_id id;
};

struct status_pio1h {
	enum status_id id;
};

struct status_pio1l {
	enum status_id id;
};

struct status_pio2h {
	enum status_id id;
};

struct status_pio2l {
	enum status_id id;
};

struct status_pio3h {
	enum status_id id;
};

struct status_pio3l {
	enum status_id id;
};

struct status_re_discv {
	enum status_id id;
};

struct status_reboot {
	enum status_id id;
};

struct status_rmt_cmd_off {
	enum status_id id;
};

struct status_rmt_cmd_on {
	enum status_id id;
};

struct status_rv {
	enum status_id id;
};

struct status_s_run {
	enum status_id id;
};

struct status_secured {
	enum status_id id;
};

struct status_stream_open {
	enum status_id id;
};

struct status_tmr1 {
	enum status_id id;
};

struct status_tmr2 {
	enum status_id id;
};

struct status_tmr3 {
	enum status_id id;
};

struct status_wc {
	enum status_id id;
	uint16_t handle;
	enum client_notif_req notif_req;
};

struct status_wv {
	enum status_id id;
};

struct status_unknown_dev {
	enum status_id id;
};

struct status_con_advert {
	enum status_id id;
};

struct status_no_con_advert {
	enum status_id id;
};

union status {
	struct status_common common;
	struct status_adv_timeout adv_timeout;
	struct status_bonded bonded;
	struct status_conn_param conn_param;
	struct status_connect connect;
	struct status_disconnect disconnect;
	struct status_err_connparam err_connparam;
	struct status_err_memory err_memory;
	struct status_err_read err_read;
	struct status_err_rmt_cmd err_rmt_cmd;
	struct status_err_sec err_sec;
	struct status_key key;
	struct status_key_req key_req;
	struct status_indication indication;
	struct status_notification notification;
	struct status_pio1h pio1h;
	struct status_pio1l pio1l;
	struct status_pio2h pio2h;
	struct status_pio2l pio2l;
	struct status_pio3h pio3h;
	struct status_pio3l pio3l;
	struct status_re_discv re_discv;
	struct status_reboot reboot;
	struct status_rmt_cmd_off rmt_cmd_off;
	struct status_rmt_cmd_on rmt_cmd_on;
	struct status_rv rv;
	struct status_s_run s_run;
	struct status_secured secured;
	struct status_stream_open stream_open;
	struct status_tmr1 tmr1;
	struct status_tmr2 tmr2;
	struct status_tmr3 tmr3;
	struct status_wc wc;
	struct status_wv wv;
	struct status_unknown_dev unknown_dev;
	struct status_con_advert con_advert;
	struct status_no_con_advert no_con_advert;
};

struct stats {
	int rx_msg_char_cnt;
	int rx_msg_ebfov;
        int rx_msg_ercv;
	int rx_msg_efmt;
        int rx_msg_nost1;
	int st_pars_err;
        int unpar_status;
	int unpar_adv_status;
	int unpar_con_status;
        int unpar_cse_status;
	int unpar_dis_status;
	int reset;
	int ext_reset;
	int sscanf_err;
	int fifo_full;
	int adv_rxp_err;
        int con_rxp_err;
	int cse_rxp_err;
        int dis_rxp_err;
	int nfail;
	int cli_wr_que_full;
        int stm_evnt_que_full;
};

enum rx_parser_ret {
	RX_PARSER_RET_CMD_AOK,
	RX_PARSER_RET_CMD_ERR,
	RX_PARSER_RET_CMDP,
        RX_PARSER_RET_STRING,
	RX_PARSER_RET_STATUS,
	RX_PARSER_RET_TMO,
	RX_PARSER_RET_INTR,
	RX_PARSER_RET_ERROR
};

#if TERMOUT == 1
static const struct txt_item rxp_ret_desc_ary[] = {
	{RX_PARSER_RET_CMD_AOK, "CMD_AOK"},
	{RX_PARSER_RET_CMD_ERR, "CMD_ERR"},
	{RX_PARSER_RET_CMDP, "CMDP"},
	{RX_PARSER_RET_STRING, "STRING"},
        {RX_PARSER_RET_STATUS, "STATUS"},
	{RX_PARSER_RET_TMO, "TMO"},
	{RX_PARSER_RET_INTR, "INTR"},
	{RX_PARSER_RET_ERROR, "ERROR"},
	{0, NULL}
};
#endif

struct dev_addr {
	uint8_t bta[6];
	uint8_t rna[6];
};

enum cmd_ret {
	CMD_RET_AOK,
	CMD_RET_ERR,
	CMD_RET_STR,
	CMD_RET_FAIL
};

static p_stf_t stmf;
static const struct rn487x_conf *conf;
static char cmdb[RN487X_CMDB_SIZE];
static char rxb[RN487X_RXB_SIZE];
static QueueHandle_t status_fifo;
static struct stats stats;
static boolean_t eepr_corrupted;
static struct dev_addr dev_addr;
static char rnfwv[RNFWV_ARY_SIZE] = "RNFW ";
static struct status_connect peer;
static boolean_t conn_secured;
static QueueHandle_t rn487x_cmd_que;
static union rn487x_cmd rn487x_cmd_act;
static QueueHandle_t stm_evnt_que;
static boolean_t connected;

static void stm_tsk(void *p);
static gfp_t state_reset(void);
static gfp_t state_ext_reset(void);
static gfp_t state_after_reset(void);
static gfp_t state_check_config(void);
static gfp_t state_check_services(void);
static gfp_t state_conf_services(void);
static gfp_t state_device_info(void);
static gfp_t state_set_advert_data(void);
static gfp_t state_advertise(void);
static gfp_t state_connected(void);
static gfp_t state_con_sec_err(void);
static gfp_t state_disconnect(void);
static gfp_t state_set_pub_bta(void);
static gfp_t state_set_fact_def(void);
static boolean_t set_adv_payload(const char *cmd, const struct rn487x_ad_record *ad);
#if TERMOUT == 1
static int adv_payload_size(const struct rn487x_ad_record *ad);
#endif
static int check_srv_services(void);
static int services_parser(char *row);
static struct rn487x_characteristic *gatt_update_pend(void);
static boolean_t cmd_shw(const struct rn487x_characteristic *chr);
static enum cmd_ret cmd_string(const char *cmd, const char *str);
static int cmd_check_string(const char *cmd, const char *str);
static enum cmd_ret cmd_1hex(const char *cmd, uint8_t n);
static int cmd_check_1hex(const char *cmd, uint8_t n);
static enum cmd_ret cmd_n_2hex(const char *cmd, int par_num, ...);
static int cmd_check_n_2hex(const char *cmd, int par_num, ...);
static enum cmd_ret cmd_n_4hex(const char *cmd, int par_num, ...);
static int cmd_check_n_4hex(const char *cmd, int par_num, ...);
static enum cmd_ret cmd_noarg(const char *cmd);
static int cmd_gw(uint8_t pin, uint8_t func);
static boolean_t get_fw_version(void);
static boolean_t read_all_bond(struct rn487x_bond_tbl_row (*btbl)[8]);
static boolean_t read_whl(struct rn487x_whl_row (*whl)[16]);
static boolean_t cmdp(const char *cmd);
static enum cmd_ret wait_cmd_ret(const char *cmd, TickType_t tmo);
static void snd_stm_evnt(enum rn487x_event_type type);
static boolean_t clear_status_fifo(const char *msg);
static boolean_t conv_addr_str_bin(const char *str, uint8_t *bin);
static enum rx_parser_ret rx_parser(TickType_t tmo);
static void rx_stat_parser(enum rx_parser_ret *ret);
static int rx_msg(TickType_t tmo);
static boolean_t match_start(char *m, char *s);
static struct rn487x_characteristic *chr_search_by_h(uint16_t h);
static struct rn487x_characteristic *chr_search_by_n(const char *n);
static boolean_t is_bt_addr_inval(const uint8_t *a);
#if TERMOUT == 1 && RN487X_LOG_LEVEL > 0
static void log_notif_req(const struct status_wc *wc);
#endif
#if TERMOUT == 1
static void log_def_params(void);
#endif
#if RN487X_LOG_CMD == 1
static void log_cmd(char *cmd);
#endif
static void upd_chr_def_vals(void);
static p_stf_t cmd_handler(p_stf_t stf);

/**
 * init_rn487x
 */
void init_rn487x(const struct rn487x_conf *rn487x_conf)
{
	struct rn487x_service *srv;
	struct rn487x_characteristic *chr;

	if (!(conf = rn487x_conf)) {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	if ((srv = conf->serv_list)) {
		do {
			if (!(chr = srv->chr_list)) {
				crit_err_exit(UNEXP_PROG_STATE);
			}
			do {
				if (NULL == (chr->update = xQueueCreate(1, chr->size))) {
					crit_err_exit(MALLOC_ERROR);
				}
				if (chr->props & (RN487X_CHAR_PROP_WRITE | RN487X_CHAR_PROP_WRITE_NO_RESP)) {
					if (NULL == (chr->write = xQueueCreate(RN487X_CLIENT_WRITE_QUEUE_SIZE, chr->size))) {
						crit_err_exit(MALLOC_ERROR);
					}
				}
			} while ((chr = chr->next));
		} while ((srv = srv->next));
	}
	if (NULL == (status_fifo = xQueueCreate(RN487X_STATUS_FIFO_SIZE, sizeof(union status)))) {
		crit_err_exit(MALLOC_ERROR);
	}
	if (NULL == (rn487x_cmd_que = xQueueCreate(1, sizeof(union rn487x_cmd)))) {
		crit_err_exit(MALLOC_ERROR);
	}
	if (NULL == (stm_evnt_que = xQueueCreate(RN487X_STM_EVENT_QUEUE_SIZE, sizeof(struct rn487x_event)))) {
		crit_err_exit(MALLOC_ERROR);
	}
	if (pdPASS != xTaskCreate(stm_tsk, "RN487X", RN487X_STM_TASK_STACK_SIZE, NULL, RN487X_STM_TASK_PRIO, NULL)) {
		crit_err_exit(MALLOC_ERROR);
	}
}

/**
 * rn487x_update_gatt_local
 */
boolean_t rn487x_update_gatt_local(const char *name, const uint8_t *val)
{
	struct rn487x_characteristic *chr;
	boolean_t ret = FALSE;

	if (conf->serv_list) {
		if (NULL != (chr = chr_search_by_n(name))) {
			if (pdTRUE == xQueueSend(chr->update, val, 0)) {
				ret = TRUE;
			}
			conf->intr_rx_fn(conf->ser_dev);
		} else {
			crit_err_exit(BAD_PARAMETER);
		}
	} else {
		crit_err_exit(UNEXP_PROG_STATE);
	}
        return (ret);
}

/**
 * rn487x_wait_gatt_client_write
 */
boolean_t rn487x_wait_gatt_client_write(const char *name, uint8_t *buf, TickType_t tmo)
{
	struct rn487x_characteristic *chr;
        boolean_t ret = FALSE;

	if (conf->serv_list) {
		if (NULL != (chr = chr_search_by_n(name))) {
			if (chr->write) {
				if (pdTRUE == xQueueReceive(chr->write, buf, tmo)) {
					ret = TRUE;
				}
			} else {
				crit_err_exit(UNEXP_PROG_STATE);
			}
		} else {
			crit_err_exit(BAD_PARAMETER);
		}
	} else {
		crit_err_exit(UNEXP_PROG_STATE);
	}
        return (ret);
}

/**
 * rn487x_read_gatt_client_write
 */
void rn487x_read_gatt_client_write(const char *name, uint8_t *buf)
{
	struct rn487x_characteristic *chr;

	if (conf->serv_list) {
		if (NULL != (chr = chr_search_by_n(name))) {
			if (chr->write) {
				if (pdFALSE == xQueueReceive(chr->write, buf, 0)) {
					crit_err_exit(UNEXP_PROG_STATE);
				}
			} else {
				crit_err_exit(UNEXP_PROG_STATE);
			}
		} else {
			crit_err_exit(BAD_PARAMETER);
		}
	} else {
		crit_err_exit(UNEXP_PROG_STATE);
	}
}

/**
 * rn487x_wait_event
 */
struct rn487x_event rn487x_wait_event(TickType_t tmo)
{
	struct rn487x_event event;

	memset(&event, 0, sizeof(struct rn487x_event));
	xQueueReceive(stm_evnt_que, &event, tmo);
	return (event);
}

/**
 * rn487x_chr_data_size
 */
int rn487x_chr_data_size(const char *name)
{
	struct rn487x_characteristic *chr;
	int sz = 0;

	if (conf->serv_list) {
		if (NULL != (chr = chr_search_by_n(name))) {
			sz = chr->size;
		} else {
			crit_err_exit(BAD_PARAMETER);
		}
	} else {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	return (sz);
}

/**
 * stm_tsk
 */
static void stm_tsk(void *p)
{
	vTaskDelay(INIT_DELAY);
#if RN487X_EXT_RST_INIT == 1
	stmf = state_ext_reset;
#else
	stmf = state_reset;
#endif
	while (TRUE) {
		stmf = (p_stf_t) (*stmf)();
	}
}

/**
 * state_reset
 */
static gfp_t state_reset(void)
{
	enum rx_parser_ret ret;
	char cmd_r[] = "R,1";
	char cmd_cm[] = "$$$";
	char cmd_g[] = "G$";
	boolean_t reboot;
	static boolean_t rsd = FALSE;
        TickType_t tmo;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> reset\n");
#endif
#if RN487X_RX_IND_CTL == 1
	conf->rx_ind_ctl_fn(LOW);
        vTaskDelay(RX_IND_TMO);
#endif
	if (connected) {
		connected = FALSE;
                snd_stm_evnt(RN487X_EVENT_DISCONNECTED);
	}
	stats.reset++;
        upd_chr_def_vals();
	for (int i = 0; i < 2; i++) {
		if (rsd) {
			while (RX_PARSER_RET_TMO != rx_parser(RESET_STATE_DLY));
		} else {
			while (RX_PARSER_RET_TMO != rx_parser(RESET_STATE_1_DLY));
			rsd = TRUE;
		}
                clear_status_fifo("on reset");
#if RN487X_LOG_CMD == 1
		log_cmd(cmd_g);
#endif
		cmd_g[sizeof(cmd_g) - 1] = '\r';
		conf->snd_fn(conf->ser_dev, cmd_g, sizeof(cmd_g));
		cmd_g[sizeof(cmd_g) - 1] = '\0';
		while (TRUE) {
			ret = rx_parser(CMD_TMO);
			if (ret == RX_PARSER_RET_STRING && rxb[0] == '$') {
				if (cmdp(cmd_g)) {
#if RN487X_LOG_LEVEL > 0
					msg(INF, "RN# Command mode (rst)\n");
#endif
					break;
				} else {
					return ((gfp_t) state_ext_reset);
				}
			} else if (ret == RX_PARSER_RET_TMO) {
#if RN487X_LOG_LEVEL > 0
				msg(INF, "RN# Not respond, module in data mode? (rst)\n");
#endif
#if RN487X_LOG_CMD == 1
				log_cmd(cmd_cm);
#endif
				conf->snd_fn(conf->ser_dev, cmd_cm, 3);
				if (cmdp(cmd_cm)) {
#if RN487X_LOG_LEVEL > 0
					msg(INF, "RN# Command mode (rst)\n");
#endif
					break;
				} else {
					return ((gfp_t) state_ext_reset);
				}
			}
		}
#if RN487X_LOG_CMD == 1
		log_cmd(cmd_r);
#endif
		cmd_r[sizeof(cmd_r) - 1] = '\r';
                reboot = FALSE;
		tmo = CMD_RESET_TMO;
#if RN487X_LOG_LEVEL > 0
                msg(INF, "RN# Resetting (sw) ...\n");
#endif
		conf->snd_fn(conf->ser_dev, cmd_r, sizeof(cmd_r));
		while (TRUE) {
			ret = rx_parser(tmo);
			if (ret == RX_PARSER_RET_STATUS) {
				union status st;
				xQueueReceive(status_fifo, &st, 0);
				if (st.common.id == STATUS_ID_REBOOT) {
					reboot = TRUE;
                                        tmo = CMD_TMO;
				} else if (st.common.id == STATUS_ID_UNKNOWN_DEVICE) {
					eepr_corrupted = TRUE;
					msg(INF, "RN# !! Factory setting eeprom corrupted !!\n");
				}
			} else if (ret == RX_PARSER_RET_TMO) {
				break;
			}
		}
		if (!reboot) {
			continue;
		}
                msg(INF, "RN# Reboot ok\n");
#if RN487X_LOG_CMD == 1
		log_cmd(cmd_cm);
#endif
		conf->snd_fn(conf->ser_dev, cmd_cm, 3);
		if (cmdp(cmd_cm)) {
#if RN487X_LOG_LEVEL > 0
			msg(INF, "RN# Command mode\n");
#endif
                        return ((gfp_t) state_after_reset);
		} else {
			return ((gfp_t) state_ext_reset);
		}
	}
	msg(INF, "RN# Device do not answer, trying ext_reset\n");
        return ((gfp_t) state_ext_reset);
}

/**
 * state_ext_reset
 */
static gfp_t state_ext_reset(void)
{
	enum rx_parser_ret ret;
        char cmd_cm[] = "$$$";
	TickType_t tmo;
        boolean_t reboot;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> ext_reset\n");
#endif
#if RN487X_RX_IND_CTL == 1
	conf->rx_ind_ctl_fn(LOW);
        vTaskDelay(RX_IND_TMO);
#endif
	if (connected) {
		connected = FALSE;
                snd_stm_evnt(RN487X_EVENT_DISCONNECTED);
	}
	stats.ext_reset++;
        upd_chr_def_vals();
	while (RX_PARSER_RET_TMO != rx_parser(RESET_STATE_1_DLY));
	clear_status_fifo("on ext reset");
#if RN487X_LOG_LEVEL > 0
	msg(INF, "RN# Resetting (ext) ...\n");
#endif
	conf->ext_rst_ctl_fn(RN487X_EXT_RESET_ACTIVE);
	vTaskDelay(EXT_RESET_TIME);
        conf->ext_rst_ctl_fn(RN487X_EXT_RESET_OFF);
	reboot = FALSE;
	tmo = CMD_RESET_TMO;
	while (TRUE) {
		ret = rx_parser(tmo);
		if (ret == RX_PARSER_RET_STATUS) {
			union status st;
			xQueueReceive(status_fifo, &st, 0);
			if (st.common.id == STATUS_ID_REBOOT) {
				reboot = TRUE;
				tmo = CMD_TMO;
			} else if (st.common.id == STATUS_ID_UNKNOWN_DEVICE) {
				eepr_corrupted = TRUE;
				msg(INF, "RN# !! Factory setting eeprom corrupted !!\n");
			}
		} else if (ret == RX_PARSER_RET_TMO) {
			break;
		}
	}
	if (!reboot) {
		vTaskDelay(EXT_RESET_AGAIN_TMO);
		return ((gfp_t) state_ext_reset);
	}
        msg(INF, "RN# Reboot ok\n");
#if RN487X_LOG_CMD == 1
	log_cmd(cmd_cm);
#endif
	conf->snd_fn(conf->ser_dev, cmd_cm, 3);
	if (cmdp(cmd_cm)) {
#if RN487X_LOG_LEVEL > 0
		msg(INF, "RN# Command mode\n");
#endif
                return ((gfp_t) state_after_reset);
	} else {
		return ((gfp_t) state_ext_reset);
	}
}

/**
 * state_after_reset
 */
static gfp_t state_after_reset(void)
{
#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> after_reset\n");
#endif
	if (CMD_RET_AOK != cmd_noarg("Y")) {
		return ((gfp_t) state_reset);
	}
	if (rn487x_cmd_act.common.cmd_type == RN487X_CMD_SET_PUB_ADDR) {
		return ((gfp_t) state_set_pub_bta);
	} else if (rn487x_cmd_act.common.cmd_type == RN487X_CMD_SET_FACT_DEF) {
		return ((gfp_t) state_set_fact_def);
	} else {
		return ((gfp_t) state_check_config);
	}
}

/**
 * state_check_config
 */
static gfp_t state_check_config(void)
{
	int ret;
	boolean_t reboot = FALSE;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> check_config\n");
#endif
	if (!get_fw_version()) {
		return ((gfp_t) state_reset);
	}
        if (0 != (ret = cmd_check_n_4hex("GR", 1, conf->features))) {
		if (ret == -EDATA) {
			reboot = TRUE;
                        if (CMD_RET_AOK != cmd_n_4hex("SR", 1, conf->features)) {
				return ((gfp_t) state_reset);
			}
		} else {
			return ((gfp_t) state_reset);
		}
	}
        if (0 != (ret = cmd_check_1hex("GA", conf->io_cap))) {
		if (ret == -EDATA) {
			reboot = TRUE;
                        if (CMD_RET_AOK != cmd_1hex("SA", conf->io_cap)) {
				return ((gfp_t) state_reset);
			}
		} else {
			return ((gfp_t) state_reset);
		}
	}
        if (0 != (ret = cmd_check_1hex("GC", conf->advert_style))) {
		if (ret == -EDATA) {
			reboot = TRUE;
                        if (CMD_RET_AOK != cmd_1hex("SC", conf->advert_style)) {
				return ((gfp_t) state_reset);
			}
		} else {
			return ((gfp_t) state_reset);
		}
	}
        if (0 != (ret = cmd_check_1hex("GGA", conf->advert_tx_power))) {
		if (ret == -EDATA) {
			reboot = TRUE;
                        if (CMD_RET_AOK != cmd_1hex("SGA", conf->advert_tx_power)) {
				return ((gfp_t) state_reset);
			}
		} else {
			return ((gfp_t) state_reset);
		}
	}
        if (0 != (ret = cmd_check_1hex("GGC", conf->connect_tx_power))) {
		if (ret == -EDATA) {
			reboot = TRUE;
                        if (CMD_RET_AOK != cmd_1hex("SGC", conf->connect_tx_power)) {
				return ((gfp_t) state_reset);
			}
		} else {
			return ((gfp_t) state_reset);
		}
	}
        if (0 != (ret = cmd_check_string("GN", conf->name))) {
		if (ret == -EDATA) {
			reboot = TRUE;
                        if (CMD_RET_AOK != cmd_string("SN", conf->name)) {
				return ((gfp_t) state_reset);
			}
		} else {
			return ((gfp_t) state_reset);
		}
	}
        if (0 != (ret = cmd_check_string("GP", conf->pin6))) {
		if (ret == -EDATA) {
			reboot = TRUE;
                        if (CMD_RET_AOK != cmd_string("SP", conf->pin6)) {
				return ((gfp_t) state_reset);
			}
		} else {
			return ((gfp_t) state_reset);
		}
	}
	if (conf->def_conn_params) {
		if (0 != (ret = cmd_check_n_4hex("GT", 4,
						 conf->def_conn_params->min_interval,
						 conf->def_conn_params->max_interval,
                                                 conf->def_conn_params->latency,
                                                 conf->def_conn_params->timeout))) {
			if (ret == -EDATA) {
				reboot = TRUE;
				if (CMD_RET_AOK != cmd_n_4hex("ST", 4,
							      conf->def_conn_params->min_interval,
							      conf->def_conn_params->max_interval,
							      conf->def_conn_params->latency,
						              conf->def_conn_params->timeout)) {
					return ((gfp_t) state_reset);
				}
			} else {
				return ((gfp_t) state_reset);
			}
		}
	}
	if (conf->def_advert_params) {
		if (0 != (ret = cmd_check_n_4hex("GTA", 3,
		                                 conf->def_advert_params->fast_interval,
						 conf->def_advert_params->fast_timeout,
                                                 conf->def_advert_params->slow_interval))) {
			if (ret == -EDATA) {
				reboot = TRUE;
				if (CMD_RET_AOK != cmd_n_4hex("STA", 3,
							      conf->def_advert_params->fast_interval,
							      conf->def_advert_params->fast_timeout,
							      conf->def_advert_params->slow_interval)) {
					return ((gfp_t) state_reset);
				}
			} else {
				return ((gfp_t) state_reset);
			}
		}
	}
	if (conf->def_beac_advert_params) {
		if (0 != (ret = cmd_check_n_4hex("GTB", 1, conf->def_beac_advert_params->interval))) {
			if (ret == -EDATA) {
				reboot = TRUE;
				if (CMD_RET_AOK != cmd_n_4hex("STB", 1, conf->def_beac_advert_params->interval)) {
					return ((gfp_t) state_reset);
				}
			} else {
				return ((gfp_t) state_reset);
			}
		}
	}
	if (conf->pins_conf) {
		int i = 0;
		while (conf->pins_conf[i].pin != RN487X_PIN_ARY_END) {
			if (0 != (ret = cmd_gw(conf->pins_conf[i].pin, conf->pins_conf[i].func))) {
				if (ret == -EDATA) {
					reboot = TRUE;
					if (CMD_RET_AOK != cmd_n_2hex("SW", 2, conf->pins_conf[i].pin,
					                              conf->pins_conf[i].func)) {
						return ((gfp_t) state_reset);
					}
				} else {
					return ((gfp_t) state_reset);
				}
			}
			i++;
		}
	}
	if (reboot) {
		msg(INF, "RN# Device configuration was changed\n");
		return ((gfp_t) state_reset);
	} else {
		return ((gfp_t) state_check_services);
	}
}

/**
 * state_check_services
 */
static gfp_t state_check_services(void)
{
	int ret;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> check_services\n");
#endif
        if (0 != (ret = cmd_check_n_2hex("GS", 1, conf->default_serv))) {
		if (ret == -EDATA) {
			goto conf_services;
		} else {
			return ((gfp_t) state_reset);
		}
	}
	if (conf->serv_list) {
		if (0 != (ret = check_srv_services())) {
			if (ret == -EDATA) {
				goto conf_services;
			} else {
				return ((gfp_t) state_reset);
			}
		}
	}
        if (0 != (ret = cmd_check_n_4hex("GDA", 1, conf->gap_appearance))) {
		if (ret == -EDATA) {
			goto conf_services;
		} else {
			return ((gfp_t) state_reset);
		}
	}
	if (conf->default_serv & RN487X_DEFAULT_SERV_DEV_INFO) {
		if (0 != (ret = cmd_check_string("GDF", rnfwv))) {
			if (ret == -EDATA) {
				goto conf_services;
			} else {
				return ((gfp_t) state_reset);
			}
		}
		if (0 != (ret = cmd_check_string("GDM", conf->dis_model))) {
			if (ret == -EDATA) {
				goto conf_services;
			} else {
				return ((gfp_t) state_reset);
			}
		}
		if (0 != (ret = cmd_check_string("GDN", conf->dis_factory))) {
			if (ret == -EDATA) {
				goto conf_services;
			} else {
				return ((gfp_t) state_reset);
			}
		}
		if (0 != (ret = cmd_check_string("GDR", conf->dis_sw_version))) {
			if (ret == -EDATA) {
				goto conf_services;
			} else {
				return ((gfp_t) state_reset);
			}
		}
		if (0 != (ret = cmd_check_string("GDS", conf->dis_serial))) {
			if (ret == -EDATA) {
				goto conf_services;
			} else {
				return ((gfp_t) state_reset);
			}
		}
	}
	return ((gfp_t) state_device_info);
conf_services:
	msg(INF, "RN# Services configuration was changed\n");
	return ((gfp_t) state_conf_services);
}

/**
 * state_conf_services
 */
static gfp_t state_conf_services(void)
{
	struct rn487x_service *serv;
        struct rn487x_characteristic *chr;
        int idx;
	char *cmd_ps = "PS";
        char *cmd_pc = "PC";

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> configure_services\n");
#endif
	if (CMD_RET_AOK != cmd_noarg("PZ")) {
		return ((gfp_t) state_reset);
	}
	if (CMD_RET_AOK != cmd_n_2hex("SS", 1, conf->default_serv)) {
		return ((gfp_t) state_reset);
	}
	if ((serv = conf->serv_list)) {
		do {
			memset(cmdb, 0, RN487X_CMDB_SIZE);
			if (0 > snprintf(cmdb, RN487X_CMDB_SIZE, "%s,%s", cmd_ps, serv->uuid)) {
				crit_err_exit(BAD_PARAMETER);
			}
#if RN487X_LOG_CMD == 1
			log_cmd(cmdb);
#endif
			idx = (serv->uuid_type == RN487X_UUID_TYPE_PUBLIC) ? 7 : 36;
			cmdb[idx] = '\r';
			conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
			if (CMD_RET_AOK == wait_cmd_ret(cmd_ps, CMD_TMO)) {
				if (!cmdp(cmd_ps)) {
					return ((gfp_t) state_reset);
				}
			} else {
				return ((gfp_t) state_reset);
			}
			chr = serv->chr_list;
			do {
				memset(cmdb, 0, RN487X_CMDB_SIZE);
				if (0 > snprintf(cmdb, RN487X_CMDB_SIZE, "%s,%s,%02hhX,%02hhX",
						 cmd_pc, chr->uuid, chr->props, chr->size)) {
					crit_err_exit(BAD_PARAMETER);
				}
#if RN487X_LOG_CMD == 1
				log_cmd(cmdb);
#endif
				idx = (serv->uuid_type == RN487X_UUID_TYPE_PUBLIC) ? 13 : 42;
				cmdb[idx] = '\r';
				conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
				if (CMD_RET_AOK == wait_cmd_ret(cmd_pc, CMD_PC_TMO)) {
					if (!cmdp(cmd_pc)) {
						return ((gfp_t) state_reset);
					}
				} else {
					return ((gfp_t) state_reset);
				}
			} while ((chr = chr->next));
		} while ((serv = serv->next));
	}
	if (CMD_RET_AOK != cmd_n_4hex("SDA", 1, conf->gap_appearance)) {
		return ((gfp_t) state_reset);
	}
        if (conf->default_serv & RN487X_DEFAULT_SERV_DEV_INFO) {
		if (CMD_RET_AOK != cmd_string("SDF", rnfwv)) {
			return ((gfp_t) state_reset);
		}
		if (CMD_RET_AOK != cmd_string("SDM", conf->dis_model)) {
			return ((gfp_t) state_reset);
		}
		if (CMD_RET_AOK != cmd_string("SDN", conf->dis_factory)) {
			return ((gfp_t) state_reset);
		}
		if (CMD_RET_AOK != cmd_string("SDR", conf->dis_sw_version)) {
			return ((gfp_t) state_reset);
		}
		if (CMD_RET_AOK != cmd_string("SDS", conf->dis_serial)) {
			return ((gfp_t) state_reset);
		}
	}
	return ((gfp_t) state_reset);
}

/**
 * state_device_info
 */
static gfp_t state_device_info(void)
{
	enum rx_parser_ret ret;
        char *cmd_ar = "&R";
        int idx;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> device_info\n");
#endif
	memset(&dev_addr, 0, sizeof(dev_addr));
	while (TRUE) {
		memset(cmdb, 0, RN487X_CMDB_SIZE);
		cmdb[0] = 'D';
#if RN487X_LOG_CMD == 1
		log_cmd(cmdb);
#endif
		cmdb[1] = '\r';
		conf->snd_fn(conf->ser_dev, cmdb, 2);
		while (TRUE) {
			ret = rx_parser(CMD_TMO);
			if (ret == RX_PARSER_RET_STRING) {
				if (match_start("BTA=", rxb)) {
					if (!conv_addr_str_bin(rxb + 4, dev_addr.bta)) {
						return ((gfp_t) state_reset);
					}
				} else if (match_start("RAddr=", rxb)) {
					if (!conv_addr_str_bin(rxb + 6, dev_addr.rna)) {
						return ((gfp_t) state_reset);
					}
				}
			} else if (ret == RX_PARSER_RET_CMDP) {
				break;
			} else if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
				continue;
			} else if (ret == RX_PARSER_RET_TMO) {
				msg(INF, "RN# Error: command D timeout\n");
				return ((gfp_t) state_reset);
			} else {
				msg(INF, "RN# Error: state_device_info() rxp_ret=%s\n",
				    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
				return ((gfp_t) state_reset);
			}
		}
		if ((is_bt_addr_inval(dev_addr.bta) && is_bt_addr_inval(dev_addr.rna)) ||
		    (conf->use_rnd_addr && is_bt_addr_inval(dev_addr.rna))) {
			memset(cmdb, 0, RN487X_CMDB_SIZE);
			strcpy(cmdb, cmd_ar);
#if RN487X_LOG_CMD == 1
			log_cmd(cmdb);
#endif
			idx = strlen(cmdb);
			cmdb[idx] = '\r';
			conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
			if (CMD_RET_STR == wait_cmd_ret(cmd_ar, CMD_TMO)) {
#if RN487X_LOG_LEVEL > 0
				msg(INF, "RN# RNA %s assigned\n", rxb);
#endif
				if (!cmdp(cmd_ar)) {
					return ((gfp_t) state_reset);
				}
				continue;
			} else {
				return ((gfp_t) state_reset);
			}
		}
		break;
	}
        msg(INF, "RN# BTA=%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX RNA=%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX\n",
	    dev_addr.bta[0], dev_addr.bta[1], dev_addr.bta[2],
	    dev_addr.bta[3], dev_addr.bta[4], dev_addr.bta[5],
	    dev_addr.rna[0], dev_addr.rna[1], dev_addr.rna[2],
	    dev_addr.rna[3], dev_addr.rna[4], dev_addr.rna[5]);
#if TERMOUT == 1
	log_def_params();
#endif
	return ((gfp_t) state_set_advert_data);
}

/**
 * state_set_advert_data
 */
static gfp_t state_set_advert_data(void)
{
	int ep;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> set_advert_data\n");
#endif
	if (conf->advert_list) {
		if (!set_adv_payload("IA", conf->advert_list)) {
			return ((gfp_t) state_reset);
		}
		msg(INF, "RN# Advertising packet payload size=%d bytes\n", adv_payload_size(conf->advert_list));
	}
	if (conf->scan_resp_list) {
		if (!set_adv_payload("IS", conf->scan_resp_list)) {
			return ((gfp_t) state_reset);
		}
		msg(INF, "RN# Scan response packet payload size=%d bytes\n", adv_payload_size(conf->scan_resp_list));
	}
	if (conf->beacon_list) {
		if (!set_adv_payload("IB", conf->beacon_list)) {
			return ((gfp_t) state_reset);
		}
		msg(INF, "RN# Beacon packet payload size=%d bytes\n", adv_payload_size(conf->beacon_list));
	}
	if (conf->advert_style == RN487X_ADVERT_STYLE_BEACON) {
		ep = 0;
	} else {
		ep = 1;
	}
	if (CMD_RET_AOK != cmd_1hex("E", ep)) {
		return ((gfp_t) state_reset);
	}
	if (clear_status_fifo("after configuration")) {
		return ((gfp_t) state_reset);
	}
	msg(INF, "RN# Init done\n");
	return ((gfp_t) state_advertise);
}

/**
 * state_advertise
 */
static gfp_t state_advertise(void)
{
	union status st;
	enum rx_parser_ret ret;
        struct rn487x_characteristic *chr;
        p_stf_t stf;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> advertise\n");
#endif
	if (conf->advert_params) {
		if (CMD_RET_AOK != cmd_n_4hex("A", 1, conf->advert_params->interval)) {
			return ((gfp_t) state_reset);
		}
	} else {
		if (!conf->def_advert_params) {
			msg(INF, "RN# Error: def_advert_params not defined\n");
			crit_err_exit(BAD_PARAMETER);
		}
		if (CMD_RET_AOK != cmd_noarg("A")) {
			return ((gfp_t) state_reset);
		}
	}
#if RN487X_LOG_LEVEL > 0
        msg(INF, "RN# Advertising\n");
#endif
	while (TRUE) {
		while (pdTRUE == xQueueReceive(status_fifo, &st, 0)) {
			if (st.common.id == STATUS_ID_CONNECT) {
                                connected = TRUE;
				snd_stm_evnt(RN487X_EVENT_CONNECTED);
				peer = st.connect;
				return ((gfp_t) state_connected);
			} else if (st.common.id == STATUS_ID_REBOOT) {
				msg(INF, "RN# Error: state_advertise() unexpected reboot\n");
				return ((gfp_t) state_reset);
			} else {
				msg(INF, "RN# state_advertise() unparsed status %s\n",
				    find_txt_item(st.common.id, status_id_desc_ary, "IDX_ERR"));
				stats.unpar_adv_status++;
			}
		}
		if (NULL != (chr = gatt_update_pend())) {
			if (!cmd_shw(chr)) {
				return ((gfp_t) state_reset);
			}
			continue;
		}
                if (NULL != (stf = cmd_handler(state_advertise))) {
			if (stf != state_advertise) {
				return ((gfp_t) stf);
			}
		}
		ret = rx_parser(RN487X_CHECK_ADVERTISE_MS / portTICK_PERIOD_MS);
		if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
			if (0 == cmd_check_string("GK", "none")) {
				continue;
			} else {
				return ((gfp_t) state_reset);
			}
		} else {
			if (ret == RX_PARSER_RET_STRING && match_start("Rebooting", rxb)) {
				continue;
			}
			msg(INF, "RN# Error: state_advertise() rxp_ret=%s\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
			stats.adv_rxp_err++;
                        return ((gfp_t) state_reset);
		}
	}
}

/**
 * state_connected
 */
static gfp_t state_connected(void)
{
	union status st;
	enum rx_parser_ret ret;
        struct rn487x_characteristic *chr;
	p_stf_t stf;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> connected\n");
#endif
	conn_secured = FALSE;
#if RN487X_LOG_LEVEL > 0
        msg(INF, "RN# Connected to %s address %02hhX%02hhX%02hhX%02hhX%02hhX%02hhX\n",
	    (peer.addr_type == RN487X_ADDR_TYPE_PUBLIC) ? "public" : "random",
	    peer.addr[0], peer.addr[1], peer.addr[2],
	    peer.addr[3], peer.addr[4], peer.addr[5]);
#endif
	if (conf->req_sec_link) {
		enum cmd_ret r = cmd_noarg("B");
		if (r == CMD_RET_ERR) {
			return ((gfp_t) state_disconnect);
		} else if (r != CMD_RET_AOK) {
			return ((gfp_t) state_reset);
		}
	}
	while (TRUE) {
		while (pdTRUE == xQueueReceive(status_fifo, &st, 0)) {
			if (st.common.id == STATUS_ID_SECURED) {
				conn_secured = TRUE;
				snd_stm_evnt(RN487X_EVENT_CONN_SECURED);
#if RN487X_LOG_LEVEL > 0
				msg(INF, "RN# Connection secured\n");
#endif
			} else if (st.common.id == STATUS_ID_BONDED) {
				snd_stm_evnt(RN487X_EVENT_BONDED);
#if RN487X_LOG_LEVEL > 0
				msg(INF, "RN# Bonded\n");
#endif
			} else if (st.common.id == STATUS_ID_WC) {
#if TERMOUT == 1 && RN487X_LOG_LEVEL > 0
				log_notif_req(&st.wc);
#endif
			} else if (st.common.id == STATUS_ID_WV) {
				;
			} else if (st.common.id == STATUS_ID_DISCONNECT) {
				if (CMD_RET_AOK != cmd_noarg("Y")) {
					return ((gfp_t) state_reset);
				}
                                connected = FALSE;
                                snd_stm_evnt(RN487X_EVENT_DISCONNECTED);
#if RN487X_LOG_LEVEL > 0
                                msg(INF, "RN# Disconnected\n");
#endif
				return ((gfp_t) state_advertise);
			} else if (st.common.id == STATUS_ID_ERR_SEC) {
				snd_stm_evnt(RN487X_EVENT_CONN_SEC_ERR);
#if RN487X_LOG_LEVEL > 0
				msg(INF, "RN# Connection security error\n");
#endif
                                return ((gfp_t) state_con_sec_err);
			} else if (st.common.id == STATUS_ID_KEY) {
				struct rn487x_event event;
				memset(&event, 0, sizeof(struct rn487x_event));
				event.event_type = RN487X_EVENT_SECURITY_KEY;
				strcpy(event.param.key, st.key.key);
				if (errQUEUE_FULL == xQueueSend(stm_evnt_que, &event, 0)) {
					stats.stm_evnt_que_full++;
					msg(INF, "RN# Error: stm_evnt_que full\n");
				}
			} else if (st.common.id == STATUS_ID_REBOOT) {
				msg(INF, "RN# Error: state_connected() unexpected reboot\n");
				return ((gfp_t) state_reset);
			} else {
				stats.unpar_con_status++;
				msg(INF, "RN# state_connected() unparsed status %s\n",
				    find_txt_item(st.common.id, status_id_desc_ary, "IDX_ERR"));
			}
		}
		if (NULL != (chr = gatt_update_pend())) {
			if (!cmd_shw(chr)) {
				return ((gfp_t) state_reset);
			}
			continue;
		}
                if (NULL != (stf = cmd_handler(state_connected))) {
			if (stf != state_connected) {
				return ((gfp_t) stf);
			}
		}
		ret = rx_parser(portMAX_DELAY);
		if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else {
			if (ret == RX_PARSER_RET_STRING && match_start("Rebooting", rxb)) {
				continue;
			}
			stats.con_rxp_err++;
			msg(INF, "RN# Error: state_connected() rxp_ret=%s\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
                        return ((gfp_t) state_reset);
		}
	}
}

/**
 * state_con_sec_err
 */
static gfp_t state_con_sec_err(void)
{
	union status st;
        enum rx_parser_ret ret;
	boolean_t disc = 0;
        TickType_t tmo = CON_SEC_ERR_D_TMO;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> con_sec_err\n");
#endif
	while (TRUE) {
		while (pdTRUE == xQueueReceive(status_fifo, &st, 0)) {
			if (st.common.id == STATUS_ID_DISCONNECT) {
				if (++disc == 1) {
					if (CMD_RET_AOK != cmd_noarg("Y")) {
						return ((gfp_t) state_reset);
					}
					tmo = CON_SEC_ERR_Y_TMO;
                                        connected = FALSE;
                                        snd_stm_evnt(RN487X_EVENT_DISCONNECTED);
#if RN487X_LOG_LEVEL > 0
					msg(INF, "RN# Disconnected\n");
#endif
				}
			} else if (st.common.id == STATUS_ID_REBOOT) {
				msg(INF, "RN# Error: state_con_sec_err() unexpected reboot\n");
				return ((gfp_t) state_reset);
			} else {
				msg(INF, "RN# state_con_sec_err() unparsed status %s\n",
				    find_txt_item(st.common.id, status_id_desc_ary, "IDX_ERR"));
				stats.unpar_cse_status++;
			}
		}
		ret = rx_parser(tmo);
		if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
			if (disc) {
				return ((gfp_t) state_advertise);
			} else {
				return ((gfp_t) state_disconnect);
			}
		} else {
			if (ret == RX_PARSER_RET_STRING && match_start("Rebooting", rxb)) {
				continue;
			}
			msg(INF, "RN# Error: state_con_sec_err() rxp_ret=%s\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
			stats.cse_rxp_err++;
                        return ((gfp_t) state_reset);
		}
	}
}

/**
 * state_disconnect
 */
static gfp_t state_disconnect(void)
{
	union status st;
	enum cmd_ret cmd_ret;
        enum rx_parser_ret ret;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> disconnect\n");
#endif
	if (CMD_RET_AOK != (cmd_ret = cmd_noarg("K,1")) && cmd_ret != CMD_RET_ERR) {
		return ((gfp_t) state_reset);
	}
	while (TRUE) {
		while (pdTRUE == xQueueReceive(status_fifo, &st, 0)) {
			if (st.common.id == STATUS_ID_DISCONNECT) {
				if (CMD_RET_AOK != cmd_noarg("Y")) {
					return ((gfp_t) state_reset);
				}
                                connected = FALSE;
                                snd_stm_evnt(RN487X_EVENT_DISCONNECTED);
#if RN487X_LOG_LEVEL > 0
				msg(INF, "RN# Disconnected\n");
#endif
				return ((gfp_t) state_advertise);
			} else if (st.common.id == STATUS_ID_REBOOT) {
				msg(INF, "RN# Error: state_disconnect() unexpected reboot\n");
				return ((gfp_t) state_reset);
			} else {
				msg(INF, "RN# state_disconnect() unparsed status %s\n",
				    find_txt_item(st.common.id, status_id_desc_ary, "IDX_ERR"));
				stats.unpar_dis_status++;
			}
		}
		ret = rx_parser(DISCON_TMO);
		if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
			msg(INF, "RN# Error: disconnect failed\n");
			return ((gfp_t) state_reset);
		} else {
			if (ret == RX_PARSER_RET_STRING && match_start("Rebooting", rxb)) {
				continue;
			}
			msg(INF, "RN# Error: state_disconnect() rxp_ret=%s\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
			stats.dis_rxp_err++;
                        return ((gfp_t) state_reset);
		}
	}
}

/**
 * state_set_pub_bta
 */
static gfp_t state_set_pub_bta(void)
{
	char bta[13];

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> set_pub_bta\n");
#endif
	if (0 > snprintf(bta, 13, "%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX",
	                 rn487x_cmd_act.set_pub_addr.bta[5], rn487x_cmd_act.set_pub_addr.bta[4],
			 rn487x_cmd_act.set_pub_addr.bta[3], rn487x_cmd_act.set_pub_addr.bta[2],
			 rn487x_cmd_act.set_pub_addr.bta[1], rn487x_cmd_act.set_pub_addr.bta[0])) {
		crit_err_exit(BAD_PARAMETER);
	}
        memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
	if (CMD_RET_AOK == cmd_string("S:,0000", bta)) {
		snd_stm_evnt(RN487X_EVENT_CMD_OK);
		msg(INF, "RN# New public BTA assigned\n");
	} else {
		snd_stm_evnt(RN487X_EVENT_CMD_ERR);
	}
	return ((gfp_t) state_reset);
}

/**
 * state_set_fact_def
 */
static gfp_t state_set_fact_def(void)
{
	int idx;
        enum rx_parser_ret ret;
	union status st;
	boolean_t rbt = FALSE;

#if RN487X_LOG_LEVEL > 1
	msg(INF, "RN# STATE> set_fact_def\n");
#endif
        memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, "SF,2");
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	while (TRUE) {
		ret = rx_parser(CMD_FACT_DEF_TMO);
		if (ret == RX_PARSER_RET_STRING && match_start("Reboot after Factory Reset", rxb)) {
			snd_stm_evnt(RN487X_EVENT_CMD_OK);
			rbt = TRUE;
                        msg(INF, "RN# Factory default configuration applied\n");
		} else if (ret == RX_PARSER_RET_TMO) {
			if (!rbt) {
				snd_stm_evnt(RN487X_EVENT_CMD_ERR);
			}
			break;
		}
	}
	while (pdTRUE == xQueueReceive(status_fifo, &st, 0));
	return ((gfp_t) state_reset);
}

/**
 * set_adv_payload
 */
static boolean_t set_adv_payload(const char *cmd, const struct rn487x_ad_record *ad)
{
	int idx;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	if (0 > snprintf(cmdb, RN487X_CMDB_SIZE, "%s,Z", cmd)) {
		crit_err_exit(BAD_PARAMETER);
	}
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	if (CMD_RET_AOK == wait_cmd_ret(cmd, CMD_TMO)) {
		if (!cmdp(cmd)) {
			return (FALSE);
		}
	} else {
		return (FALSE);
	}
	do {
		memset(cmdb, 0, RN487X_CMDB_SIZE);
		if (0 > snprintf(cmdb, RN487X_CMDB_SIZE, "%s,%02hhX,%s", cmd, ad->type, ad->data)) {
			crit_err_exit(BAD_PARAMETER);
		}
#if RN487X_LOG_CMD == 1
		log_cmd(cmdb);
#endif
		idx = strlen(cmdb);
		cmdb[idx] = '\r';
		conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
		if (CMD_RET_AOK == wait_cmd_ret(cmd, CMD_TMO)) {
			if (!cmdp(cmd)) {
				return (FALSE);
			}
		} else {
			return (FALSE);
		}
	} while ((ad = ad->next));
	return (TRUE);
}

#if TERMOUT == 1
/**
 * adv_payload_size
 */
static int adv_payload_size(const struct rn487x_ad_record *ad)
{
	int sz = 0;

	do {
		sz += strlen(ad->data) / 2;
		sz += 2;
	} while ((ad = ad->next));
	return (sz);
}
#endif

/**
 * check_srv_services
 */
static int check_srv_services(void)
{
	int idx;
	enum rx_parser_ret ret;
	UBaseType_t prio;
	struct rn487x_service *srv;
	struct rn487x_characteristic *chr;
        int fn_ret = 0;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, "LS");
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	prio = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, TASK_PRIO_HIGH);
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	while (TRUE) {
		ret = rx_parser(CMD_TMO);
		if (ret == RX_PARSER_RET_STRING) {
			int r;
			if (0 != (r = services_parser(rxb))) {
				if (fn_ret != -EFMT) {
					fn_ret = r;
				}
			}
		} else if (ret == RX_PARSER_RET_CMDP) {
			break;
		} else if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
			msg(INF, "RN# Error: command LS timeout\n");
                        fn_ret = -ETMO;
			break;
		} else {
			msg(INF, "RN# Error: check_srv_services() rxp_ret=%s\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
			fn_ret = -EFMT;
			break;
		}
	}
        vTaskPrioritySet(NULL, prio);
	if (fn_ret == 0) {
		srv = conf->serv_list;
		do {
			chr = srv->chr_list;
			do {
				if (!chr->handle) {
					fn_ret = -EDATA;
				} else {
					uint8_t props;
					if (chr->cccd_handle) {
						props = chr->eepr_props | chr->cccd_props;
					} else {
						props = chr->eepr_props;
					}
					if (chr->props != props) {
						fn_ret = -EDATA;
					}
				}
			} while ((chr = chr->next));
		} while ((srv = srv->next));
	}
        return (fn_ret);
}

/**
 * services_parser
 */
static int services_parser(char *row)
{
	static struct rn487x_service *act_srv;
	struct rn487x_service *srv;
	struct rn487x_characteristic *chr;
	boolean_t uu16 = TRUE;
	int cm_n;
#if TERMOUT == 1
        const char *err_fmt = "RN# Error: services_parser() %s\n";
#endif
	if (row[0] != ' ') {
		srv = conf->serv_list;
		do {
			if (0 == strcmp(row, srv->uuid)) {
				act_srv = srv;
				return (0);
			}
		} while ((srv = srv->next));
		if (0 == strcmp(row, "END")) {
			return (0);
		} else {
			return (-EDATA);
		}
	} else {
		chr = act_srv->chr_list;
		boolean_t fst = TRUE;
                cm_n = 0;
		for (int i = 2; ; i++) {
			if (row[i] == ',') {
				row[i] = '\0';
				cm_n++;
				if (fst == TRUE) {
					fst = FALSE;
					if (i == 6) {
						uu16 = TRUE;
					} else if (i == 34) {
						uu16 = FALSE;
					} else {
						msg(INF, err_fmt, "bad 1st comma position");
						return (-EFMT);
					}
				}
			} else if (row[i] == '\0') {
				break;
			}
		}
		do {
			if (0 == strcmp(row + 2, chr->uuid)) {
				uint16_t handle;
				uint8_t props;
				char *p;
				if (uu16) {
					p = row + 7;
				} else {
					p = row + 35;
				}
				if (4 != strlen(p)) {
					msg(INF, err_fmt, "bad handle size");
					return (-EFMT);
				}
				if (1 != sscanf(p, "%4hx", &handle)) {
					stats.sscanf_err++;
					msg(INF, err_fmt, "sscanf failed");
					return (-EFMT);
				}
				if (uu16) {
					p = row + 12;
				} else {
					p = row + 40;
				}
				if (2 != strlen(p)) {
					msg(INF, err_fmt, "bad props size");
					return (-EFMT);
				}
				if (1 != sscanf(p, "%2hhx", &props)) {
					stats.sscanf_err++;
					msg(INF, err_fmt, "sscanf failed");
                                        return (-EFMT);
				}
				if (cm_n == 2) {
					chr->handle = handle;
					chr->eepr_props = props;
				} else if (cm_n == 3) {
					chr->cccd_handle = handle;
					chr->cccd_props = props;
				} else {
					msg(INF, err_fmt, "bad comma count");
					return (-EFMT);
				}
				return (0);
			}
		} while ((chr = chr->next));
		return (-EDATA);
	}
}

/**
 * gatt_update_pend
 */
static struct rn487x_characteristic *gatt_update_pend(void)
{
	static struct rn487x_service *srv;
	static struct rn487x_characteristic *last_chr;
	struct rn487x_characteristic *chr;

	if (conf->serv_list == NULL) {
		return (NULL);
	}
	if (srv == NULL) {
		srv = conf->serv_list;
		last_chr = srv->chr_list;
	}
	chr = last_chr;
	for (;;) {
		if (chr->next) {
			chr = chr->next;
		} else {
			if (srv->next) {
				srv = srv->next;
			} else {
				srv = conf->serv_list;
			}
                        chr = srv->chr_list;
		}
		if (0 == uxQueueSpacesAvailable(chr->update)) {
			last_chr = chr;
			return (chr);
		}
		if (chr == last_chr) {
			return (NULL);
		}
	}
}

/**
 * cmd_shw
 */
static boolean_t cmd_shw(const struct rn487x_characteristic *chr)
{
	int idx;
	enum rx_parser_ret ret;
	uint8_t data[chr->size];
        boolean_t fn_ret = FALSE;

	xQueueReceive(chr->update, data, 0);
	if (9 + chr->size * 2 + 1 > RN487X_CMDB_SIZE) {
		msg(INF, "RN# Error: cmd_shw() command string not fit in cmdb\n");
		crit_err_exit(UNEXP_PROG_STATE);
	}
	memset(cmdb, 0, RN487X_CMDB_SIZE);
	if (0 > snprintf(cmdb, RN487X_CMDB_SIZE, "SHW,%04hX,", chr->handle)) {
		crit_err_exit(BAD_PARAMETER);
	}
	for (int i = 0; i < chr->size; i++) {
		if (0 > sprintf(cmdb + 9 + 2 * i, "%02hhX", *(data + i))) {
			crit_err_exit(BAD_PARAMETER);
		}
	}
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	while (TRUE) {
		ret = rx_parser(CMD_SHW_TMO);
		if (ret == RX_PARSER_RET_CMD_AOK) {
			fn_ret = TRUE;
		} else if (ret == RX_PARSER_RET_CMD_ERR) {
			fn_ret = FALSE;
		} else if (ret == RX_PARSER_RET_CMDP) {
			return (fn_ret);
		} else if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
			msg(INF, "RN# Error: command SHW timeout\n");
			return (FALSE);
		} else {
              		if (ret == RX_PARSER_RET_STRING && match_start("NFail", rxb)) {
				fn_ret = TRUE;
                                stats.nfail++;
				continue;
			}
			msg(INF, "RN# Error: cmd_shw() rxp_ret=%s\n", find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
			return (FALSE);
		}
	}
}

/**
 * cmd_string
 */
static enum cmd_ret cmd_string(const char *cmd, const char *str)
{
	int idx;
        enum cmd_ret ret;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	if (0 > snprintf(cmdb, RN487X_CMDB_SIZE, "%s,%s", cmd, str)) {
		crit_err_exit(BAD_PARAMETER);
	}
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
        ret = wait_cmd_ret(cmd, CMD_TMO);
	if (ret == CMD_RET_AOK || ret == CMD_RET_ERR) {
		if (cmdp(cmd)) {
			return (ret);
		} else {
			return (CMD_RET_FAIL);
		}
	} else {
		return (CMD_RET_FAIL);
	}
}

/**
 * cmd_check_string
 */
static int cmd_check_string(const char *cmd, const char *str)
{
	int ret = 0;
	int idx;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, cmd);
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	if (CMD_RET_STR == wait_cmd_ret(cmd, CMD_TMO)) {
		if (0 != strcmp(rxb, str)) {
			ret = -EDATA;
		}
		if (!cmdp(cmd)) {
			return (-ERCV);
		}
                return (ret);
	} else {
		return (-ERCV);
	}
}

/**
 * cmd_1hex
 */
static enum cmd_ret cmd_1hex(const char *cmd, uint8_t n)
{
	int idx;
        enum cmd_ret ret;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	if (0 > snprintf(cmdb, RN487X_CMDB_SIZE, "%s,%1hhX", cmd, n)) {
		crit_err_exit(BAD_PARAMETER);
	}
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	ret = wait_cmd_ret(cmd, CMD_TMO);
	if (ret == CMD_RET_AOK || ret == CMD_RET_ERR) {
		if (cmdp(cmd)) {
			return (ret);
		} else {
			return (CMD_RET_FAIL);
		}
	} else {
		return (CMD_RET_FAIL);
	}
}

/**
 * cmd_check_1hex
 */
static int cmd_check_1hex(const char *cmd, uint8_t n)
{
	int ret = 0;
	int idx;
        uint8_t u8;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, cmd);
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	if (CMD_RET_STR == wait_cmd_ret(cmd, CMD_TMO)) {
		if (1 == sscanf(rxb, "%1hhx", &u8)) {
			if (u8 != n) {
				ret = -EDATA;
			}
		} else {
			msg(INF, "RN# Error: cmd_check_1hex() sscanf failed\n");
			stats.sscanf_err++;
			ret = -EFMT;
		}
		if (!cmdp(cmd)) {
                        return (-ERCV);
		}
                return (ret);
	} else {
                return (-ERCV);
	}
}

/**
 * cmd_n_2hex
 */
static enum cmd_ret cmd_n_2hex(const char *cmd, int par_num, ...)
{
	int idx;
	char fmt[strlen(cmd) + 7 * par_num + 1];
        va_list ap;
        enum cmd_ret ret;

	idx = 0;
	while (*(cmd + idx) != '\0') {
		fmt[idx] = *(cmd + idx);
		idx++;
	}
	for (int i = 0; i < par_num; i++) {
		fmt[idx++] = ',';
                fmt[idx++] = '%';
                fmt[idx++] = '0';
                fmt[idx++] = '2';
                fmt[idx++] = 'h';
                fmt[idx++] = 'h';
                fmt[idx++] = 'X';
	}
	fmt[idx] = '\0';
	memset(cmdb, 0, RN487X_CMDB_SIZE);
        va_start(ap, par_num);
	if (0 > vsnprintf(cmdb, RN487X_CMDB_SIZE, fmt, ap)) {
		va_end(ap);
		crit_err_exit(BAD_PARAMETER);
	}
        va_end(ap);
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	ret = wait_cmd_ret(cmd, CMD_TMO);
	if (ret == CMD_RET_AOK || ret == CMD_RET_ERR) {
		if (cmdp(cmd)) {
			return (ret);
		} else {
			return (CMD_RET_FAIL);
		}
	} else {
		return (CMD_RET_FAIL);
	}
}

/**
 * cmd_check_n_2hex
 */
static int cmd_check_n_2hex(const char *cmd, int par_num, ...)
{
	int ret = 0;
	int idx;
        uint8_t u8, par;
        va_list ap;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, cmd);
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	if (CMD_RET_STR == wait_cmd_ret(cmd, CMD_TMO)) {
		if ((int) strlen(rxb) == (par_num * 2) + par_num - 1) {
			va_start(ap, par_num);
			idx = 0;
			for (int i = 0; i < par_num; i++) {
				par = va_arg(ap, int);
				if (1 == sscanf(rxb + idx, "%2hhx", &u8)) {
					if (u8 != par) {
						ret = -EDATA;
						break;
					}
				} else {
					msg(INF, "RN# Error: cmd_check_n_2hex() sscanf failed\n");
					stats.sscanf_err++;
					ret = -EFMT;
					break;
				}
				idx += 3;
			}
                        va_end(ap);
		} else {
			ret = -EFMT;
		}
		if (!cmdp(cmd)) {
                        return (-ERCV);
		}
                return (ret);
	} else {
                return (-ERCV);
	}
}

/**
 * cmd_n_4hex
 */
static enum cmd_ret cmd_n_4hex(const char *cmd, int par_num, ...)
{
	int idx;
	char fmt[strlen(cmd) + 6 * par_num + 1];
        va_list ap;
	enum cmd_ret ret;

	idx = 0;
	while (*(cmd + idx) != '\0') {
		fmt[idx] = *(cmd + idx);
		idx++;
	}
	for (int i = 0; i < par_num; i++) {
		fmt[idx++] = ',';
                fmt[idx++] = '%';
                fmt[idx++] = '0';
                fmt[idx++] = '4';
                fmt[idx++] = 'h';
                fmt[idx++] = 'X';
	}
	fmt[idx] = '\0';
	memset(cmdb, 0, RN487X_CMDB_SIZE);
        va_start(ap, par_num);
	if (0 > vsnprintf(cmdb, RN487X_CMDB_SIZE, fmt, ap)) {
		va_end(ap);
		crit_err_exit(BAD_PARAMETER);
	}
        va_end(ap);
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	ret = wait_cmd_ret(cmd, CMD_TMO);
	if (ret == CMD_RET_AOK || ret == CMD_RET_ERR) {
		if (cmdp(cmd)) {
			return (ret);
		} else {
			return (CMD_RET_FAIL);
		}
	} else {
		return (CMD_RET_FAIL);
	}
}

/**
 * cmd_check_n_4hex
 */
static int cmd_check_n_4hex(const char *cmd, int par_num, ...)
{
	int ret = 0;
	int idx;
        uint16_t u16, par;
        va_list ap;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, cmd);
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	if (CMD_RET_STR == wait_cmd_ret(cmd, CMD_TMO)) {
		if ((int) strlen(rxb) == (par_num * 4) + par_num - 1) {
			va_start(ap, par_num);
			idx = 0;
			for (int i = 0; i < par_num; i++) {
				par = va_arg(ap, int);
				if (1 == sscanf(rxb + idx, "%4hx", &u16)) {
					if (u16 != par) {
						ret = -EDATA;
						break;
					}
				} else {
					msg(INF, "RN# Error: cmd_check_n_4hex() sscanf failed\n");
					stats.sscanf_err++;
					ret = -EFMT;
					break;
				}
				idx += 5;
			}
                        va_end(ap);
		} else {
			ret = -EFMT;
		}
		if (!cmdp(cmd)) {
                        return (-ERCV);
		}
		return (ret);
	} else {
                return (-ERCV);
	}
}

/**
 * cmd_noarg
 */
static enum cmd_ret cmd_noarg(const char *cmd)
{
	int idx;
	enum cmd_ret ret;

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, cmd);
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	ret = wait_cmd_ret(cmd, CMD_TMO);
	if (ret == CMD_RET_AOK || ret == CMD_RET_ERR) {
		if (cmdp(cmd)) {
			return (ret);
		} else {
			return (CMD_RET_FAIL);
		}
	} else {
		return (CMD_RET_FAIL);
	}
}

/**
 * cmd_gw
 */
static int cmd_gw(uint8_t pin, uint8_t func)
{
	int ret = 0;
	int idx;
        uint8_t u8;
	char *cmd = "GW";

	memset(cmdb, 0, RN487X_CMDB_SIZE);
	if (0 > snprintf(cmdb, RN487X_CMDB_SIZE, "%s,%02hhX", cmd, pin)) {
		crit_err_exit(BAD_PARAMETER);
	}
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	if (CMD_RET_STR == wait_cmd_ret(cmd, CMD_TMO)) {
		if (strlen(rxb) == 2) {
			if (1 == sscanf(rxb, "%2hhx", &u8)) {
				if (u8 != func) {
					ret = -EDATA;
				}
			} else {
				msg(INF, "RN# Error: cmd_gw() sscanf failed\n");
				stats.sscanf_err++;
				ret = -EFMT;
			}
		} else {
			ret = -EFMT;
		}
		if (!cmdp(cmd)) {
			return (-ERCV);
		}
		return (ret);
	} else {
		return (-ERCV);
	}
}

/**
 * get_fw_version
 */
static boolean_t get_fw_version(void)
{
	char *cmd = "V";

	memset(cmdb, 0, RN487X_CMDB_SIZE);
        strcpy(cmdb, cmd);
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	cmdb[1] = '\r';
	conf->snd_fn(conf->ser_dev, cmdb, 2);
	if (CMD_RET_STR == wait_cmd_ret(cmd, CMD_TMO)) {
		int j = 0;
		boolean_t v_st = FALSE;
		for (int i = 0; rxb[i] != '\0'; i++) {
			if (!v_st) {
				if (rxb[i] == 'V' || rxb[i] == 'v') {
					v_st = TRUE;
				}
			} else {
				if (rxb[i] == '.' || isdigit(rxb[i])) {
					rnfwv[j++ + 5] = rxb[i];
					if (j + 5 == RNFWV_ARY_SIZE - 1) {
						msg(INF, "RN# Error: get_fw_version() v_string too long\n");
						break;
					}
				} else {
					msg(INF, "RN# %s\n", rnfwv);
					break;
				}
			}
		}
		if (!v_st) {
			msg(INF, "RN# Error: get_fw_version() v_string not found\n");
		}
		if (!cmdp(cmd)) {
			return (FALSE);
		}
		return (TRUE);
	} else {
		return (FALSE);
	}
}

/**
 * read_all_bond
 */
static boolean_t read_all_bond(struct rn487x_bond_tbl_row (*btbl)[8])
{
	int idx;
	enum rx_parser_ret ret;
	UBaseType_t prio;
        boolean_t fn_ret;
        uint8_t bta[6];
	enum rn487x_addr_type atype;
	int a_idx;
#if TERMOUT == 1
        const char *err_fmt = "RN# Error: command LB %s\n";
#endif
	if (btbl) {
		memset(btbl, 0, sizeof(*btbl));
	}
	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, "LB");
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	prio = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, TASK_PRIO_HIGH);
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	while (TRUE) {
		ret = rx_parser(CMD_TMO);
		if (ret == RX_PARSER_RET_STRING) {
			if (match_start("END", rxb)) {
				continue;
			}
			if (strlen(rxb) == 16 && rxb[1] == ',' && rxb[14] == ',') {
				if (!conv_addr_str_bin(rxb + 2, bta)) {
					fn_ret = FALSE;
					msg(INF, err_fmt, "bad address format");
					break;
				}
				if (rxb[15] == '0') {
					atype = RN487X_ADDR_TYPE_PUBLIC;
				} else if (rxb[15] == '1') {
					atype = RN487X_ADDR_TYPE_RANDOM;
				} else {
                                        fn_ret = FALSE;
					msg(INF, err_fmt, "bad address type format");
					break;
				}
				a_idx = rxb[0] - 0x30;
				if (a_idx < 1 || a_idx > 8) {
                                        fn_ret = FALSE;
					msg(INF, err_fmt, "bad row index format");
					break;
				}
                                if (btbl) {
					((struct rn487x_bond_tbl_row *) btbl)[a_idx - 1].idx = a_idx;
                                        ((struct rn487x_bond_tbl_row *) btbl)[a_idx - 1].atype = atype;
					for (int i = 0; i < 6; i++) {
						((struct rn487x_bond_tbl_row *) btbl)[a_idx - 1].addr[i] = bta[i];
					}
				}
				msg(INF, "RN# Bond[%d] %02hhX%02hhX%02hhX%02hhX%02hhX%02hhX %s\n",
				    a_idx,
				    bta[0], bta[1], bta[2], bta[3], bta[4], bta[5],
				    (atype == RN487X_ADDR_TYPE_PUBLIC) ? "PUB" : "RND");
			} else {
				fn_ret = FALSE;
                                msg(INF, err_fmt, "bad format");
				break;
			}
		} else if (ret == RX_PARSER_RET_CMDP) {
			fn_ret = TRUE;
			break;
		} else if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
                        fn_ret = FALSE;
			msg(INF, err_fmt, "timeout");
			break;
		} else {
			fn_ret = FALSE;
			msg(INF, "RN# Error: read_all_bond() rxp_ret=%s\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
			break;
		}
	}
	vTaskPrioritySet(NULL, prio);
	return (fn_ret);
}

/**
 * read_whl
 */
static boolean_t read_whl(struct rn487x_whl_row (*whl)[16])
{
	int idx;
	enum rx_parser_ret ret;
	UBaseType_t prio;
        boolean_t fn_ret;
        uint8_t bta[6];
	enum rn487x_addr_type atype;
	int a_idx = 1;
#if TERMOUT == 1
        const char *err_fmt = "RN# Error: command JD %s\n";
#endif
	if (whl) {
		memset(whl, 0, sizeof(*whl));
	}
	memset(cmdb, 0, RN487X_CMDB_SIZE);
	strcpy(cmdb, "JD");
#if RN487X_LOG_CMD == 1
	log_cmd(cmdb);
#endif
	idx = strlen(cmdb);
	cmdb[idx] = '\r';
	prio = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, TASK_PRIO_HIGH);
	conf->snd_fn(conf->ser_dev, cmdb, idx + 1);
	while (TRUE) {
		ret = rx_parser(CMD_TMO);
		if (ret == RX_PARSER_RET_STRING) {
			if (match_start("END", rxb)) {
				continue;
			}
			if (strlen(rxb) == 14 && rxb[12] == ',') {
				if (!conv_addr_str_bin(rxb, bta)) {
					fn_ret = FALSE;
					msg(INF, err_fmt, "bad address format");
					break;
				}
				if (rxb[13] == '0') {
					atype = RN487X_ADDR_TYPE_PUBLIC;
				} else if (rxb[13] == '1') {
					atype = RN487X_ADDR_TYPE_RANDOM;
				} else {
                                        fn_ret = FALSE;
					msg(INF, err_fmt, "bad address type format");
					break;
				}
                                if (whl) {
                                        ((struct rn487x_whl_row *) whl)[a_idx - 1].atype = atype;
					for (int i = 0; i < 6; i++) {
						((struct rn487x_whl_row *) whl)[a_idx - 1].addr[i] = bta[i];
					}
				}
				msg(INF, "RN# Whl[%d] %02hhX%02hhX%02hhX%02hhX%02hhX%02hhX %s\n",
				    a_idx,
				    bta[0], bta[1], bta[2], bta[3], bta[4], bta[5],
				    (atype == RN487X_ADDR_TYPE_PUBLIC) ? "PUB" : "RND");
				a_idx++;
			} else {
				fn_ret = FALSE;
                                msg(INF, err_fmt, "bad format");
				break;
			}
		} else if (ret == RX_PARSER_RET_CMDP) {
			fn_ret = TRUE;
			break;
		} else if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
                        fn_ret = FALSE;
			msg(INF, err_fmt, "timeout");
			break;
		} else {
			fn_ret = FALSE;
			msg(INF, "RN# Error: read_whl() rxp_ret=%s\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"));
			break;
		}
	}
	vTaskPrioritySet(NULL, prio);
	return (fn_ret);
}

/**
 * cmdp
 */
static boolean_t cmdp(const char *cmd)
{
	enum rx_parser_ret ret;

	while (TRUE) {
		ret = rx_parser(PROMPT_TMO);
		if (ret == RX_PARSER_RET_CMDP) {
			return (TRUE);
		} else if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
			msg(INF, "RN# Error: 'CMD>' not received (command %s)\n", cmd);
			return (FALSE);
		} else {
			msg(INF, "RN# Error: cmdp() rxp_ret=%s (command %s)\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"), cmd);
			return (FALSE);
		}
	}
}

/**
 * wait_cmd_ret
 */
static enum cmd_ret wait_cmd_ret(const char *cmd, TickType_t tmo)
{
	enum rx_parser_ret ret;

	while (TRUE) {
		ret = rx_parser(tmo);
		if (ret == RX_PARSER_RET_CMD_AOK) {
			return (CMD_RET_AOK);
		} else if (ret == RX_PARSER_RET_CMD_ERR) {
			return (CMD_RET_ERR);
		} else if (ret == RX_PARSER_RET_STRING) {
			return (CMD_RET_STR);
		} else if (ret == RX_PARSER_RET_STATUS || ret == RX_PARSER_RET_INTR) {
			continue;
		} else if (ret == RX_PARSER_RET_TMO) {
			msg(INF, "RN# Error: command %s timeout\n", cmd);
			return (CMD_RET_FAIL);
		} else {
			msg(INF, "RN# Error: wait_cmd_ret() rxp_ret=%s (command %s)\n",
			    find_txt_item(ret, rxp_ret_desc_ary, "IDX_ERR"), cmd);
			return (CMD_RET_FAIL);
		}
	}
}

/**
 * snd_stm_evnt
 */
static void snd_stm_evnt(enum rn487x_event_type type)
{
	struct rn487x_event event;

	memset(&event, 0, sizeof(struct rn487x_event));
	event.event_type = type;
	if (errQUEUE_FULL == xQueueSend(stm_evnt_que, &event, 0)) {
		stats.stm_evnt_que_full++;
		msg(INF, "RN# Error: stm_evnt_que full\n");
	}
}

/**
 * clear_status_fifo
 */
static boolean_t clear_status_fifo(const char *msg)
{
	union status st;
	boolean_t ret = FALSE;

	while (pdTRUE == xQueueReceive(status_fifo, &st, 0)) {
		msg(INF, "RN# Error: unprocessed status %s (%s)\n",
		    find_txt_item(st.common.id, status_id_desc_ary, "IDX_ERR"), msg);
		ret = TRUE;
                stats.unpar_status++;
	}
	return (ret);
}

/**
 * conv_addr_str_bin
 */
static boolean_t conv_addr_str_bin(const char *str, uint8_t *bin)
{
	for (int i = 0; i < 6; i++) {
		if (1 != sscanf(str + 2 * i, "%2hhx", bin + i)) {
			stats.sscanf_err++;
			msg(INF, "RN# Error: conv_addr_str_bin() sscanf failed\n");
                        return (FALSE);
		}
	}
	return (TRUE);
}

/**
 * rx_parser
 */
static enum rx_parser_ret rx_parser(TickType_t tmo)
{
	enum rx_parser_ret ret;
	int r;

	if (0 == (r = rx_msg(tmo))) {
		if (rxb[0] == '%') {
			rx_stat_parser(&ret);
		} else if (match_start("AOK", rxb)) {
			ret = RX_PARSER_RET_CMD_AOK;
		} else if (match_start("Err", rxb)) {
			ret = RX_PARSER_RET_CMD_ERR;
		} else if (match_start("CMD> ", rxb)) {
			ret = RX_PARSER_RET_CMDP;
		} else {
			ret = RX_PARSER_RET_STRING;
		}
	} else if (r == -ETMO) {
		ret = RX_PARSER_RET_TMO;
	} else if (r == -EBFOV) {
		ret = RX_PARSER_RET_ERROR;
	} else if (r == -ERCV) {
		ret = RX_PARSER_RET_ERROR;
        } else if (r == -EFMT) {
		ret = RX_PARSER_RET_ERROR;
        } else if (r == -EINTR) {
		ret = RX_PARSER_RET_INTR;
	}
#if RN487X_LOG_RXB == 1
	if (r == 0) {
		msg(INF, "RN# rx[%d] '%s'\n", strlen(rxb), rxb);
	} else if (r == -ERCV || r == -EFMT) {
		msg(INF, "RN# rx[%d] err '%s'\n", strlen(rxb), rxb);
	}
#endif
	return (ret);
}

/**
 * rx_stat_parser
 */
static void rx_stat_parser(enum rx_parser_ret *ret)
{
	union status st;
#if TERMOUT == 1
	const char *err_fmt = "RN# Error: rx_stat_parser() '%s'\n";
#endif
	memset(&st, 0, sizeof(union status));
	if (rxb[strlen(rxb) - 1] != '%') {
		goto parser_err;
	}
	if (rxb[13] == ',' && rxb[15] == ',') {
		if (strstr(rxb + 16, ",Brcst,")) {
			st.common.id = STATUS_ID_NO_CON_ADVERT;
		} else {
			st.common.id = STATUS_ID_CON_ADVERT;
		}
		goto push_status;
	}
	switch (rxb[1]) {
	case 'A' :
		if (match_start("ADV_TIMEOUT", rxb + 1)) {
			st.common.id = STATUS_ID_ADV_TIMEOUT;
			goto push_status;
		}
		break;
	case 'B' :
		if (match_start("BONDED", rxb + 1)) {
			st.common.id = STATUS_ID_BONDED;
			goto push_status;
		}
		break;
	case 'C' :
		if (match_start("CONN_PARAM", rxb + 1)) {
			st.common.id = STATUS_ID_CONN_PARAM;
			goto push_status;
		}
		if (match_start("CONNECT", rxb + 1)) {
			st.common.id = STATUS_ID_CONNECT;
			if (*(rxb + 9) == '0') {
				st.connect.addr_type = RN487X_ADDR_TYPE_PUBLIC;
			} else if (*(rxb + 9) == '1') {
				st.connect.addr_type = RN487X_ADDR_TYPE_RANDOM;
			} else {
				msg(INF, err_fmt, "CONNECT format");
				goto parser_err;
			}
			if (*(rxb + 23) != '%') {
				msg(INF, err_fmt, "CONNECT format");
				goto parser_err;
			}
			if (!conv_addr_str_bin(rxb + 11, st.connect.addr)) {
				msg(INF, err_fmt, "CONNECT format");
				goto parser_err;
			}
			goto push_status;
		}
		break;
	case 'D' :
		if (match_start("DISCONNECT", rxb + 1)) {
			st.common.id = STATUS_ID_DISCONNECT;
			goto push_status;
		}
		break;
	case 'E' :
		if (match_start("ERR_CONNPARAM", rxb + 1)) {
			st.common.id = STATUS_ID_ERR_CONNPARAM;
			goto push_status;
		}
		if (match_start("ERR_MEMORY", rxb + 1)) {
			st.common.id = STATUS_ID_ERR_MEMORY;
			goto push_status;
		}
		if (match_start("ERR_READ", rxb + 1)) {
			st.common.id = STATUS_ID_ERR_READ;
			goto push_status;
		}
		if (match_start("ERR_RMT_CMD", rxb + 1)) {
			st.common.id = STATUS_ID_ERR_RMT_CMD;
			goto push_status;
		}
		if (match_start("ERR_SEC", rxb + 1)) {
			st.common.id = STATUS_ID_ERR_SEC;
			goto push_status;
		}
		break;
	case 'K' :
		if (match_start("KEY:", rxb + 1)) {
			st.common.id = STATUS_ID_KEY;
			int i;
			for (i = 0; i < 6; i++) {
				st.key.key[i] = *(rxb + 5 + i);
			}
			st.key.key[i] = 0;
			goto push_status;
		}
		if (match_start("KEY_REQ", rxb + 1)) {
			st.common.id = STATUS_ID_KEY_REQ;
			goto push_status;
		}
		break;
	case 'I' :
		if (match_start("INDI,", rxb + 1)) {
			st.common.id = STATUS_ID_INDICATION;
			goto push_status;
		}
		break;
	case 'N' :
		if (match_start("NOTI,", rxb + 1)) {
			st.common.id = STATUS_ID_NOTIFICATION;
			goto push_status;
		}
		break;
	case 'P' :
		if (match_start("PIO1H", rxb + 1)) {
			st.common.id = STATUS_ID_PIO1H;
			goto push_status;
		}
		if (match_start("PIO1L", rxb + 1)) {
			st.common.id = STATUS_ID_PIO1L;
			goto push_status;
		}
		if (match_start("PIO2H", rxb + 1)) {
			st.common.id = STATUS_ID_PIO2H;
			goto push_status;
		}
		if (match_start("PIO2L", rxb + 1)) {
			st.common.id = STATUS_ID_PIO2L;
			goto push_status;
		}
		if (match_start("PIO3H", rxb + 1)) {
			st.common.id = STATUS_ID_PIO3H;
			goto push_status;
		}
		if (match_start("PIO3L", rxb + 1)) {
			st.common.id = STATUS_ID_PIO3L;
			goto push_status;
		}
		break;
	case 'R' :
		if (match_start("RE_DISCV", rxb + 1)) {
			st.common.id = STATUS_ID_RE_DISCV;
			goto push_status;
		}
		if (match_start("REBOOT", rxb + 1)) {
			st.common.id = STATUS_ID_REBOOT;
			goto push_status;
		}
		if (match_start("RMT_CMD_OFF", rxb + 1)) {
			st.common.id = STATUS_ID_RMT_CMD_OFF;
			goto push_status;
		}
		if (match_start("RMT_CMD_ON", rxb + 1)) {
			st.common.id = STATUS_ID_RMT_CMD_ON;
			goto push_status;
		}
		if (match_start("RV", rxb + 1)) {
			st.common.id = STATUS_ID_RV;
			goto push_status;
		}
		break;
	case 'S' :
		if (match_start("S_RUN", rxb + 1)) {
			st.common.id = STATUS_ID_S_RUN;
			goto push_status;
		}
		if (match_start("SECURED", rxb + 1)) {
			st.common.id = STATUS_ID_SECURED;
			goto push_status;
		}
		if (match_start("STREAM_OPEN", rxb + 1)) {
			st.common.id = STATUS_ID_STREAM_OPEN;
			goto push_status;
		}
		break;
	case 'T' :
		if (match_start("TMR1", rxb + 1)) {
			st.common.id = STATUS_ID_TMR1;
			goto push_status;
		}
		if (match_start("TMR2", rxb + 1)) {
			st.common.id = STATUS_ID_TMR2;
			goto push_status;
		}
		if (match_start("TMR3", rxb + 1)) {
			st.common.id = STATUS_ID_TMR3;
			goto push_status;
		}
		break;
	case 'U' :
		if (match_start("Unknown Device", rxb + 1)) {
			st.common.id = STATUS_ID_UNKNOWN_DEVICE;
			goto push_status;
		}
		break;
	case 'W' :
		if (match_start("WC", rxb + 1)) {
			st.common.id = STATUS_ID_WC;
			if (14 != strlen(rxb)) {
				msg(INF, err_fmt, "WC invalid format");
				goto parser_err;
			}
			if (1 != sscanf(rxb + 4, "%4hx", &st.wc.handle)) {
				stats.sscanf_err++;
                                msg(INF, err_fmt, "WC handle sscanf failed");
				goto parser_err;
			}
			if (1 != sscanf(rxb + 9, "%4hx", &st.wc.notif_req)) {
				stats.sscanf_err++;
                                msg(INF, err_fmt, "WC notif_req sscanf failed");
				goto parser_err;
			}
			if (st.wc.notif_req != CLIENT_NOTIF_ON && st.wc.notif_req != CLIENT_INDI_ON &&
			    st.wc.notif_req != CLIENT_NOTIF_OFF) {
				msg(INF, err_fmt, "WC invalid value");
				goto parser_err;
			}
			goto push_status;
		}
		if (match_start("WV", rxb + 1)) {
			uint16_t h;
                        struct rn487x_characteristic *chr;
			st.common.id = STATUS_ID_WV;
			if (rxb[8] != ',') {
				msg(INF, err_fmt, "WV invalid format");
				goto parser_err;
			}
			if (1 != sscanf(rxb + 4, "%4hx", &h)) {
				stats.sscanf_err++;
                                msg(INF, err_fmt, "WV handle sscanf failed");
				goto parser_err;
			}
			if (NULL != (chr = chr_search_by_h(h))) {
				uint8_t ary[chr->size];
				memset(ary, 0, chr->size);
				int sz = (strlen(rxb) - 10) / 2;
				if (sz < 1 || sz > chr->size) {
					msg(INF, err_fmt, "WV bad data size");
                                        goto parser_err;
				}
				int d = chr->size - sz;
				for (int i = 0; i < sz; i++) {
					if (1 != sscanf(rxb + 9 + 2 * i, "%2hhx", &ary[i + d])) {
						stats.sscanf_err++;
						msg(INF, err_fmt, "WV value sscanf failed");
						goto parser_err;
					}
				}
				if (errQUEUE_FULL == xQueueSend(chr->write, ary, 0)) {
                                        stats.cli_wr_que_full++;
                                        msg(INF, "RN# Error: rx_stat_parser() WV cli_wr_que full '%'\n", chr->name);
                                        goto push_status;
				}
				struct rn487x_event event;
				memset(&event, 0, sizeof(struct rn487x_event));
				event.event_type = RN487X_EVENT_GATT_CLIENT_WRITE;
				event.param.chr_name = chr->name;
				if (errQUEUE_FULL == xQueueSend(stm_evnt_que, &event, 0)) {
                                        stats.stm_evnt_que_full++;
                                        msg(INF, err_fmt, "WV stm_evnt_que full\n");
				}
			} else {
				msg(INF, err_fmt, "WV chr not found");
                                goto parser_err;
			}
			goto push_status;
		}
		break;
	}
parser_err:
	stats.st_pars_err++;
	*ret = RX_PARSER_RET_ERROR;
	return;
push_status:
	*ret = RX_PARSER_RET_STATUS;
	if (errQUEUE_FULL == xQueueSend(status_fifo, &st, 0)) {
		stats.fifo_full++;
                msg(INF, err_fmt, "status_fifo full");
	}
}

/**
 * rx_msg
 */
static int rx_msg(TickType_t tmo)
{
	char c;
	int r;
	int idx = 0;
        TickType_t tmo_sum = 0;
	TickType_t tmo_1ch = RX_CHAR_TMO;
	boolean_t ercv = FALSE;
	boolean_t st1 = FALSE;

	memset(rxb, 0, RN487X_RXB_SIZE);
	while (TRUE) {
		if (0 == (r = conf->rcv_fn(conf->ser_dev, &c, tmo_1ch)) || r == -ERCV) {
			stats.rx_msg_char_cnt++;
			if (idx > RN487X_RXB_SIZE - 2) {
				stats.rx_msg_ebfov++;
				return (-EBFOV);
			}
			if (r == -ERCV) {
				ercv = TRUE;
				c = '?';
                                stats.rx_msg_ercv++;
			}
			if (c == '\r' || c == '\n') {
				if (idx == 0) {
#if RN487X_LOG_RXB == 1
					msg(INF, "RN# rx[1] '%s'\n", (c == '\r') ? "\\r" : "\\n");
#endif
					continue;
				}
				if (ercv) {
					return (-ERCV);
				} else {
					return (0);
				}
			} else if (c == '%') {
				if (idx == 0) {
					st1 = TRUE;
                                        rxb[idx++] = '%';
                                        tmo_1ch = RX_CHAR_EXT_TMO;
				} else {
					if (!st1) {
						st1 = TRUE;
						rxb[0] = '%';
                                                idx = 1;
                                                stats.rx_msg_nost1++;
					} else {
						rxb[idx] = '%';
						return (0);
					}
				}
			} else {
				rxb[idx++] = c;
				if (idx == 5 && match_start("CMD> ", rxb)) {
					return (0);
				}
			}
		} else if (r == -ETMO) {
			if (idx != 0) {
				stats.rx_msg_efmt++;
				return (-EFMT);
			}
			tmo_sum += tmo_1ch;
			if (tmo_sum >= tmo) {
				return (-ETMO);
			}
		} else if (r == -EINTR && idx == 0) {
			return (-EINTR);
		}
	}
}

/**
 * match_start
 */
static boolean_t match_start(char *m, char *s)
{
	int i = 0;

	do {
		if (m[i] != s[i]) {
			return (FALSE);
		}
	} while (m[++i] != '\0');
	return (TRUE);
}

/**
 * chr_search_by_h
 */
static struct rn487x_characteristic *chr_search_by_h(uint16_t h)
{
	struct rn487x_service *srv;
	struct rn487x_characteristic *chr;

	if ((srv = conf->serv_list)) {
		do {
			chr = srv->chr_list;
			do {
				if (chr->handle == h) {
					return (chr);
				}
			} while ((chr = chr->next));
		} while ((srv = srv->next));
	} else {
		msg(INF, "RN# Error: chr_search_by_h() conf->serv_list not defined\n");
	}
        return (NULL);
}

/**
 * chr_search_by_n
 */
static struct rn487x_characteristic *chr_search_by_n(const char *n)
{
	struct rn487x_service *srv;
	struct rn487x_characteristic *chr;

	if ((srv = conf->serv_list)) {
		do {
			chr = srv->chr_list;
			do {
				if (0 == strcmp(n, chr->name)) {
					return (chr);
				}
			} while ((chr = chr->next));
		} while ((srv = srv->next));
	} else {
		msg(INF, "RN# Error: chr_search_by_n() conf->serv_list not defined\n");
	}
        return (NULL);
}

/**
 * is_bt_addr_inval
 */
static boolean_t is_bt_addr_inval(const uint8_t *a)
{
	for (int i = 0; i < 6; i++) {
		if (*(a + i) != 0) {
			return (FALSE);
		}
	}
	return (TRUE);
}

#if TERMOUT == 1 && RN487X_LOG_LEVEL > 0
/**
 * log_notif_req
 */
static void log_notif_req(const struct status_wc *wc)
{
	struct rn487x_service *srv;
	struct rn487x_characteristic *chr;

	if ((srv = conf->serv_list)) {
		do {
			chr = srv->chr_list;
			do {
				if (chr->cccd_handle == wc->handle) {
					char *p;
					if (wc->notif_req == CLIENT_NOTIF_ON || wc->notif_req == CLIENT_INDI_ON) {
						if (wc->notif_req == CLIENT_NOTIF_ON) {
							p = "notification";
						} else {
							p = "indication";
						}
						msg(INF, "RN# Enable %s on '%s'\n", p, chr->name);
					} else {
						msg(INF, "RN# Stop %s on '%s'\n",
						    (chr->props & RN487X_CHAR_PROP_NOTIFY) ? "notification" : "indication",
						    chr->name);
					}
					return;
				}
			} while ((chr = chr->next));
		} while ((srv = srv->next));
                msg(INF, "RN# Error: unknown handle in 'WC' status\n");
	}
}
#endif

#if TERMOUT == 1
/**
 * log_def_params
 */
static void log_def_params(void)
{
	if (conf->def_conn_params) {
		if (conf->def_conn_params->min_interval != conf->def_conn_params->max_interval) {
			msg(INF, "RN# Default con_param: con_int=<%.2f, %.2f> ms, lat=%hu evnt, tmo=%d ms\n",
			    conf->def_conn_params->min_interval * 1.25F,
			    conf->def_conn_params->max_interval * 1.25F,
			    conf->def_conn_params->latency,
			    conf->def_conn_params->timeout * 10);
		} else {
			msg(INF, "RN# Default con_param: con_int=%.2f ms, lat=%hu evnt, tmo=%d ms\n",
			    conf->def_conn_params->min_interval * 1.25F,
			    conf->def_conn_params->latency,
			    conf->def_conn_params->timeout * 10);
		}
	}
	if (conf->def_advert_params) {
		msg(INF, "RN# Default adv_param: fast_int=%.2f ms, fast_tmo=%.2f s, slow_int=%.2f ms\n",
		    conf->def_advert_params->fast_interval * 0.625F,
		    conf->def_advert_params->fast_timeout * 10.24F,
		    conf->def_advert_params->slow_interval * 0.625F);
	}
	if (conf->advert_params) {
		msg(INF, "RN# Actual adv_param: int=%hu ms\n", conf->advert_params->interval);
	}
	if (conf->def_beac_advert_params) {
		msg(INF, "RN# Default adv_beac_param: int=%.2f ms\n",
		    conf->def_beac_advert_params->interval * 0.625F);
	}
}
#endif

#if RN487X_LOG_CMD == 1
/**
 * log_cmd
 */
static void log_cmd(char *cmd)
{
	for (int i = 0; ;i++) {
		if (*(cmd + i) == '\0') {
			break;
		} else if (*(cmd + i) == '\r') {
			*(cmd + i) = '\0';
			break;
		}
	}
	msg(INF, "RN# > %s\n", cmd);
}
#endif

/**
 * upd_chr_def_vals
 */
static void upd_chr_def_vals(void)
{
	struct rn487x_service *srv;
	struct rn487x_characteristic *chr;

	if ((srv = conf->serv_list)) {
		do {
			chr = srv->chr_list;
			do {
				uint8_t data[chr->size];
				while (pdTRUE == xQueueReceive(chr->update, data, 0));
                                if (chr->def_val) {
					xQueueSend(chr->update, chr->def_val, 0);
				}
			} while ((chr = chr->next));
		} while ((srv = srv->next));
	}
}

/**
 * rn487x_send_cmd
 */
void rn487x_send_cmd(const union rn487x_cmd *cmd)
{
	xQueueSend(rn487x_cmd_que, cmd, portMAX_DELAY);
        conf->intr_rx_fn(conf->ser_dev);
}

/**
 * cmd_handler
 */
static p_stf_t cmd_handler(p_stf_t stf)
{
	if (pdFALSE == xQueueReceive(rn487x_cmd_que, &rn487x_cmd_act, 0)) {
		return (NULL);
	}
	switch (rn487x_cmd_act.common.cmd_type) {
	case RN487X_CMD_RESET         :
                memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
		return (state_reset);
	case RN487X_CMD_EXT_RESET     :
                memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
		return (state_ext_reset);
	case RN487X_CMD_DISCONNECT    :
                memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
		if (stf == state_connected) {
			return (state_disconnect);
		}
		break;
	case RN487X_CMD_SET_PUB_ADDR  :
		/* FALLTHRU */
	case RN487X_CMD_SET_FACT_DEF  :
                return (state_reset);
	case RN487X_CMD_REQ_SEC_LINK  :
		memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
		if (stf == state_connected && conn_secured == FALSE) {
			enum cmd_ret r = cmd_noarg("B");
			if (r == CMD_RET_AOK) {
				return (NULL);
			} else if (r == CMD_RET_ERR) {
				return (state_disconnect);
			} else {
				return (state_reset);
			}
		}
		break;
	case RN487X_CMD_CLR_ALL_BOND  :
		memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
		if (CMD_RET_AOK != cmd_noarg("U,Z")) {
			snd_stm_evnt(RN487X_EVENT_CMD_ERR);
			return (state_reset);
		}
                snd_stm_evnt(RN487X_EVENT_CMD_OK);
		break;
	case RN487X_CMD_CLR_WHL       :
		memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
		if (CMD_RET_AOK != cmd_noarg("JC")) {
			snd_stm_evnt(RN487X_EVENT_CMD_ERR);
			return (state_reset);
		}
                snd_stm_evnt(RN487X_EVENT_CMD_OK);
		break;
	case RN487X_CMD_BOND_TO_WHL   :
		memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
		if (CMD_RET_AOK != cmd_noarg("JB")) {
			snd_stm_evnt(RN487X_EVENT_CMD_ERR);
			return (state_reset);
		}
                snd_stm_evnt(RN487X_EVENT_CMD_OK);
		break;
	case RN487X_CMD_READ_ALL_BOND :
		if (read_all_bond(rn487x_cmd_act.read_all_bond.btbl)) {
			memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
			snd_stm_evnt(RN487X_EVENT_CMD_OK);
			return (NULL);
		} else {
			memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
			snd_stm_evnt(RN487X_EVENT_CMD_ERR);
			return (state_reset);
		}
		break;
	case RN487X_CMD_READ_WHL      :
		if (read_whl(rn487x_cmd_act.read_whl.whl)) {
			memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
			snd_stm_evnt(RN487X_EVENT_CMD_OK);
			return (NULL);
		} else {
                	memset(&rn487x_cmd_act, 0, sizeof(rn487x_cmd_act));
			snd_stm_evnt(RN487X_EVENT_CMD_ERR);
			return (state_reset);
		}
		break;
	default                       :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
	return (NULL);
}

#if TERMOUT == 1
/**
 * log_rn487x_stats
 */
void log_rn487x_stats(void)
{
	msg(INF, "RN# CNT> rx_msg_char_cnt=%d rx_msg_ebfov=%d\n",
	    stats.rx_msg_char_cnt, stats.rx_msg_ebfov);
	msg(INF, "RN# CNT> rx_msg_efmt=%d rx_msg_ercv=%d\n",
	    stats.rx_msg_efmt, stats.rx_msg_ercv);
	msg(INF, "RN# CNT> rx_msg_nost1=%d fifo_full=%d\n",
	    stats.rx_msg_nost1, stats.fifo_full);
	msg(INF, "RN# CNT> st_pars_err=%d unpar_status=%d\n",
	    stats.st_pars_err, stats.unpar_status);
	msg(INF, "RN# CNT> unpar_adv_status=%d unpar_con_status=%d\n",
	    stats.unpar_adv_status, stats.unpar_con_status);
	msg(INF, "RN# CNT> unpar_cse_status=%d unpar_dis_status=%d\n",
	    stats.unpar_cse_status, stats.unpar_dis_status);
	msg(INF, "RN# CNT> reset=%d ext_reset=%d\n",
	    stats.reset, stats.ext_reset);
	msg(INF, "RN# CNT> adv_rxp_err=%d con_rxp_err=%d\n",
	    stats.adv_rxp_err, stats.con_rxp_err);
	msg(INF, "RN# CNT> cse_rxp_err=%d dis_rxp_err=%d\n",
	    stats.cse_rxp_err, stats.dis_rxp_err);
	msg(INF, "RN# CNT> sscanf_err=%d nfail=%d\n",
	    stats.sscanf_err, stats.nfail);
	msg(INF, "RN# CNT> cli_wr_que_full=%d stm_evnt_que_full=%d\n",
	    stats.cli_wr_que_full, stats.stm_evnt_que_full);
}
#endif
