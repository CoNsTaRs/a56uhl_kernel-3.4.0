

#define NFC_READ_RFSKUID 1
#define NFC_GET_BOOTMODE 1

#define NFC_BOOT_MODE_NORMAL 0
#define NFC_BOOT_MODE_FTM 1
#define NFC_BOOT_MODE_DOWNLOAD 2
#define NFC_BOOT_MODE_OFF_MODE_CHARGING 5

#define NFC_OFF_MODE_CHARGING_LOAD_SWITCH 0

int pn548_htc_check_rfskuid(int in_is_alive);

int pn548_htc_get_bootmode(void);


bool pn548_htc_parse_dt(struct device *dev);


void pn548_htc_off_mode_charging(void);


void pn548_htc_turn_off_pvdd(void);


bool pn548_htc_turn_on_pvdd(struct i2c_client *client);


void pn548_htc_power_off_sequence(int is_alive);

void pn548_htc_regulator_status(void);
