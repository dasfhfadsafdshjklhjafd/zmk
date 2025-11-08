/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci_types.h>

#if IS_ENABLED(CONFIG_SETTINGS)

#include <zephyr/settings/settings.h>

#endif

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ble.h>
#include <zmk/keys.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)
#include <zmk/events/keycode_state_changed.h>

#define PASSKEY_DIGITS 6

static struct bt_conn *auth_passkey_entry_conn;
RING_BUF_DECLARE(passkey_entries, PASSKEY_DIGITS);

#endif /* IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY) */

enum advertising_type {
    ZMK_ADV_NONE,
    ZMK_ADV_DIR,
    ZMK_ADV_CONN,
} advertising_status;

#define CURR_ADV(adv) (adv << 4)

#define ZMK_ADV_CONN_NAME                                                                          \
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME | BT_LE_ADV_OPT_USE_NAME |  \
                        BT_LE_ADV_OPT_FORCE_NAME_IN_AD,                                            \
                    BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

static struct zmk_ble_profile profiles[ZMK_BLE_PROFILE_COUNT];
static uint8_t active_profile;

#if IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS)

#if defined(CONFIG_EFOG_BLE_CONN_PARAMS_RESUME_DELAY)
#define BLE_CONN_PARAMS_RESUME_DELAY_MS CONFIG_EFOG_BLE_CONN_PARAMS_RESUME_DELAY
#else
#define BLE_CONN_PARAMS_RESUME_DELAY_MS 200
#endif

static bool force_conn_params_waiting_for_activity;

static uint16_t preferred_conn_min_interval = CONFIG_EFOG_BLE_BASE_MIN_INT;
static uint16_t preferred_conn_max_interval = CONFIG_EFOG_BLE_BASE_MAX_INT;
static uint16_t preferred_conn_latency = CONFIG_EFOG_BLE_BASE_LATENCY;
static uint16_t preferred_conn_timeout = CONFIG_BT_PERIPHERAL_PREF_TIMEOUT;

enum conn_param_mode { CONN_PARAMS_MODE_BASE, CONN_PARAMS_MODE_FAST };
static enum conn_param_mode conn_params_mode = CONN_PARAMS_MODE_BASE;

static bool fast_conn_params_pending;
static uint8_t fast_conn_params_attempts;
#if IS_ENABLED(CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT)
static bool fast_conn_params_disconnecting;
#endif

static void force_conn_params_after_activity(struct k_work *work);
static void fast_conn_params_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(force_conn_params_after_activity_work,
                               force_conn_params_after_activity);
static K_WORK_DELAYABLE_DEFINE(fast_conn_params_work, fast_conn_params_work_handler);

#endif

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#if IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS)
static int64_t force_conn_params_last_request;
static uint8_t force_conn_params_failures;

static inline bool conn_params_cooldown_expired(int64_t now) {
    return (force_conn_params_last_request == 0) ||
           (now - force_conn_params_last_request >= CONFIG_ZMK_BLE_FORCE_CONN_PARAMS_RETRY_MS);
}

static void conn_params_use_base(void) {
    preferred_conn_min_interval = CONFIG_EFOG_BLE_BASE_MIN_INT;
    preferred_conn_max_interval = CONFIG_EFOG_BLE_BASE_MAX_INT;
    preferred_conn_latency = CONFIG_EFOG_BLE_BASE_LATENCY;
    conn_params_mode = CONN_PARAMS_MODE_BASE;
}

static void conn_params_use_fast(void) {
    preferred_conn_min_interval = CONFIG_EFOG_BLE_FAST_MIN_INT;
    preferred_conn_max_interval = CONFIG_EFOG_BLE_FAST_MAX_INT;
    preferred_conn_latency = CONFIG_EFOG_BLE_FAST_LATENCY;
    conn_params_mode = CONN_PARAMS_MODE_FAST;
}

static inline bool intervals_match_fast(uint16_t interval, uint16_t latency) {
    return interval <= CONFIG_EFOG_BLE_FAST_MAX_INT && latency <= CONFIG_EFOG_BLE_FAST_LATENCY;
}

static void fast_conn_params_schedule(int32_t delay_ms, bool reset_attempts) {
    if (delay_ms < 0) {
        delay_ms = 0;
    }
    if (reset_attempts) {
        fast_conn_params_attempts = 0;
#if IS_ENABLED(CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT)
        fast_conn_params_disconnecting = false;
#endif
    }

#if IS_ENABLED(CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT)
    if (fast_conn_params_disconnecting) {
        return;
    }
#endif

    fast_conn_params_pending = true;
    k_work_reschedule(&fast_conn_params_work, K_MSEC(delay_ms));
}

static void force_conn_params_request(struct bt_conn *conn, bool ignore_cooldown) {
    int64_t now = k_uptime_get();

    if (!ignore_cooldown && !conn_params_cooldown_expired(now)) {
        return;
    }

    int err = bt_conn_le_param_update(conn,
                                      BT_LE_CONN_PARAM(preferred_conn_min_interval,
                                                       preferred_conn_max_interval,
                                                       preferred_conn_latency,
                                                       preferred_conn_timeout));

    if (err == -EALREADY || err == -EINPROGRESS) {
        force_conn_params_failures = 0;
    } else if (err) {
        force_conn_params_failures++;
        LOG_WRN("Failed to request preferred BLE params (%d)", err);
    } else {
        force_conn_params_failures = 0;
        LOG_DBG("Requested preferred BLE params");
    }

    force_conn_params_last_request = now;

#if CONFIG_ZMK_BLE_FORCE_CONN_PARAMS_MAX_RETRIES > 0
    if (IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS_DISCONNECT) &&
        force_conn_params_failures >= CONFIG_ZMK_BLE_FORCE_CONN_PARAMS_MAX_RETRIES) {
        LOG_WRN("Disconnecting to renegotiate BLE params after %u failures",
                force_conn_params_failures);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        force_conn_params_failures = 0;
        force_conn_params_last_request = k_uptime_get();
    }
#endif
}

static void force_conn_params_reset_state(void) {
    force_conn_params_last_request = 0;
    force_conn_params_failures = 0;
    force_conn_params_waiting_for_activity = false;
    fast_conn_params_pending = false;
    fast_conn_params_attempts = 0;
#if IS_ENABLED(CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT)
    fast_conn_params_disconnecting = false;
#endif
    conn_params_use_base();
    preferred_conn_timeout = CONFIG_BT_PERIPHERAL_PREF_TIMEOUT;
    k_work_cancel_delayable(&force_conn_params_after_activity_work);
    k_work_cancel_delayable(&fast_conn_params_work);
}

static void force_conn_params_after_activity(struct k_work *work) {
    ARG_UNUSED(work);

    struct bt_conn *conn = zmk_ble_active_profile_conn();

    if (!conn) {
        return;
    }

    force_conn_params_request(conn, false);
    bt_conn_unref(conn);
}

static void fast_conn_params_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    struct bt_conn *conn = zmk_ble_active_profile_conn();

    if (!conn) {
        fast_conn_params_pending = false;
#if IS_ENABLED(CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT)
        fast_conn_params_disconnecting = false;
#endif
        return;
    }

    int err = bt_conn_le_param_update(
        conn, BT_LE_CONN_PARAM(CONFIG_EFOG_BLE_FAST_MIN_INT, CONFIG_EFOG_BLE_FAST_MAX_INT,
                               CONFIG_EFOG_BLE_FAST_LATENCY, preferred_conn_timeout));

    if (err == -EALREADY) {
        LOG_DBG("Fast BLE params already active");
        conn_params_use_fast();
        fast_conn_params_pending = false;
        fast_conn_params_attempts = 0;
#if IS_ENABLED(CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT)
        fast_conn_params_disconnecting = false;
#endif
    } else if (err == -EINPROGRESS) {
        LOG_DBG("Fast BLE params request in progress");
    } else if (err) {
        LOG_WRN("Fast BLE params request failed (%d)", err);
        fast_conn_params_attempts++;
#if IS_ENABLED(CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT)
        if (!fast_conn_params_disconnecting &&
            fast_conn_params_attempts >= CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT_ATTEMPTS) {
            fast_conn_params_disconnecting = true;
            LOG_WRN("Disconnecting to renegotiate BLE params after %u fast failures",
                    fast_conn_params_attempts);
            bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            fast_conn_params_pending = false;
            bt_conn_unref(conn);
            return;
        }
#endif
        bool attempts_left = false;
#if CONFIG_EFOG_BLE_FAST_REQUEST_RETRIES > 0
        attempts_left = fast_conn_params_attempts <= CONFIG_EFOG_BLE_FAST_REQUEST_RETRIES;
#endif
        if (IS_ENABLED(CONFIG_EFOG_BLE_FAST_RETRY_FOREVER) || attempts_left) {
            k_work_reschedule(&fast_conn_params_work,
                              K_MSEC(CONFIG_EFOG_BLE_FAST_REQUEST_SPACING_MS));
        } else {
            fast_conn_params_pending = false;
        }
    } else {
        LOG_DBG("Requested fast BLE params");
#if IS_ENABLED(CONFIG_EFOG_BLE_FAST_FORCE_DISCONNECT)
        fast_conn_params_disconnecting = false;
#endif
    }

    bt_conn_unref(conn);
}
#endif /* CONFIG_ZMK_BLE_FORCE_CONN_PARAMS */

BUILD_ASSERT(
    DEVICE_NAME_LEN <= CONFIG_BT_DEVICE_NAME_MAX,
    "ERROR: BLE device name is too long. Max length: " STRINGIFY(CONFIG_BT_DEVICE_NAME_MAX));

static struct bt_data zmk_ble_ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0xC1, 0x03),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME, 0x12, 0x18, /* HID Service */
                  0x0f, 0x18                       /* Battery Service */
                  ),
};

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

static bt_addr_le_t peripheral_addrs[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) */

static void raise_profile_changed_event(void) {
    raise_zmk_ble_active_profile_changed((struct zmk_ble_active_profile_changed){
        .index = active_profile, .profile = &profiles[active_profile]});
}

static void raise_profile_changed_event_callback(struct k_work *work) {
    raise_profile_changed_event();
}

K_WORK_DEFINE(raise_profile_changed_event_work, raise_profile_changed_event_callback);

bool zmk_ble_active_profile_is_open(void) { return zmk_ble_profile_is_open(active_profile); }

bool zmk_ble_profile_is_open(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return false;
    }
    return !bt_addr_le_cmp(&profiles[index].peer, BT_ADDR_LE_ANY);
}

void set_profile_address(uint8_t index, const bt_addr_le_t *addr) {
    char setting_name[17];
    char addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    memcpy(&profiles[index].peer, addr, sizeof(bt_addr_le_t));
    sprintf(setting_name, "ble/profiles/%d", index);
    LOG_DBG("Setting profile addr for %s to %s", setting_name, addr_str);
#if IS_ENABLED(CONFIG_SETTINGS)
    settings_save_one(setting_name, &profiles[index], sizeof(struct zmk_ble_profile));
#endif
    k_work_submit(&raise_profile_changed_event_work);
}

bool zmk_ble_active_profile_is_connected(void) {
    return zmk_ble_profile_is_connected(active_profile);
}

void zmk_ble_notify_activity(void) {
#if IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS)
    if (!force_conn_params_waiting_for_activity) {
        return;
    }

    if (!zmk_ble_active_profile_is_connected()) {
        return;
    }

    k_work_reschedule(&force_conn_params_after_activity_work,
                      K_MSEC(BLE_CONN_PARAMS_RESUME_DELAY_MS));
#endif
}

bool zmk_ble_profile_is_connected(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return false;
    }
    struct bt_conn *conn;
    struct bt_conn_info info;
    bt_addr_le_t *addr = &profiles[index].peer;
    if (!bt_addr_le_cmp(addr, BT_ADDR_LE_ANY)) {
        return false;
    } else if ((conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr)) == NULL) {
        return false;
    }

    bt_conn_get_info(conn, &info);

    bt_conn_unref(conn);

    return info.state == BT_CONN_STATE_CONNECTED;
}

#define CHECKED_ADV_STOP()                                                                         \
    err = bt_le_adv_stop();                                                                        \
    advertising_status = ZMK_ADV_NONE;                                                             \
    if (err) {                                                                                     \
        LOG_ERR("Failed to stop advertising (err %d)", err);                                       \
        return err;                                                                                \
    }

#define CHECKED_DIR_ADV()                                                                          \
    addr = zmk_ble_active_profile_addr();                                                          \
    conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);                                            \
    if (conn != NULL) { /* TODO: Check status of connection */                                     \
        LOG_DBG("Skipping advertising, profile host is already connected");                        \
        bt_conn_unref(conn);                                                                       \
        return 0;                                                                                  \
    }                                                                                              \
    err = bt_le_adv_start(BT_LE_ADV_CONN_DIR_LOW_DUTY(addr), zmk_ble_ad, ARRAY_SIZE(zmk_ble_ad),   \
                          NULL, 0);                                                                \
    if (err) {                                                                                     \
        LOG_ERR("Advertising failed to start (err %d)", err);                                      \
        return err;                                                                                \
    }                                                                                              \
    advertising_status = ZMK_ADV_DIR;

#define CHECKED_OPEN_ADV()                                                                         \
    err = bt_le_adv_start(ZMK_ADV_CONN_NAME, zmk_ble_ad, ARRAY_SIZE(zmk_ble_ad), NULL, 0);         \
    if (err) {                                                                                     \
        LOG_ERR("Advertising failed to start (err %d)", err);                                      \
        return err;                                                                                \
    }                                                                                              \
    advertising_status = ZMK_ADV_CONN;

int update_advertising(void) {
    int err = 0;
    bt_addr_le_t *addr;
    struct bt_conn *conn;
    enum advertising_type desired_adv = ZMK_ADV_NONE;

    if (zmk_ble_active_profile_is_open()) {
        desired_adv = ZMK_ADV_CONN;
    } else if (!zmk_ble_active_profile_is_connected()) {
        desired_adv = ZMK_ADV_CONN;
        // Need to fix directed advertising for privacy centrals. See
        // https://github.com/zephyrproject-rtos/zephyr/pull/14984 char
        // addr_str[BT_ADDR_LE_STR_LEN]; bt_addr_le_to_str(zmk_ble_active_profile_addr(), addr_str,
        // sizeof(addr_str));

        // LOG_DBG("Directed advertising to %s", addr_str);
        // desired_adv = ZMK_ADV_DIR;
    }
    LOG_DBG("advertising from %d to %d", advertising_status, desired_adv);

    switch (desired_adv + CURR_ADV(advertising_status)) {
    case ZMK_ADV_NONE + CURR_ADV(ZMK_ADV_DIR):
    case ZMK_ADV_NONE + CURR_ADV(ZMK_ADV_CONN):
        CHECKED_ADV_STOP();
        break;
    case ZMK_ADV_DIR + CURR_ADV(ZMK_ADV_DIR):
    case ZMK_ADV_DIR + CURR_ADV(ZMK_ADV_CONN):
        CHECKED_ADV_STOP();
        CHECKED_DIR_ADV();
        break;
    case ZMK_ADV_DIR + CURR_ADV(ZMK_ADV_NONE):
        CHECKED_DIR_ADV();
        break;
    case ZMK_ADV_CONN + CURR_ADV(ZMK_ADV_DIR):
        CHECKED_ADV_STOP();
        CHECKED_OPEN_ADV();
        break;
    case ZMK_ADV_CONN + CURR_ADV(ZMK_ADV_NONE):
        CHECKED_OPEN_ADV();
        break;
    }

    return 0;
};

static void update_advertising_callback(struct k_work *work) { update_advertising(); }

K_WORK_DEFINE(update_advertising_work, update_advertising_callback);

static void clear_profile_bond(uint8_t profile) {
    if (bt_addr_le_cmp(&profiles[profile].peer, BT_ADDR_LE_ANY)) {
        bt_unpair(BT_ID_DEFAULT, &profiles[profile].peer);
        set_profile_address(profile, BT_ADDR_LE_ANY);
    }
}

void zmk_ble_clear_bonds(void) {
    LOG_DBG("zmk_ble_clear_bonds()");

    clear_profile_bond(active_profile);
    update_advertising();
};

void zmk_ble_clear_all_bonds(void) {
    LOG_DBG("zmk_ble_clear_all_bonds()");

    // Unpair all profiles
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        clear_profile_bond(i);
    }

    // Automatically switch to profile 0
    zmk_ble_prof_select(0);
    update_advertising();
};

int zmk_ble_active_profile_index(void) { return active_profile; }

int zmk_ble_profile_index(const bt_addr_le_t *addr) {
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (bt_addr_le_cmp(addr, &profiles[i].peer) == 0) {
            return i;
        }
    }
    return -ENODEV;
}

bt_addr_le_t *zmk_ble_profile_address(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return (bt_addr_le_t *)(BT_ADDR_LE_NONE);
    }
    return &profiles[index].peer;
}

#if IS_ENABLED(CONFIG_SETTINGS)
static void ble_save_profile_work(struct k_work *work) {
    settings_save_one("ble/active_profile", &active_profile, sizeof(active_profile));
}

static struct k_work_delayable ble_save_work;
#endif

static int ble_save_profile(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    return k_work_reschedule(&ble_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
#else
    return 0;
#endif
}

int zmk_ble_prof_select(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT) {
        return -ERANGE;
    }

    LOG_DBG("profile %d", index);
    if (active_profile == index) {
        return 0;
    }

    active_profile = index;
    ble_save_profile();

    update_advertising();

    raise_profile_changed_event();

    return 0;
};

int zmk_ble_prof_next(void) {
    LOG_DBG("");
    return zmk_ble_prof_select((active_profile + 1) % ZMK_BLE_PROFILE_COUNT);
};

int zmk_ble_prof_prev(void) {
    LOG_DBG("");
    return zmk_ble_prof_select((active_profile + ZMK_BLE_PROFILE_COUNT - 1) %
                               ZMK_BLE_PROFILE_COUNT);
};

int zmk_ble_prof_disconnect(uint8_t index) {
    if (index >= ZMK_BLE_PROFILE_COUNT)
        return -ERANGE;

    bt_addr_le_t *addr = &profiles[index].peer;
    struct bt_conn *conn;
    int result;

    if (!bt_addr_le_cmp(addr, BT_ADDR_LE_ANY)) {
        return -ENODEV;
    } else if ((conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr)) == NULL) {
        return -ENODEV;
    }

    result = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    LOG_DBG("Disconnected from profile %d: %d", index, result);

    bt_conn_unref(conn);
    return result;
}

bt_addr_le_t *zmk_ble_active_profile_addr(void) { return &profiles[active_profile].peer; }

struct bt_conn *zmk_ble_active_profile_conn(void) {
    struct bt_conn *conn;
    bt_addr_le_t *addr = zmk_ble_active_profile_addr();

    if (!bt_addr_le_cmp(addr, BT_ADDR_LE_ANY)) {
        LOG_WRN("Not sending, no active address for current profile");
        return NULL;
    } else if ((conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr)) == NULL) {
        LOG_WRN("Not sending, not connected to active profile");
        return NULL;
    }

    return conn;
}

char *zmk_ble_active_profile_name(void) { return profiles[active_profile].name; }

int zmk_ble_set_device_name(char *name) {
    // Copy new name to advertising parameters
    int err = bt_set_name(name);
    LOG_DBG("New device name: %s", name);
    if (err) {
        LOG_ERR("Failed to set new device name (err %d)", err);
        return err;
    }
    if (advertising_status == ZMK_ADV_CONN) {
        // Stop current advertising so it can restart with new name
        err = bt_le_adv_stop();
        advertising_status = ZMK_ADV_NONE;
        if (err) {
            LOG_ERR("Failed to stop advertising (err %d)", err);
            return err;
        }
    }
    return update_advertising();
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

int zmk_ble_put_peripheral_addr(const bt_addr_le_t *addr) {
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        // If the address is recognized and already stored in settings, return
        // index and no additional action is necessary.
        if (bt_addr_le_cmp(&peripheral_addrs[i], addr) == 0) {
            LOG_DBG("Found existing peripheral address in slot %d", i);
            return i;
        } else {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&peripheral_addrs[i], addr_str, sizeof(addr_str));
            LOG_DBG("peripheral slot %d occupied by %s", i, addr_str);
        }

        // If the peripheral address slot is open, store new peripheral in the
        // slot and return index. This compares against BT_ADDR_LE_ANY as that
        // is the zero value.
        if (bt_addr_le_cmp(&peripheral_addrs[i], BT_ADDR_LE_ANY) == 0) {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
            LOG_DBG("Storing peripheral %s in slot %d", addr_str, i);
            bt_addr_le_copy(&peripheral_addrs[i], addr);

#if IS_ENABLED(CONFIG_SETTINGS)
            char setting_name[32];
            sprintf(setting_name, "ble/peripheral_addresses/%d", i);
            settings_save_one(setting_name, addr, sizeof(bt_addr_le_t));
#endif // IS_ENABLED(CONFIG_SETTINGS)
            return i;
        }
    }

    // The peripheral does not match a known peripheral and there is no
    // available slot.
    return -ENOMEM;
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) */

#if IS_ENABLED(CONFIG_SETTINGS)

static int ble_profiles_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    const char *next;

    LOG_DBG("Setting BLE value %s", name);

    if (settings_name_steq(name, "profiles", &next) && next) {
        char *endptr;
        uint8_t idx = strtoul(next, &endptr, 10);
        if (*endptr != '\0') {
            LOG_WRN("Invalid profile index: %s", next);
            return -EINVAL;
        }

        if (len != sizeof(struct zmk_ble_profile)) {
            LOG_ERR("Invalid profile size (got %d expected %d)", len,
                    sizeof(struct zmk_ble_profile));
            return -EINVAL;
        }

        if (idx >= ZMK_BLE_PROFILE_COUNT) {
            LOG_WRN("Profile address for index %d is larger than max of %d", idx,
                    ZMK_BLE_PROFILE_COUNT);
            return -EINVAL;
        }

        int err = read_cb(cb_arg, &profiles[idx], sizeof(struct zmk_ble_profile));
        if (err <= 0) {
            LOG_ERR("Failed to handle profile address from settings (err %d)", err);
            return err;
        }

        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&profiles[idx].peer, addr_str, sizeof(addr_str));

        LOG_DBG("Loaded %s address for profile %d", addr_str, idx);
    } else if (settings_name_steq(name, "active_profile", &next) && !next) {
        if (len != sizeof(active_profile)) {
            return -EINVAL;
        }

        int err = read_cb(cb_arg, &active_profile, sizeof(active_profile));
        if (err <= 0) {
            LOG_ERR("Failed to handle active profile from settings (err %d)", err);
            return err;
        }
    }
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    else if (settings_name_steq(name, "peripheral_addresses", &next) && next) {
        if (len != sizeof(bt_addr_le_t)) {
            return -EINVAL;
        }

        int i = atoi(next);
        if (i < 0 || i >= ZMK_SPLIT_BLE_PERIPHERAL_COUNT) {
            LOG_ERR("Failed to store peripheral address in memory");
        } else {
            int err = read_cb(cb_arg, &peripheral_addrs[i], sizeof(bt_addr_le_t));
            if (err <= 0) {
                LOG_ERR("Failed to handle peripheral address from settings (err %d)", err);
                return err;
            }
        }
    }
#endif

    return 0;
};

static int zmk_ble_complete_startup(void);

static struct settings_handler profiles_handler = {
    .name = "ble", .h_set = ble_profiles_handle_set, .h_commit = zmk_ble_complete_startup};

#endif /* IS_ENABLED(CONFIG_SETTINGS) */

static bool is_conn_active_profile(const struct bt_conn *conn) {
    return bt_addr_le_cmp(bt_conn_get_dst(conn), &profiles[active_profile].peer) == 0;
}

#if IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS)
static bool clamp_host_conn_params(struct bt_conn *conn, struct bt_le_conn_param *param) {
    struct bt_conn_info info;
    bool changed = false;

    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        return true;
    }

    if (!is_conn_active_profile(conn)) {
        return true;
    }

    uint16_t original_min = param->interval_min;
    uint16_t original_max = param->interval_max;
    uint16_t original_latency = param->latency;
    uint16_t original_timeout = param->timeout;

    if (param->interval_max > CONFIG_BT_PERIPHERAL_PREF_MAX_INT) {
        param->interval_max = CONFIG_BT_PERIPHERAL_PREF_MAX_INT;
        changed = true;
    }

    if (param->interval_min > param->interval_max) {
        param->interval_min = param->interval_max;
        changed = true;
    }

    if (param->latency > CONFIG_BT_PERIPHERAL_PREF_LATENCY) {
        param->latency = CONFIG_BT_PERIPHERAL_PREF_LATENCY;
        changed = true;
    }

    if (param->timeout > CONFIG_BT_PERIPHERAL_PREF_TIMEOUT) {
        param->timeout = CONFIG_BT_PERIPHERAL_PREF_TIMEOUT;
        changed = true;
    }

    if (changed) {
        LOG_WRN("Clamping host BLE params from %u-%u/%u/%u to %u-%u/%u/%u", original_min,
                original_max, original_latency, original_timeout, param->interval_min,
                param->interval_max, param->latency, param->timeout);
    } else {
        LOG_DBG("Accepting host BLE params %u-%u/%u/%u", param->interval_min,
                param->interval_max, param->latency, param->timeout);
    }

    return true;
}
#endif

static void connected(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;
    LOG_DBG("Connected thread: %p", k_current_get());

    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    advertising_status = ZMK_ADV_NONE;

    if (err) {
        LOG_WRN("Failed to connect to %s (%u)", addr, err);
        update_advertising();
        return;
    }

    LOG_DBG("Connected %s", addr);

    update_advertising();

#if IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS)
    force_conn_params_reset_state();
    if (is_conn_active_profile(conn)) {
        force_conn_params_waiting_for_activity = true;
        fast_conn_params_schedule(CONFIG_EFOG_BLE_FAST_REQUEST_DELAY_MS, true);
    }
#endif

    if (is_conn_active_profile(conn)) {
        LOG_DBG("Active profile connected");
        k_work_submit(&raise_profile_changed_event_work);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Disconnected from %s (reason 0x%02x)", addr, reason);

    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

    // We need to do this in a work callback, otherwise the advertising update will still see the
    // connection for a profile as active, and not start advertising yet.
    k_work_submit(&update_advertising_work);

    if (is_conn_active_profile(conn)) {
        LOG_DBG("Active profile disconnected");
#if IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS)
        force_conn_params_reset_state();
#endif
        k_work_submit(&raise_profile_changed_event_work);
    }
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_DBG("Security changed: %s level %u", addr, level);
    } else {
        LOG_ERR("Security failed: %s level %u err %d", addr, level, err);
    }
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                             uint16_t timeout) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    bt_conn_get_info(conn, &info);

    LOG_DBG("%s: interval %d latency %d timeout %d", addr, interval, latency, timeout);

#if IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS)
    if (info.role == BT_CONN_ROLE_PERIPHERAL && is_conn_active_profile(conn)) {
        if (interval > preferred_conn_max_interval || latency > preferred_conn_latency) {
            LOG_WRN("BLE params drifted (interval %d, latency %d) – requesting preferred values",
                    interval, latency);
            force_conn_params_request(conn, false);
        } else {
            force_conn_params_failures = 0;
            force_conn_params_waiting_for_activity = false;
            k_work_cancel_delayable(&force_conn_params_after_activity_work);
        }

        if (intervals_match_fast(interval, latency)) {
            if (conn_params_mode != CONN_PARAMS_MODE_FAST) {
                LOG_DBG("Fast BLE params active (interval %d, latency %d)", interval, latency);
            }
            conn_params_use_fast();
            fast_conn_params_pending = false;
            fast_conn_params_attempts = 0;
            k_work_cancel_delayable(&fast_conn_params_work);
        } else {
            if (conn_params_mode == CONN_PARAMS_MODE_FAST) {
                LOG_DBG("Host stepped off fast BLE params (interval %d, latency %d)", interval,
                        latency);
                conn_params_use_base();
            }

            if (!fast_conn_params_pending) {
                fast_conn_params_schedule(CONFIG_EFOG_BLE_FAST_REQUEST_DELAY_MS, true);
            }
        }
    }
#endif
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
    .le_param_updated = le_param_updated,
#if IS_ENABLED(CONFIG_ZMK_BLE_FORCE_CONN_PARAMS)
    .le_param_req = clamp_host_conn_params,
#endif
};

/*
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Passkey for %s: %06u", addr, passkey);
}
*/

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)

static void auth_passkey_entry(struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Passkey entry requested for %s", addr);
    ring_buf_reset(&passkey_entries);
    auth_passkey_entry_conn = bt_conn_ref(conn);
}

#endif

static void auth_cancel(struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)
    if (auth_passkey_entry_conn) {
        bt_conn_unref(auth_passkey_entry_conn);
        auth_passkey_entry_conn = NULL;
    }

    ring_buf_reset(&passkey_entries);
#endif

    LOG_DBG("Pairing cancelled: %s", addr);
}

static bool pairing_allowed_for_current_profile(struct bt_conn *conn) {
    return zmk_ble_active_profile_is_open() ||
           (IS_ENABLED(CONFIG_BT_SMP_ALLOW_UNAUTH_OVERWRITE) &&
            bt_addr_le_cmp(zmk_ble_active_profile_addr(), bt_conn_get_dst(conn)) == 0);
}

static enum bt_security_err auth_pairing_accept(struct bt_conn *conn,
                                                const struct bt_conn_pairing_feat *const feat) {
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);

    LOG_DBG("role %d, open? %s", info.role, zmk_ble_active_profile_is_open() ? "yes" : "no");
    if (info.role == BT_CONN_ROLE_PERIPHERAL && !pairing_allowed_for_current_profile(conn)) {
        LOG_WRN("Rejecting pairing request to taken profile %d", active_profile);
        return BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
    }

    return BT_SECURITY_ERR_SUCCESS;
};

static void auth_pairing_complete(struct bt_conn *conn, bool bonded) {
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];
    const bt_addr_le_t *dst = bt_conn_get_dst(conn);

    bt_addr_le_to_str(dst, addr, sizeof(addr));
    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

    if (!pairing_allowed_for_current_profile(conn)) {
        LOG_ERR("Pairing completed but current profile is not open: %s", addr);
        bt_unpair(BT_ID_DEFAULT, dst);
        return;
    }

    set_profile_address(active_profile, dst);
    update_advertising();
};

static struct bt_conn_auth_cb zmk_ble_auth_cb_display = {
    .pairing_accept = auth_pairing_accept,
// .passkey_display = auth_passkey_display,

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)
    .passkey_entry = auth_passkey_entry,
#endif
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb zmk_ble_auth_info_cb_display = {
    .pairing_complete = auth_pairing_complete,
};

static void zmk_ble_ready(int err) {
    LOG_DBG("ready? %d", err);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    update_advertising();
}

static int zmk_ble_complete_startup(void) {

#if IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START)
    LOG_WRN("Clearing all existing BLE bond information from the keyboard");

    bt_unpair(BT_ID_DEFAULT, NULL);

    for (int i = 0; i < 8; i++) {
        char setting_name[15];
        sprintf(setting_name, "ble/profiles/%d", i);

        int err = settings_delete(setting_name);
        if (err) {
            LOG_ERR("Failed to delete setting: %d", err);
        }
    }

    // Hardcoding a reasonable hardcoded value of peripheral addresses
    // to clear so we properly clear a split central as well.
    for (int i = 0; i < 8; i++) {
        char setting_name[32];
        sprintf(setting_name, "ble/peripheral_addresses/%d", i);

        int err = settings_delete(setting_name);
        if (err) {
            LOG_ERR("Failed to delete setting: %d", err);
        }
    }

#endif // IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START)

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&zmk_ble_auth_cb_display);
    bt_conn_auth_info_cb_register(&zmk_ble_auth_info_cb_display);

    zmk_ble_ready(0);

    return 0;
}

static int zmk_ble_init(void) {
    int err = bt_enable(NULL);

    if (err < 0 && err != -EALREADY) {
        LOG_ERR("BLUETOOTH FAILED (%d)", err);
        return err;
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_register(&profiles_handler);
    k_work_init_delayable(&ble_save_work, ble_save_profile_work);
#else
    zmk_ble_complete_startup();
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY)

static bool zmk_ble_numeric_usage_to_value(const zmk_key_t key, const zmk_key_t one,
                                           const zmk_key_t zero, uint8_t *value) {
    if (key < one || key > zero) {
        return false;
    }

    *value = (key == zero) ? 0 : (key - one + 1);
    return true;
}

static int zmk_ble_handle_key_user(struct zmk_keycode_state_changed *event) {
    zmk_key_t key = event->keycode;

    LOG_DBG("key %d", key);

    if (!auth_passkey_entry_conn) {
        LOG_DBG("No connection for passkey entry");
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (event->state) {
        LOG_DBG("Key press, ignoring");
        return ZMK_EV_EVENT_HANDLED;
    }

    if (key == HID_USAGE_KEY_KEYBOARD_ESCAPE) {
        bt_conn_auth_cancel(auth_passkey_entry_conn);
        return ZMK_EV_EVENT_HANDLED;
    }

    if (key == HID_USAGE_KEY_KEYBOARD_RETURN || key == HID_USAGE_KEY_KEYBOARD_RETURN_ENTER) {
        uint8_t digits[PASSKEY_DIGITS];
        uint32_t count = ring_buf_get(&passkey_entries, digits, PASSKEY_DIGITS);

        uint32_t passkey = 0;
        for (int i = 0; i < count; i++) {
            passkey = (passkey * 10) + digits[i];
        }

        LOG_DBG("Final passkey: %d", passkey);
        bt_conn_auth_passkey_entry(auth_passkey_entry_conn, passkey);
        bt_conn_unref(auth_passkey_entry_conn);
        auth_passkey_entry_conn = NULL;
        return ZMK_EV_EVENT_HANDLED;
    }

    uint8_t val;
    if (!(zmk_ble_numeric_usage_to_value(key, HID_USAGE_KEY_KEYBOARD_1_AND_EXCLAMATION,
                                         HID_USAGE_KEY_KEYBOARD_0_AND_RIGHT_PARENTHESIS, &val) ||
          zmk_ble_numeric_usage_to_value(key, HID_USAGE_KEY_KEYPAD_1_AND_END,
                                         HID_USAGE_KEY_KEYPAD_0_AND_INSERT, &val))) {
        LOG_DBG("Key not a number, ignoring");
        return ZMK_EV_EVENT_HANDLED;
    }

    if (ring_buf_space_get(&passkey_entries) <= 0) {
        uint8_t discard_val;
        ring_buf_get(&passkey_entries, &discard_val, 1);
    }
    ring_buf_put(&passkey_entries, &val, 1);
    LOG_DBG("value entered: %d, digits collected so far: %d", val,
            ring_buf_size_get(&passkey_entries));

    return ZMK_EV_EVENT_HANDLED;
}

static int zmk_ble_listener(const zmk_event_t *eh) {
    struct zmk_keycode_state_changed *kc_state;

    kc_state = as_zmk_keycode_state_changed(eh);

    if (kc_state != NULL) {
        return zmk_ble_handle_key_user(kc_state);
    }

    return 0;
}

ZMK_LISTENER(zmk_ble, zmk_ble_listener);
ZMK_SUBSCRIPTION(zmk_ble, zmk_keycode_state_changed);
#endif /* IS_ENABLED(CONFIG_ZMK_BLE_PASSKEY_ENTRY) */

SYS_INIT(zmk_ble_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
