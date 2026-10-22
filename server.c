/**
 * Raspberry Pi Pico 2 W - BLE LED & PWM Control Server
 * 
 * Bluetooth Low Energy (BLE) 서버 예제
 * - 온보드 LED 제어 (CYW43 GPIO) : Characteristic FF11 (0: OFF, 1: ON)
 * - 외부 핀 하드웨어 PWM 제어 : Characteristic FF12 (0 ~ 255 Duty Cycle)
 */

#include <stdio.h>
#include <string.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "temp_sensor.h"

// =========================================================================
// 사용자 설정 (Configuration)
// =========================================================================
#define HEARTBEAT_PERIOD_MS 1000
#define APP_AD_FLAGS 0x06

// 하드웨어 PWM 제어 핀 설정 (원하는 GPIO 핀으로 변경 가능)
#define PWM_GPIO_PIN 15
// PWM 해상도 (255 설정 시 0~255로 8비트 제어)
#define PWM_WRAP 255

// =========================================================================
// BLE Advertising 데이터
// - Device Name: "Pico2W-LED-PWM"
// - Service UUID: 0xFF10 (Custom Control Service)
// =========================================================================
static uint8_t adv_data[] = {
    // Flags
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // 16-bit Service UUID: 0xFF10 (리틀 엔디안: 0x10, 0xFF)
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x10, 0xFF,
    // Complete Local Name: "Pico2W-LED-PWM" (14글자)
    0x0F, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'P', 'i', 'c', 'o', '2', 'W', '-', 'L', 'E', 'D', '-', 'P', 'W', 'M'
};
static const uint8_t adv_data_len = sizeof(adv_data);

// =========================================================================
// 전역 변수 및 핸들러
// =========================================================================
static uint8_t led_state = 0;       // 0: OFF, 1: ON
static uint8_t pwm_duty = 0;        // 0 ~ 255
static uint pwm_slice_num;
static uint pwm_channel;

static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

extern uint8_t const profile_data[];

// =========================================================================
// PWM 및 LED 제어 함수
// =========================================================================
static void init_pwm(void) {
    gpio_set_function(PWM_GPIO_PIN, GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(PWM_GPIO_PIN);
    pwm_channel = pwm_gpio_to_channel(PWM_GPIO_PIN);

    // RP2350 클럭 기준 150MHz / 150 = 1MHz 클럭 카운터
    // wrap = 255 -> 약 3.9kHz 주파수 (LED 깜빡임 없이 부드러운 밝기 제어)
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 150.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(pwm_slice_num, &config, true);

    // 초기 듀티비 0 (OFF)
    pwm_set_chan_level(pwm_slice_num, pwm_channel, 0);
    printf("[INIT] PWM initialized on GPIO %d (Slice %u, Channel %u, Freq ~3.9kHz)\n", 
           PWM_GPIO_PIN, pwm_slice_num, pwm_channel);
}

static void set_pwm_duty(uint8_t duty) {
    pwm_duty = duty;
    pwm_set_chan_level(pwm_slice_num, pwm_channel, duty);
}

static void set_led_state(uint8_t state) {
    led_state = state ? 1 : 0;
    // Pico 2 W의 온보드 LED는 CYW43 무선 칩을 통해 제어
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
}

// =========================================================================
// BTstack BLE 이벤트 처리
// =========================================================================
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // Advertising 파라미터 설정 (간격 약 500ms)
            uint16_t adv_int_min = 800;
            uint16_t adv_int_max = 800;
            uint8_t adv_type = 0; // Connectable undirected advertising
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            assert(adv_data_len <= 31);
            gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
            gap_advertisements_enable(1);
            printf("BLE Advertising started. Device Name: 'Pico2W-LED-PWM'\n");
            break;

        case HCI_EVENT_CONNECTION_COMPLETE:
            con_handle = hci_subevent_connection_complete_get_connection_handle(packet);
            printf(">> BLE Client connected! (Handle: 0x%04x)\n", con_handle);
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf(">> BLE Client disconnected.\n");
            con_handle = HCI_CON_HANDLE_INVALID;
            // 연결 해제 시 다시 광고 재개
            gap_advertisements_enable(1);
            break;

        default:
            break;
    }
}

// =========================================================================
// GATT Read Callback (클라이언트가 특성 값을 읽으려 할 때)
// =========================================================================
static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    // 온보드 LED 상태 (UUID 0xFF11)
    if (att_handle == ATT_CHARACTERISTIC_FF11_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)&led_state, sizeof(led_state), offset, buffer, buffer_size);
    }
    // PWM 듀티비 (UUID 0xFF12)
    if (att_handle == ATT_CHARACTERISTIC_FF12_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)&pwm_duty, sizeof(pwm_duty), offset, buffer, buffer_size);
    }
    return 0;
}

// =========================================================================
// GATT Write Callback (클라이언트가 값을 쓸 때)
// =========================================================================
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(connection_handle);

    // 온보드 LED 제어 (UUID 0xFF11): 1바이트 (0: OFF, 1: ON)
    if (att_handle == ATT_CHARACTERISTIC_FF11_01_VALUE_HANDLE) {
        if (buffer_size >= 1) {
            set_led_state(buffer[0]);
            printf("[BLE WRITE] On-board LED -> %s (%d)\n", led_state ? "ON" : "OFF", led_state);
        }
        return 0;
    }

    // PWM 듀티비 제어 (UUID 0xFF12): 1바이트 (0 ~ 255)
    if (att_handle == ATT_CHARACTERISTIC_FF12_01_VALUE_HANDLE) {
        if (buffer_size >= 1) {
            set_pwm_duty(buffer[0]);
            printf("[BLE WRITE] PWM (GPIO %d) Duty -> %u / 255 (%.1f%%)\n",
                   PWM_GPIO_PIN, pwm_duty, (pwm_duty * 100.0f) / 255.0f);
        }
        return 0;
    }

    return 0;
}

// =========================================================================
// 하트비트 타이머 핸들러 (연결 유지 및 주기적 백그라운드 작업)
// =========================================================================
static void heartbeat_handler(struct btstack_timer_source *ts) {
    // 기존 예제에서는 여기서 LED를 계속 반전(toggle)시켰으나,
    // 이제는 BLE를 통해 사용자가 직접 LED를 제어하므로 건드리지 않습니다.
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

// 키보드 's' 입력으로 블루투스를 중지/재시작할 수 있는 기능 유지
static volatile bool key_pressed;
void key_pressed_func(void *param) {
    int key = getchar_timeout_us(0);
    if (key == 's' || key == 'S') {
        key_pressed = true;
    }
}

// =========================================================================
// Main 진입점
// =========================================================================
int main() {
    stdio_init_all();

restart:
    // CYW43 드라이버 초기화 (Pico 2 W 무선 및 블루투스)
    if (cyw43_arch_init()) {
        printf("Failed to initialise cyw43_arch\n");
        return -1;
    }

    // PWM 핀 초기화
    init_pwm();

    // 온보드 LED 초기 상태 OFF
    set_led_state(0);

    // 콘솔 안내
    printf("==========================================\n");
    printf(" Pico 2 W BLE LED & PWM Server Started\n");
    printf(" - On-board LED: Characteristic 0xFF11 (0 or 1)\n");
    printf(" - PWM Pin GPIO %d: Characteristic 0xFF12 (0 ~ 255)\n", PWM_GPIO_PIN);
    printf(" Press 'S' key on serial to Stop bluetooth\n");
    printf("==========================================\n");
    stdio_set_chars_available_callback(key_pressed_func, NULL);

    // BTstack L2CAP & Security Manager 초기화
    l2cap_init();
    sm_init();

    // GATT 프로파일 및 콜백 등록
    att_server_init(profile_data, att_read_callback, att_write_callback);

    // HCI 이벤트 핸들러 등록
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // ATT 서버 패킷 핸들러 등록
    att_server_register_packet_handler(packet_handler);

    // 하트비트 타이머 시작
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);

    // 블루투스 파워 ON
    hci_power_control(HCI_POWER_ON);

    // 이벤트 루프
    key_pressed = false;
    while (!key_pressed) {
        async_context_poll(cyw43_arch_async_context());
        async_context_wait_for_work_until(cyw43_arch_async_context(), at_the_end_of_time);
    }

    // 블루투스 정리 및 종료
    att_server_deinit();
    sm_deinit();
    l2cap_deinit();
    cyw43_arch_deinit();

    printf("Press 'S' key to Start bluetooth again\n");
    key_pressed = false;
    while (!key_pressed) {
        sleep_ms(1000);
    }
    goto restart;

    return 0;
}
