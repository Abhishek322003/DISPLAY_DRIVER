#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_system.h"
#include "esp_log.h"
#include "inttypes.h"
static const char *TAG = "EV_CHARGER";
 
// ==================== DWIN VP Addresses ====================
#define START_STOP_ADDR      0x1000
#define VOLTAGE_ADDR         0x4000
#define CURRENT_ADDR         0x4100
#define POWER_ADDR           0x4200
#define ENERGY_ADDR          0x4300
#define SOC_ADDR             0x4400
#define TEMP_ADDR            0x4500
#define  DURATION            0x5000
#define  UNIT_RATE           0x5100
#define AMOUNT               0x5200
#define  CHARGING_POWER      0x5300
#define  ENERGY_DELIVERD     0x5400
#define TOTAL_ENERGY         0x5500
#define CHARGE_PERC_ADDR     0x3000
 
 
#define PAGE_SWITCH_ADDR     0x0085
#define PAGE_SWITCH_INITADDR 0x0084
 
// ==================== CAN Config ====================
#define CAN_TX_GPIO          GPIO_NUM_4
#define CAN_RX_GPIO          GPIO_NUM_5
#define EMERGENCY_PIN        GPIO_NUM_3
#define CAN_ID_DISPLAY       0x100   // DWIN → ESP32
#define CAN_ID_CHARGER       0x100   // ESP32 → DWIN
 
// ==================== CP Voltage Thresholds ====================
#define CP_GUN_MIN_V   1.55f
#define CP_GUN_MAX_V   1.70f
#define CP_IDLE_MIN_V  1.80f
#define CP_EMERG_MAX_V 1.20f
 
// ==================== States ====================
static uint32_t GUN_CONNECTED      = 0;
static uint32_t READY_FOR_CHARGE   = 0;
static uint32_t CHARGE_ACTIVE      = 0;
static uint32_t EMERGENCY_STOP     = 0;
static uint8_t  DISPLAY_START_STOP = 0;  // 1=start, 0=stop
static float new_val=1.8f;
 
// ==================== Energy Meter Placeholder ====================
typedef struct {
    float voltage;
    float current;
    float power;
    float energy_kWh;
    float soc;
    float tempC;
} meter_data_t;
 
static meter_data_t g_meter;
static float volts_cp;
 
// ==================== CAN Transmit ====================
static void dwin_can_write(uint16_t vp_addr, uint16_t value)
{
	if (EMERGENCY_STOP) return;
    twai_message_t msg = {0};
    msg.identifier = CAN_ID_CHARGER;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 6;
 
    msg.data[0] = 0x05;
    msg.data[1] = 0x82;
    msg.data[2] = (vp_addr >> 8) & 0xFF;
    msg.data[3] = vp_addr & 0xFF;
    msg.data[4] = (value >> 8) & 0xFF;
    msg.data[5] = value & 0xFF;
 
    if (twai_transmit(&msg, pdMS_TO_TICKS(200)) == ESP_OK)
        ESP_LOGI(TAG, "[TX] VP=0x%04X → %u", vp_addr, value);
   
}
 
static void send_can_feedback(uint8_t status)
{
	if (EMERGENCY_STOP) return;
    twai_message_t tx = {0};
    tx.identifier = CAN_ID_CHARGER;
    tx.data_length_code = 8;
    tx.extd = 0;
    tx.rtr = 0;
    tx.data[0] = 0x06;
    tx.data[1] = 0x83;
    tx.data[2] = 0x20;
    tx.data[3] = 0x00;
    tx.data[4] = status;
    twai_transmit(&tx, pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Feedback 0x%02X sent", status);
}
 
// ==================== Push Meter Data ====================
// ==================== Push Meter Data ====================
static void push_meter_to_dwin(const meter_data_t *m)
{
    if (DISPLAY_START_STOP == 0 ||EMERGENCY_STOP) {
        ESP_LOGW(TAG, "Charger stopped — skipping display updates.");
        dwin_can_write(PAGE_SWITCH_ADDR, 0x0004);   //unplugged
        dwin_can_write(PAGE_SWITCH_INITADDR, 0x5A01);
        vTaskDelay(pdMS_TO_TICKS(2000));
                // Gun stays logically "connected" only if voltage indicates so later
        dwin_can_write(PAGE_SWITCH_ADDR, 0x0001);   // Switch to charge complete page
        dwin_can_write(PAGE_SWITCH_INITADDR, 0x5A01);
        
        
        return; // Don't update if charging is not active
    }
 
    // Clamp SOC to 100%
    uint16_t sc = rand() %100;
 
    // Convert to 16-bit DWIN scaled values
    uint16_t v  = (uint16_t)(m->voltage * 10.0f);
    uint16_t i  = (uint16_t)(m->current * 100.0f);
    uint16_t p  = (uint16_t)(m->power * 0.1f);
    uint16_t e  = rand() %100;
    uint16_t tc = (uint16_t)(m->tempC);
 
    // Additional dynamic data
    static uint16_t duration = 0;
    static uint16_t unit_rate = 20;  // Rs/kWh fixed
    uint16_t amt = rand()%1000; // total charge cost
    uint16_t delivered = (uint16_t)(m->energy_kWh * 100.0f); // energy in Wh*100
 
    // === Send all display updates ===
    dwin_can_write(VOLTAGE_ADDR, sc+2);
    dwin_can_write(CURRENT_ADDR, sc+1);
    dwin_can_write(POWER_ADDR, p);
    dwin_can_write(ENERGY_ADDR, e);
    dwin_can_write(SOC_ADDR, sc);
    dwin_can_write(TEMP_ADDR, tc);
    dwin_can_write(CHARGE_PERC_ADDR , sc);
    dwin_can_write(CHARGING_POWER, p);
    dwin_can_write(ENERGY_DELIVERD, delivered);
    dwin_can_write(UNIT_RATE, unit_rate);
    dwin_can_write(AMOUNT, amt);
    dwin_can_write(DURATION, ++duration); // seconds elapsed
    dwin_can_write(TOTAL_ENERGY, e);
 
    // Auto stop when SOC reaches 100%
    if (sc >= 100) {
        ESP_LOGW(TAG, "⚡ Charging complete! SOC reached 100%%");
        DISPLAY_START_STOP = 0;
        CHARGE_ACTIVE = 0;
        READY_FOR_CHARGE = 0;
        send_can_feedback(0x00);
        dwin_can_write(PAGE_SWITCH_ADDR, 0x0005);   // Switch to charge complete page
        dwin_can_write(PAGE_SWITCH_INITADDR, 0x5A01);
    }
}
 
 
// ==================== CAN Init ====================
static void can_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN started");
}
 
// ==================== Energy Simulation ====================
// ==================== Energy Simulation (Random Dynamic Values) ====================
static void read_energy_meter(meter_data_t *out)
{
    static float energy = 0.0f;
 
    if (CHARGE_ACTIVE) {
        // Random variation to make charging dynamic
        float voltage_variation = ((float)(rand() % 40) - 20) / 10.0f; // ±2.0 V variation
        float current_variation = ((float)(rand() % 30) - 15) / 10.0f; // ±1.5 A variation
 
        out->voltage = 230.0f + voltage_variation;
        out->current = 10.0f + current_variation;
    } else {
        out->voltage = 230.0f;
        out->current = 0.0f;
    }
 
    // Power and energy calculation
    out->power = out->voltage * out->current * 0.98f;
 
    if (CHARGE_ACTIVE) {
        // Energy added per update interval (2 s)
        energy += (out->power / 1000.0f) * (2.0f / 3600.0f);
    }
 
    // Update metrics
    out->energy_kWh = energy;
    out->soc = (energy / 60.0f) * 100.0f;  // assuming 60 kWh battery capacity
    if (out->soc > 100.0f) out->soc = 100.0f;
    out->tempC = 30.0f + (rand() % 5);     // 30–34 °C random temperature
}
 
// ==================== CP Voltage via Serial ====================
// ==================== CP Voltage via Serial ====================
static float read_voltage_from_serial(void)
{
	 if (EMERGENCY_STOP) return -999.0f;
    // Read only when user enters a line; non-blocking otherwise
    char buf[16] = {0};
    int i = 0, c;
   
 
    while ((c = getchar()) != EOF) {
        if (c == '\n' || c == '\r') break;
        if (i < sizeof(buf) - 1) buf[i++] = (char)c;
    }
 
    if (i > 0) {
        float v = atof(buf);
        ESP_LOGI(TAG, "📥 New CP Voltage from Serial = %.2fV", v);
        return v;  // Valid user input
    }
 
    // No new input — return special flag
    return -999.0f;
}
 
 
// ==================== CAN Receive Handler ====================
// ==================== CAN Receive Handler (with auto restart logic) ====================
static void check_can_rx(void)
{
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        ESP_LOGI(TAG, "[RX] ID=0x%03X DLC=%d", (unsigned int)rx_msg.identifier, rx_msg.data_length_code);
        for (int i = 0; i < rx_msg.data_length_code; i++)
            printf("%02X ", rx_msg.data[i]);
        printf("\n");
 
        // Check for feedback from DWIN → START/STOP commands
        if (rx_msg.data[2] == 0x10 && rx_msg.data[3] == 0x00 &&
            rx_msg.identifier == CAN_ID_DISPLAY && rx_msg.data_length_code >= 7)
        {
            if (rx_msg.data[6] == 0x0A) {   // Start / Ready for Charge
                DISPLAY_START_STOP = 1;
                READY_FOR_CHARGE = 1;
                ESP_LOGI(TAG, "✅ Received READY FOR CHARGE from DWIN");
            }
            else if (rx_msg.data[6] == 0x00) {  // Stop command
    DISPLAY_START_STOP = 0;
    READY_FOR_CHARGE   = 0;
    CHARGE_ACTIVE      = 0;
    GUN_CONNECTED      = 0;
 
    ESP_LOGW(TAG, "🛑 STOP received → Returning to Idle Page, waiting for new voltage input...");
    send_can_feedback(0x00);
 
    dwin_can_write(PAGE_SWITCH_ADDR,     0x0001);
    dwin_can_write(PAGE_SWITCH_INITADDR, 0x5A01);
 
    // ⬇️ Add these two lines
    new_val  = -999.0f; // main loop won’t overwrite volts_cp until user types
    // IMPORTANT: also move current CP to a non-connecting value
    // make this variable global or file-static if needed
           // or move volts_cp to file-scope
    volts_cp = CP_IDLE_MIN_V;    // e.g., 1.80f (outside gun-connected window)
}
 
 
 
        }
    }
}
 
 
// ==================== MAIN ====================
void app_main(void)
{
    ESP_LOGI(TAG, "EV Charger Simulation Started (Waits for DWIN Feedback)");
    can_init();
    gpio_set_direction(EMERGENCY_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(EMERGENCY_PIN, GPIO_PULLUP_ONLY);
 
 
    TickType_t last_push = 0;
    const TickType_t push_period = pdMS_TO_TICKS(2000);
    volts_cp = 1.85f;  // idle
    dwin_can_write(PAGE_SWITCH_ADDR, 0x0000);
    dwin_can_write(PAGE_SWITCH_INITADDR, 0x5A01);
    vTaskDelay(pdMS_TO_TICKS(3000));
    dwin_can_write(PAGE_SWITCH_ADDR, 0x0001);
    dwin_can_write(PAGE_SWITCH_INITADDR, 0x5A01);
 
    while (1)
    {
        check_can_rx();  // check for start/stop feedback
 
        float v_read = read_voltage_from_serial();
       if (v_read != -999.0f)  // Only update if actual serial data received
             volts_cp = v_read;
 
 
        // === Emergency ===
        int flag=0;
        if (gpio_get_level(EMERGENCY_PIN) == 0) {  // Pressed = LOW
            if (!EMERGENCY_STOP ) {
				dwin_can_write(0x0085, 0x0007);
                dwin_can_write(0x0084, 0x5A01);
                vTaskDelay(pdMS_TO_TICKS(2000));
                EMERGENCY_STOP = 1;
                CHARGE_ACTIVE = 0;
                flag=1;
                DISPLAY_START_STOP = 0;
                READY_FOR_CHARGE = 0;
                ESP_LOGE(TAG, "🚨 Emergency switch pressed! Stopping charge immediately.");
                send_can_feedback(0x03);
                
            }
            else if(EMERGENCY_STOP ){
		   // dwin_can_write(0x0085, 0x0007);
            //dwin_can_write(0x0084, 0x5A01);
            // Released → resume normal state (but must reconnect again)
            //ESP_LOGI(TAG, "🟢 Emergency switch not released. Waiting for new gun connection...");
            
            GUN_CONNECTED = 0;
            volts_cp = CP_IDLE_MIN_V;
           }
        }
        // === Gun Connected ===
        else if (volts_cp >= CP_GUN_MIN_V && volts_cp < CP_GUN_MAX_V) {
            if (!GUN_CONNECTED) {
                GUN_CONNECTED = 1;
                EMERGENCY_STOP = 0;
                ESP_LOGI(TAG, "Gun Connected, waiting for READY (%.2fV)", volts_cp);
                dwin_can_write(0x0085, 0x0002);
                dwin_can_write(0x0084, 0x5A01);
                vTaskDelay(pdMS_TO_TICKS(2000));
                dwin_can_write(0x0085, 0x0003);
                dwin_can_write(0x0084, 0x5A01);
                send_can_feedback(0x02);
                
            }
        }
        // === Gun Removed ===
        else if (volts_cp >= CP_IDLE_MIN_V) {
            if (GUN_CONNECTED || CHARGE_ACTIVE || EMERGENCY_STOP) {
                GUN_CONNECTED = 0;
                READY_FOR_CHARGE = 0;
                CHARGE_ACTIVE = 0;
                EMERGENCY_STOP = 0;
                ESP_LOGI(TAG, "Gun Removed → Idle (%.2fV)", volts_cp);
                dwin_can_write(0x0085, 0x0000);
                dwin_can_write(0x0084, 0x5A01);
                vTaskDelay(pdMS_TO_TICKS(2000));
                send_can_feedback(0x00);
            }
        }
 
        // === Normal Charging ===
        if (GUN_CONNECTED && !EMERGENCY_STOP && DISPLAY_START_STOP == 1) {
            CHARGE_ACTIVE = 1;
 
            if (xTaskGetTickCount() - last_push >= push_period) {
                read_energy_meter(&g_meter);
                push_meter_to_dwin(&g_meter);
                ESP_LOGI(TAG, "Charging... V=%.1f I=%.2f E=%.3f kWh SoC=%.1f%%",
                         g_meter.voltage, g_meter.current, g_meter.energy_kWh, g_meter.soc);
                last_push = xTaskGetTickCount();
            }
        }
 
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
 