#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ssd1306.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include <math.h>
#include "driver/dac_oneshot.h"

#define LED1_GPIO GPIO_NUM_3
#define LED2_GPIO GPIO_NUM_1
#define LED3_GPIO GPIO_NUM_23
#define PWM1_GPIO GPIO_NUM_32
#define PWM2_GPIO GPIO_NUM_33
#define PWM3_GPIO GPIO_NUM_25
#define BUTTON_GPIO GPIO_NUM_13

#define UART_TX_GPIO GPIO_NUM_5
#define UART_PORT_NUM UART_NUM_1
#define UART_BAUD_RATE 115200

#define UART2_TX_GPIO GPIO_NUM_18
#define UART2_PORT_NUM UART_NUM_2
#define UART2_BAUD_RATE 9600

#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define OLED_ADDR 0x3C

static const char *TAG = "OSCILLOSCOPE_TEST_TASK";

#define POT_AVG_SAMPLES 10

static int pot_samples[POT_AVG_SAMPLES] = {0};
static int pot_index = 0;
int pot_value = 0;

#define SINE_GPIO GPIO_NUM_26

SSD1306_t oled_dev; // Global OLED device handle

void oled_init()
{
    // If your display does NOT have a reset pin, pass -1 for reset
    i2c_master_init(&oled_dev, I2C_SDA_GPIO, I2C_SCL_GPIO, -1);
    ssd1306_init(&oled_dev, 128, 64);
    ssd1306_clear_screen(&oled_dev, false);
    ssd1306_contrast(&oled_dev, 0xFF);
    ssd1306_display_text(&oled_dev, 0, "Button Time:", 12, false);
}

void display_button_time(int64_t duration_ms)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "Time: %lld ms", duration_ms);
    ssd1306_clear_line(&oled_dev, 2, false);
    ssd1306_display_text(&oled_dev, 2, buf, strlen(buf), false);
}

void uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_driver_install(UART_PORT_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart2_init()
{
    const uart_config_t uart2_config = {
        .baud_rate = UART2_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_driver_install(UART2_PORT_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(UART2_PORT_NUM, &uart2_config);
    uart_set_pin(UART2_PORT_NUM, UART2_TX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void pot_task(void *pvParameter)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &config); // GPIO39

    int sum = 0;

    while (1)
    {
        int val = 0;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &val);

        // Update moving average buffer
        sum -= pot_samples[pot_index];
        pot_samples[pot_index] = val;
        sum += val;
        pot_index = (pot_index + 1) % POT_AVG_SAMPLES;

        // Calculate average
        pot_value = sum / POT_AVG_SAMPLES;

        char pot_buf[24];
        snprintf(pot_buf, sizeof(pot_buf), "Pot Value: %d", pot_value);
        ssd1306_clear_line(&oled_dev, 3, false);
        ssd1306_display_text(&oled_dev, 3, pot_buf, strlen(pot_buf), false);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void uart2_task(void *pvParameter)
{
    const char *msg = "Hello, World!\r\n";
    while (1)
    {
        uart_write_bytes(UART2_PORT_NUM, msg, strlen(msg));
        ESP_LOGI(TAG, "%s", msg);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void led_task_1(void *pvParameter)
{
    while (1)
    {
        gpio_set_level(LED1_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED1_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void led_task_2(void *pvParameter)
{
    while (1)
    {
        gpio_set_level(LED2_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(330));
        gpio_set_level(LED2_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(330));
    }
}

void led_task_3(void *pvParameter)
{
    while (1)
    {
        gpio_set_level(LED3_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(LED3_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// PWM task for channel 0, duty 25%
void pwm_task_1(void *pvParameter)
{
    while (1)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 256); // 25% of 1023
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// PWM task for channel 1, duty 50%
void pwm_task_2(void *pvParameter)
{
    while (1)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 512); // 50% of 1023
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// PWM task for channel 2, duty 75%
void pwm_task_3(void *pvParameter)
{
    while (1)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 768); // 75% of 1023
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void button_task(void *pvParameter)
{
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    int last_state = 1;
    int pressed = 0;
    int64_t press_time = 0;

    while (1)
    {
        int state = gpio_get_level(BUTTON_GPIO);
        if (last_state == 1 && state == 0)
        {
            // Button pressed
            pressed = 1;
            press_time = esp_timer_get_time();
        }
        else if (last_state == 0 && state == 1 && pressed)
        {
            // Button released
            int64_t release_time = esp_timer_get_time();
            int64_t duration_ms = (release_time - press_time) / 1000;
            ESP_LOGI(TAG, "Button pressed for %lld ms", duration_ms);
            display_button_time(duration_ms);
            // Send message over UART
            char uart_buf[32];
            snprintf(uart_buf, sizeof(uart_buf), "%lld ms\r\n", duration_ms);
            uart_write_bytes(UART_PORT_NUM, uart_buf, strlen(uart_buf));

            pressed = 0;
        }
        last_state = state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void sine_wave_task(void *pvParameter)
{
    dac_oneshot_handle_t dac_handle;
    dac_oneshot_config_t dac_cfg = {
        .chan_id = DAC_CHAN_1 // GPIO26
    };
    dac_oneshot_new_channel(&dac_cfg, &dac_handle);

    const int sine_points = 64;
    float sine_table[sine_points];
    for (int i = 0; i < sine_points; i++)
    {
        sine_table[i] = sinf(2 * M_PI * i / sine_points);
    }

    while (1)
    {
        int min_period_ms = 5;
        int max_period_ms = 100;
        int min_amp = 20;
        int max_amp = 120;

        int pv = pot_value;
        if (pv < 0)
            pv = 0;
        if (pv > 4095)
            pv = 4095;

        int period_ms = max_period_ms - ((max_period_ms - min_period_ms) * pv) / 4095;
        int amplitude = max_amp - ((max_amp - min_amp) * pv) / 4095;
        int offset = 128;

        for (int i = 0; i < sine_points; i++)
        {
            int raw_val = offset + (int)(amplitude * sine_table[i]);
            if (raw_val < 0)
                raw_val = 0;
            if (raw_val > 255)
                raw_val = 255;
            uint8_t dac_val = (uint8_t)raw_val;
            dac_oneshot_output_voltage(dac_handle, dac_val);

            vTaskDelay(pdMS_TO_TICKS(period_ms) / sine_points);
        }
    }
}

void app_main(void)
{
    uart_init();
    uart2_init();

    // Configure LED GPIOs
    gpio_reset_pin(LED1_GPIO);
    gpio_set_direction(LED1_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED2_GPIO);
    gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED3_GPIO);
    gpio_set_direction(LED3_GPIO, GPIO_MODE_OUTPUT);

    // Configure PWM channels
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[3] = {
        {.channel = LEDC_CHANNEL_0,
         .duty = 0,
         .gpio_num = PWM1_GPIO,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_0},
        {.channel = LEDC_CHANNEL_1,
         .duty = 0,
         .gpio_num = PWM2_GPIO,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_0},
        {.channel = LEDC_CHANNEL_2,
         .duty = 0,
         .gpio_num = PWM3_GPIO,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_0}};

    for (int ch = 0; ch < 3; ch++)
    {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize OLED
    oled_init();

    // Start LED tasks
    xTaskCreate(&led_task_1, "LED1_1000ms", 2048, NULL, 5, NULL);
    xTaskCreate(&led_task_2, "LED2_330ms", 2048, NULL, 5, NULL);
    xTaskCreate(&led_task_3, "LED3_50ms", 2048, NULL, 5, NULL);

    // Start PWM tasks
    xTaskCreate(&pwm_task_1, "PWM1_25", 2048, NULL, 5, NULL);
    xTaskCreate(&pwm_task_2, "PWM2_50", 2048, NULL, 5, NULL);
    xTaskCreate(&pwm_task_3, "PWM3_75", 2048, NULL, 5, NULL);

    xTaskCreate(&button_task, "ButtonTask", 2048, NULL, 5, NULL);

    xTaskCreate(&uart2_task, "UART2_Task", 2048, NULL, 5, NULL);

    xTaskCreate(&pot_task, "PotTask", 2048, NULL, 5, NULL);

    xTaskCreate(&sine_wave_task, "SineWaveTask", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Application started");
}