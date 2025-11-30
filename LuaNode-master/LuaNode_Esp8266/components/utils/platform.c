#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "platform.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
#include "driver/rtc_io.h"
#endif

#define TAG		"platform"

#if SOC_RTC_FAST_MEM_SUPPORTED
static RTC_DATA_ATTR struct timeval sleep_enter_time;
#else
static struct timeval sleep_enter_time;
#endif

void lua_node_restart()
{
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
	esp_restart();
#elif (CURRENT_PLATFORM == NODE_PLATFORM_ESP8266)
	esp_restart();
#else
	ESP_LOGE(TAG, "Not define CURRENT_PLATFORM!");
#endif
}

#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
static void deep_sleep_task(void *args)
{

    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        } 
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

#if CONFIG_IDF_TARGET_ESP32
    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
#endif
}

static void deep_sleep_register_rtc_timer_wakeup(uint64_t us)
{
    //printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(us));
	xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);
	// enter deep sleep
    esp_deep_sleep_start();
}
#endif

void lua_node_sleep(uint64_t us)
{
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
	//system_deep_sleep (us);
	deep_sleep_register_rtc_timer_wakeup(us);
#elif (CURRENT_PLATFORM == NODE_PLATFORM_ESP8266)
	esp_deep_sleep(us);
#else
	ESP_LOGE(TAG, "Not define CURRENT_PLATFORM!");
#endif
}

uint32_t lua_node_get_heap_size(void)
{
	uint32_t heap_size = 0;
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
	heap_size = esp_get_minimum_free_heap_size();
#elif (CURRENT_PLATFORM == NODE_PLATFORM_ESP8266)
	heap_size = esp_get_free_heap_size();
#else
	ESP_LOGE(TAG, "Not define CURRENT_PLATFORM!");
#endif
	return heap_size;
}

void lua_node_system_restore(void)
{
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP8266)
	//system_restore();
#endif
}

//////////////////////////////////////////////////
#define GPIO_OUTPUT_PIN_SEL(x)  ((1ULL<<x))

extern void gpio_isr_handler(void *arg);
int platform_gpio_mode(int pin, int mode, int type)
{
	//ESP_LOGI(TAG, "pin:%d, mode:%d, type:%d", pin, mode, type);
	gpio_config_t io_conf = {};
    //disable interrupt 
    io_conf.intr_type = type;
    //default set as output mode
	io_conf.mode = mode;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL(pin); 
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
	
	gpio_config(&io_conf);
	
	if (mode == GPIO_MODE_INPUT && type > GPIO_INTR_DISABLE) {
		//ESP_LOGI(TAG, "Add intr to pin: %d", pin);
		gpio_install_isr_service(0);
		gpio_isr_handler_remove(pin);
		gpio_isr_handler_add(pin, gpio_isr_handler, (void *) pin);
	}
	
	return ESP_OK;
}

void platform_gpio_isr_uninstall(void)
{
	
}

/**
 * return   -1 error.
 */
int platform_gpio_read(int pin)
{
	if (!GPIO_IS_VALID_GPIO(pin)) {
		ESP_LOGE(TAG, "Not a valid pin");
		return -1;
	}
	int level = gpio_get_level(pin);
	//ESP_LOGI(TAG, "read pin:%d, level:%d", pin, level);
	return level;
}

void platform_gpio_write(int pin, int level)
{
#if (CURRENT_PLATFORM == NODE_PLATFORM_ESP32)
	gpio_set_level(pin, level);
#elif (CURRENT_PLATFORM == NODE_PLATFORM_ESP8266)
	gpio_set_level(pin, level);
#else
	ESP_LOGE(TAG, "Not support platform");
#endif
}