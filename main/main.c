#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/uart.h"

#define TASK_PRIORITY_UART_EVENTS       10
#define TAG		"Arhiled Soft"

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define BUF_SIZE (1024)

SemaphoreHandle_t xSemaphore = NULL;
static QueueHandle_t uart0_queue;
esp_timer_handle_t timer;


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    char* dtmp = (char*) malloc(BUF_SIZE);
    char uart_tmp[128]; // temporary buffer to store uart readings
    int bytes_read = 0;

	if( xSemaphore != NULL )
	{
		if( xSemaphoreTake( xSemaphore, 0 ) == pdTRUE )
		{

			ESP_LOGI( TAG, "UART Polling" );

			// Check if something is in the uart buffer and read from until done
			if(xQueuePeek(uart0_queue, (void * )&event, 0)){

				switch(event.type) {
					case UART_DATA:
						bzero(dtmp, BUF_SIZE);
						while(xQueueReceive(uart0_queue, (void * )&event, 0)){
							// this should always be event.size < 128
							uart_read_bytes(EX_UART_NUM, uart_tmp, event.size, 0);
							// TODO - add check for buffer overflow error
							bytes_read += event.size;
							strcat( dtmp, uart_tmp );

						}

						ESP_LOGI( TAG, "FINAL BUFFER : %d", bytes_read );
						uart_write_bytes(EX_UART_NUM, (const char*) dtmp, bytes_read);


						break;
					default:
						break;
				}

			}
			xSemaphoreGive( xSemaphore );
		}
	}


    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void init_uart_task(){
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB // UART_SCLK_REF_TICK
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

}

static void timer_func(void* arg) {
	// ESP_LOGI( TAG, "Timer2:" );

    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, TASK_PRIORITY_UART_EVENTS, NULL);
}

void init_timer(int timer_period_us) {
    int64_t t_end;

    esp_timer_create_args_t args = {
            .callback = &timer_func,
            .arg = &t_end,
            .name = "timer"
    };
    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, timer_period_us);
}


void app_main(void)
{

	// Initialize  semaphore
	xSemaphore = xSemaphoreCreateMutex();

	init_uart_task();

    // One seconds timer
    init_timer(1 * 1000 * 1000); // 1uS * 1000 * 1000 = 1s
}






