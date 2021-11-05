#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/temp_sensor.h"
#include "string.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "stdlib.h"

#define LED_1 1
#define LED_2 2
#define LED_3 3
#define BUTTON_1 42
#define BUTTON_2 41
#define BUTTON_3 36                  
#define CON2_1 37
#define CON2_2 40
#define CON1_1 38
#define CON1_2 39
#define M1_EN 7
#define M1_DIR 21
#define M2_EN 6
#define M2_DIR 26
#define NUM_LIDS 1

static const int RX_BUF_SIZE = 1024;

QueueHandle_t queue;

enum state
{
    state_open,
    state_closed,
    state_unknown,
    state_failed
};

enum direction
{
    reverse,
    forward
};

enum task
{
    task_open,
    task_close,
    task_stop
};

typedef struct
{
    u_int32_t pin;
    bool state;
    bool is_configured;
} endstop;

typedef struct
{
    u_int32_t dir_pin;
    u_int32_t en_pin;
    unsigned int speed;
    enum direction dir;
    endstop motor_closed;
    endstop motor_open;
} motor;

typedef struct
{
    enum state status;
    bool moving;
    motor motor1;
    motor motor2;
} lid;



void setMotor(motor *motor)
{
    gpio_pad_select_gpio(motor->dir_pin);
    gpio_pad_select_gpio(motor->en_pin);

    gpio_pulldown_dis(motor->dir_pin);
    gpio_pulldown_dis(motor->en_pin);

    gpio_pullup_dis(motor->dir_pin);
    gpio_pullup_dis(motor->en_pin);

    gpio_set_direction(motor->dir_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(motor->en_pin, GPIO_MODE_OUTPUT);

    switch (motor->speed)
    {
    case 100:
        gpio_set_level(motor->en_pin, 1);
        gpio_set_level(motor->dir_pin, motor->dir);
        break;

    case 0:
        gpio_set_level(motor->en_pin, 0);
        break;
    default:
        gpio_set_level(motor->en_pin, 0);
        break;
    }
}


void status_led_control(u_int32_t led, int value)

{
    gpio_pad_select_gpio(led);
    gpio_pulldown_dis(led);
    gpio_pullup_dis(led);
    gpio_set_direction(led, GPIO_MODE_OUTPUT);
    gpio_set_level(led, value);
}

void update_endstop_state(endstop * endstop)
{
    if(endstop->is_configured)
    {
        endstop->state = !gpio_get_level(endstop->state);
    }
}

void update_state(lid * lid)
{
    update_endstop_state(&lid->motor1.motor_closed);
    update_endstop_state(&lid->motor1.motor_open);
    update_endstop_state(&lid->motor2.motor_closed);
    update_endstop_state(&lid->motor2.motor_open);

    if (lid->motor1.motor_closed.state && lid->motor2.motor_closed.state)
    {
        lid->status = state_closed;
    } else if (lid->motor1.motor_open.state && lid->motor2.motor_open.state)
    {
        lid->status = state_open;
    } else if ((lid->motor1.motor_closed.state && lid->motor1.motor_open.state) || (lid->motor2.motor_closed.state && lid->motor2.motor_open.state))
    {
        lid->status = state_failed;
    } else {
        lid->status = state_unknown;
    }
}

void execute_lid_task(lid * lid,int task)
{
    if (task == task_stop)
    {
        lid->motor1.speed = 0;
        lid->motor2.speed = 0;
        setMotor(&lid->motor1);
        setMotor(&lid->motor2);
        lid->moving = false;
        update_state(lid);
    }
    if (task == task_open)
    {
        lid->motor1.dir = forward;
        lid->motor2.dir = forward;

        lid->motor1.speed = 100;
        setMotor(&lid->motor1);
        do {
            update_state(lid);
        } while (lid->motor1.motor_open.state);
        lid->motor1.speed = 0;
        setMotor(&lid->motor1);

        lid->motor2.speed = 100;
        setMotor(&lid->motor2);
        do {
            update_state(lid);
        } while (!lid->motor1.motor_open.state);
        lid->motor2.speed = 0;
        setMotor(&lid->motor2);
        lid->status = state_open;
    }
    if (task == task_open)
    {
        lid->motor1.dir = reverse;
        lid->motor2.dir = reverse;

        lid->motor1.speed = 100;
        setMotor(&lid->motor1);
        do {
            update_state(lid);
        } while (!lid->motor1.motor_closed.state);
        lid->motor1.speed = 0;
        setMotor(&lid->motor1);

        lid->motor2.speed = 100;
        setMotor(&lid->motor2);
        do {
            update_state(lid);
        } while (!lid->motor1.motor_closed.state);
        lid->motor2.speed = 0;
        setMotor(&lid->motor2);
        lid->status = state_closed;
    }

}

void write_uart(const char* data)
{
    const int len = strlen(data);
    uart_write_bytes(UART_NUM_1,data, len);

}


void readButtons(void *pVParameter)
{
    uint8_t *data = (uint8_t *) malloc(RX_BUF_SIZE);

    motor motor1;
    motor motor2;

    motor1.dir_pin = M1_DIR;
    motor1.en_pin = M1_EN;

    motor2.dir_pin = M2_DIR;
    motor2.en_pin = M2_EN;
    
    gpio_set_direction(BUTTON_1, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_2, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_3, GPIO_MODE_INPUT);
    gpio_set_direction(CON2_1, GPIO_MODE_INPUT);
    gpio_set_direction(CON2_2, GPIO_MODE_INPUT);

    gpio_pulldown_dis(CON2_1);
    gpio_pulldown_dis(CON2_2);

    gpio_pullup_dis(CON2_1);
    gpio_pullup_dis(CON2_2);

    while (1)
    {
        char sendQueue[] = "";

        if (sendQueue != "")
        {

        } else

        if (gpio_get_level(BUTTON_1) || *data == 'a')
        {
            sendQueue = "Button 1 pressed";
            motor1.dir = forward;
            motor2.dir = forward;
            motor1.speed = 100;
            motor2.speed = 100;
            setMotor(&motor1);
            setMotor(&motor2);
            status_led_control(LED_2, 1);
        }
        if (gpio_get_level(BUTTON_2) || *data == 'b')
        {
            sendQueue = "Button 2 pressed";
            motor1.speed = 100;
            motor2.speed = 100;
            motor1.dir = reverse;
            motor2.dir = reverse;
            setMotor(&motor1);
            setMotor(&motor2);
            status_led_control(LED_2, 1);
        }
        if (gpio_get_level(BUTTON_3) || *data == 'c')
        {
            sendQueue = "Button 3 pressed";
            motor1.speed = 0;
            motor2.speed = 0;
            setMotor(&motor1);
            setMotor(&motor2);
            status_led_control(LED_2, 0);
        }
        for (int i = 0; i < NUM_LIDS; i++)
        {
            lid lid;
            xQueueReceive(queue, &lid, (TickType_t) 0);
            if (*data == 'd') //open
            {
                execute_lid_task(&lid, task_open);
            }
            if (*data == 'e') //close
            {
                execute_lid_task(&lid, task_close);
            }
            if (*data == 'f') //stop
            {
                execute_lid_task(&lid, task_stop);
            }
        }
    }
}

void init_endstop(endstop * endstop)
{
    gpio_set_direction(endstop->pin, GPIO_MODE_INPUT);
    gpio_pulldown_dis(endstop->pin);
    gpio_pullup_dis(endstop->pin);
    endstop->is_configured = true;
    update_endstop_state(endstop);
}

void init_uart(int baudrate) {
    const uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,

    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0,0,NULL,0));

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, U0TXD_GPIO_NUM, U0RXD_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void app_main(void)
{
    queue = xQueueCreate(NUM_LIDS, sizeof(lid));

    if (queue == NULL)
    {
        printf("Error creating queue");
    }

    for(int i = 0; i < NUM_LIDS; i++)
    {
        motor motor1;
        motor motor2;

        motor1.dir_pin = M1_DIR;
        motor1.en_pin = M1_EN;

        motor2.dir_pin = M2_DIR;
        motor2.en_pin = M2_EN;

        lid lid1;
        lid1.motor1 = motor1;
        lid1.motor2 = motor2;
        lid1.moving = false;

        endstop end1;
        endstop end2;
        endstop end3;
        endstop end4;

        end1.pin = CON1_1;
        end2.pin = CON2_1;
        end3.pin = CON1_2;
        end4.pin = CON2_2;

        init_endstop(&end1);
        init_endstop(&end2);
        init_endstop(&end3);
        init_endstop(&end4);

        xQueueSend(queue, &lid1, (TickType_t) 0);
    }

    status_led_control(LED_1, 1);

    init_uart(115200);
    xTaskCreate(&readButtons, "readButtons", 1024*4, NULL, 5, NULL);
    //xTaskCreate(&read_endstops, "readEndstops", 1024, NULL, 5, NULL);
}
