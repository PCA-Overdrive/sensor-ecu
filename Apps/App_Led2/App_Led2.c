#include "App_Led2.h"

#include "IfxPort.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static QueueHandle_t g_led2Queue = NULL;

static void Led2_on(void)
{
    IfxPort_setPinLow(LED2_PORT, LED2_PIN);
}

static void Led2_off(void)
{
    IfxPort_setPinHigh(LED2_PORT, LED2_PIN);
}

void AppLed2_init(void)
{
    IfxPort_setPinMode(LED2_PORT, LED2_PIN, IfxPort_Mode_outputPushPullGeneral);
    Led2_off();

    if (g_led2Queue == NULL)
    {
        g_led2Queue = xQueueCreate(LED2_QUEUE_LENGTH, sizeof(Led2BlinkRequest_t));
    }
}

void AppLed2_blink(uint32_t count, uint32_t periodMs)
{
    Led2BlinkRequest_t req;

    if (count == 0u)
    {
        return;
    }

    if (periodMs == 0u)
    {
        periodMs = LED2_DEFAULT_PERIOD_MS;
    }

    req.count = count;
    req.periodMs = periodMs;

    if (g_led2Queue != NULL)
    {
        (void)xQueueSend(g_led2Queue, &req, 0u);
    }
}

void task_app_led2(void *arg)
{
    Led2BlinkRequest_t req;
    uint32_t i;

    (void)arg;

    AppLed2_init();

    for (;;)
    {
        if (xQueueReceive(g_led2Queue, &req, portMAX_DELAY) == pdTRUE)
        {
            for (i = 0u; i < req.count; i++)
            {
                Led2_on();
                vTaskDelay(pdMS_TO_TICKS(req.periodMs));
                Led2_off();
                vTaskDelay(pdMS_TO_TICKS(req.periodMs));
            }

            Led2_off();
        }
    }
}
