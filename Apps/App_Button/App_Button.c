#include "App_Button.h"

#include "App_IMU.h"
#include "App_Debug.h"
#include "App_Led2.h"

#include "IfxPort.h"
#include "FreeRTOS.h"
#include "task.h"

static boolean Button1_isPressed(void)
{
    return (IfxPort_getPinState(BUTTON1_PORT, BUTTON1_PIN) == 0);
}

void AppButton_init(void)
{
    IfxPort_setPinMode(BUTTON1_PORT, BUTTON1_PIN, IfxPort_Mode_inputPullUp);
}

void task_app_button(void *arg)
{
    boolean prevPressed;
    boolean nowPressed;

    (void)arg;

    AppButton_init();
    prevPressed = Button1_isPressed();

    for (;;)
    {
        nowPressed = Button1_isPressed();

        if ((prevPressed == FALSE) && (nowPressed == TRUE))
        {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));

            if (Button1_isPressed() == TRUE)
            {
                DebugLog_printf("[BTN] BUTTON1 pressed. IMU yaw recalibration requested.\r\n");
                AppLed2_blink(1u, 100u);
                ImuYawApp_requestCalibration();

                while (Button1_isPressed() == TRUE)
                {
                    vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
                }
            }
        }

        prevPressed = nowPressed;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}
