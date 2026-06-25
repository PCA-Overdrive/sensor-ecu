#ifndef APP_LED2_H_
#define APP_LED2_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TASK_PRIO_LED2      (1u)
#define TASK_STACK_LED2     (configMINIMAL_STACK_SIZE)

#define LED2_QUEUE_LENGTH       8u
#define LED2_DEFAULT_PERIOD_MS  100u

#define LED2_PORT               (&MODULE_P00)
#define LED2_PIN                (6u)

typedef struct
{
    uint32_t count;
    uint32_t periodMs;
} Led2BlinkRequest_t;

void AppLed2_init(void);
void AppLed2_blink(uint32_t count, uint32_t periodMs);
void task_app_led2(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* APP_LED2_H_ */
