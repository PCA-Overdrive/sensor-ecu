#ifndef APP_BUTTON_H_
#define APP_BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TASK_PRIO_BUTTON    (2u)
#define TASK_STACK_BUTTON   (configMINIMAL_STACK_SIZE)

#define BUTTON_POLL_MS      10u
#define BUTTON_DEBOUNCE_MS  30u

#define BUTTON1_PORT        (&MODULE_P00)
#define BUTTON1_PIN         (7u)

void AppButton_init(void);
void task_app_button(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUTTON_H_ */
