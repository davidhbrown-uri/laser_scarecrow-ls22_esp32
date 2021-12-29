#include "buzzer.h"

void buzzer_init(void)
{
    ls_buzzer_queue = xQueueCreate( 32, sizeof(enum ls_buzzer_effects));

}

void buzzer_play(enum ls_buzzer_effects effect)
{
    xQueueSend(ls_buzzer_queue, (void *) &effect, 0); // don't block if queue full
}


void buzzer_handler_task(void *pvParameter)
{
    enum ls_buzzer_effects received;
    while(1)
    {
        if(xQueueReceive(ls_buzzer_queue, &received, portMAX_DELAY) != pdTRUE)
        {
            printf("No buzz requested maximum delay... getting very bored.");
        }
        else {
            switch (received) {
                case LS_BUZZER_CLICK:
                    printf("Click\n");
                    break;
                default:
                    printf("Unknown ls_buzzer_effect %d -- I'm confused", received);
            }
        }
    }
}
