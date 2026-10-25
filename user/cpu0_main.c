#include "app.h"
int core0_main(void)
{
    app_init();
    while(1) app_poll();
}
