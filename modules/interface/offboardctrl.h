#ifndef OFFBOARDCTRL_H_
#define OFFBOARDCTRL_H_
#include <stdint.h>
#include <stdbool.h>

#define OFFBOARDCTRL_WDT_TIMEOUT_SHUTDOWN M2T(500)

void offboardCtrlInit(void);
bool offboardCtrlTest(void);
void offboardCtrlTask(void*);

#endif /* OFFBOARDCTRL_H_ */
