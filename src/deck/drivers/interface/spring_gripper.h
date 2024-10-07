#ifndef _SPRING_GRIPPER_H
#define _PRING_GRIPPER_H

#include "deck_core.h"

enum gripperStates {
    IDLE = 0,
    RDY2LAND = 1,
    ACTIVATING_GRIPPER = 2,
    LANDED = 3,
    ACTIVATING_GRIPPER_LANDED = 4,
    RELEASING_GRIPPER = 5,
};

void springGripperInit(DeckInfo* info);

bool springGripperTest(void);
void springGripperTask(void* arg);

#endif /* _SPRING_GRIPPER_H_ */