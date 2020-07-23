#include "Revive_task.h"


int ReviveCard_Current = 0;

void ReviveCard_ReachOUT(void)
{
	ReviveCard_Current = rc_ctrl.rc.ch[4];
}
