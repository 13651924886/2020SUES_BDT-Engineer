#include "Revive_task.h"
/*
	救援任务，用于控制伸缩RFID卡
*/

int ReviveCard_Current = 0;

void ReviveCard_ReachOUT(void)
{
	ReviveCard_Current = rc_ctrl.rc.ch[4];
}
