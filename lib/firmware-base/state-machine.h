#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum sys_states {
	SYS_STATE_UNDEF = 0,
	SYS_STATE_INIT,
	SYS_STATE_IDLE,
	SYS_STATE_DBW_ACTIVE,
	SYS_STATE_LOST_CAN,
	SYS_STATE_BAD,
	SYS_STATE_ESTOP,
};

enum sys_states base_get_state(void);
void		      base_request_state(enum sys_states state);

#endif	// STATE_MACHINE_H
