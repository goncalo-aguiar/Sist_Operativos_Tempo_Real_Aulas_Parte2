/* auto-generated by gen_syscalls.py, don't edit */

#include <syscalls/can.h>

extern const struct can_timing * z_vrfy_can_get_timing_max_data(const struct device * dev);
uintptr_t z_mrsh_can_get_timing_max_data(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	(void) arg1;	/* unused */
	(void) arg2;	/* unused */
	(void) arg3;	/* unused */
	(void) arg4;	/* unused */
	(void) arg5;	/* unused */
	union { uintptr_t x; const struct device * val; } parm0;
	parm0.x = arg0;
	const struct can_timing * ret = z_vrfy_can_get_timing_max_data(parm0.val);
	_current->syscall_frame = NULL;
	return (uintptr_t) ret;
}

