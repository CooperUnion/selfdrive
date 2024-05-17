#include "modules.inc"
#include <ember_app_desc.h>
#include <ember_bltools.h>
#include <ember_tasking.h>

void app_main()
{
	/* set boot partition back to bootloader */
	ember_bltools_set_boot_partition_to_factory();

	/* begin running tasks */
	ember_tasking_begin();

	fprintf(stderr,
		"Hello from %.16s\n",
		ember_app_description.node_identity);
}
