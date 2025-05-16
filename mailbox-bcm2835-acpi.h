// SPDX-License-Identifier: GPL-2.0
#ifndef MAILBOX_BCM2835_ACPI_H
#define MAILBOX_BCM2835_ACPI_H

#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/platform_device.h>

#ifdef __cplusplus
extern "C" {
#endif

extern struct mbox_chan *global_rpi_mbox_chan;

struct rpi_firmware_power_msg {
	u32 size;           // Total size of the buffer in bytes
	u32 code;           // Request code (0 = process request)

	struct {
		u32 tag;        // Tag ID (0x00028001 = set power state)
		u32 buf_size;   // Size of the value buffer (8)
		u32 val_len;    // Length of the actual value data (8)
		u32 domain;     // Power domain ID (e.g., 0x00000000 = SD card)
		u32 state;      // Bit 0: 1 = ON, Bit 1: 1 = WAIT
	} __packed body;

	u32 end_tag;        // 0
} __packed;


#ifdef __cplusplus
}
#endif

#endif // MAILBOX_BCM2835_ACPI_H