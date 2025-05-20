// SPDX-License-Identifier: GPL-2.0
#ifndef MAILBOX_BCM2835_ACPI_H
#define MAILBOX_BCM2835_ACPI_H

#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/platform_device.h>

#ifdef __cplusplus
extern "C" {
#endif

extern struct mbox_chan *bcm2835_mbox_request_channel(struct mbox_client *);
extern int bcm2835_mbox_free_channel(struct mbox_chan *);
extern bcm2835_mbox_request_firmware_channel(struct mbox_client *);

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

struct rpi_firmware_clock_msg {
	u32 size;      // Total size of the message in bytes
	u32 code;      // Request code: 0 = request

	struct {
		u32 tag;        // 0x00038001 = set clock state
		u32 buf_size;   // Size of the value buffer: 8
		u32 val_len;    // Length of value data: 8
		u32 clock_id;   // ID of the clock (e.g. 4 = PWM)
		u32 state;      // Bit 0: 1=enable, Bit 1: 1=wait
	} __packed body;

	u32 end_tag;   // Always 0
} __packed;



#ifdef __cplusplus
}
#endif

#endif // MAILBOX_BCM2835_ACPI_H