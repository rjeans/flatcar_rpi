// SPDX-License-Identifier: GPL-2.0
#ifndef MAILBOX_BCM2835_ACPI_H
#define MAILBOX_BCM2835_ACPI_H

#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/platform_device.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int bcm2835_register_client(struct mbox_client *client);
extern int bcm2835_unregister_client(struct mbox_client *client);
extern struct mbox_chan *bcm2835_get_mbox_chan(struct mbox_client *client);

#ifdef __cplusplus
}
#endif

#endif // MAILBOX_BCM2835_ACPI_H