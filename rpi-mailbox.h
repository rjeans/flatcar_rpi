// SPDX-License-Identifier: GPL-2.0
#ifndef RPI_MAILBOX_H
#define RPI_MAILBOX_H

#include <linux/mailbox_controller.h>


#ifdef __cplusplus
extern "C" {
#endif

extern struct mbox_chan *rpi_mbox_request_channel(struct mbox_client *);
extern int rpi_mbox_free_channel(struct mbox_chan *);
extern struct mbox_chan *rpi_mbox_request_firmware_channel(struct mbox_client *);





#ifdef __cplusplus
}
#endif

#endif // RPI_MAILBOX_H