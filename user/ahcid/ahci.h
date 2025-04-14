#pragma once

#include <io.h>

#include "hba.h"
#include "sata.h"
#include "scsi.h"

#define is_digit(c) ((c) >= '0' && (c) <= '9') // 用来判断是否是数字的宏

extern user_scheme_t ahcid_scheme;

struct ahci_driver
{
    struct ahci_hba hba;
};

extern void *op_buffer;

extern struct ahci_driver *drv;

void sata_read_error(struct hba_port *port);

int ahci_try_send(struct hba_port *port, int slot);
void ahci_post(struct hba_port *port, struct hba_cmd_state *state, int slot);

int hba_prepare_cmd(struct hba_port *port,
                    struct hba_cmdt **cmdt,
                    struct hba_cmdh **cmdh);
int hba_bind_sbuf(struct hba_cmdh *cmdh, struct hba_cmdt *cmdt, void *buf, uint32_t len);

uint64_t main_loop();
