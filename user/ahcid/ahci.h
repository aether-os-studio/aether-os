#pragma once

#include "hba.h"
#include "sata.h"
#include "scsi.h"

struct ahci_driver
{
    struct ahci_hba hba;
};

int ahci_try_send(struct hba_port *port, int slot);
void ahci_post(struct hba_port *port, struct hba_cmd_state *state, int slot);

int hba_prepare_cmd(struct hba_port *port,
                    struct hba_cmdt **cmdt,
                    struct hba_cmdh **cmdh);
int hba_find_sbuf(struct hba_cmdh *cmdh, struct hba_cmdt *cmdt, void *buf, uint32_t len);
