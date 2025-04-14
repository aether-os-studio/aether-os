#include "ahci.h"

void sata_submit(struct hba_device *dev, struct blkio_req *io_req)
{
    struct hba_port *port = dev->port;
    struct hba_cmdh *header;
    struct hba_cmdt *table;

    int write = !!(io_req->flags & 1);
    int slot = hba_prepare_cmd(port, &table, &header);
    hba_find_sbuf(header, table, io_req->buf, io_req->len);

    header->options |= HBA_CMDH_WRITE * write;

    uint16_t count = ICEIL(io_req->len, port->device->block_size);
    struct sata_reg_fis *fis = (struct sata_reg_fis *)table->command_fis;

    if ((port->device->flags & HBA_DEV_FEXTLBA))
    {
        // 如果该设备支持48位LBA寻址
        sata_create_fis(fis,
                        write ? ATA_WRITE_DMA_EXT : ATA_READ_DMA_EXT,
                        io_req->buf,
                        count);
    }
    else
    {
        sata_create_fis(
            fis, write ? ATA_WRITE_DMA : ATA_READ_DMA, io_req->buf, count);
    }
    /*
          确保我们使用的是LBA寻址模式
          注意：在ACS-3中（甚至在ACS-4），只有在(READ/WRITE)_DMA_EXT指令中明确注明了需要将这一位置位
        而并没有在(READ/WRITE)_DMA注明。
          但是这在ACS-2中是有的！于是这也就导致了先前的测试中，LBA=0根本无法访问，因为此时
        的访问模式是在CHS下，也就是说LBA=0 => Sector=0，是非法的。
          所以，我猜测，这要么是QEMU/VirtualBox根据ACS-2来编写的AHCI模拟，
        要么是标准出错了（毕竟是working draft）
    */
    fis->dev = (1 << 6);

    // The async way...
    struct hba_cmd_state *cmds = malloc(sizeof(struct hba_cmd_state));
    *cmds = (struct hba_cmd_state){.cmd_table = table, .state_ctx = io_req};
    ahci_post(port, cmds, slot);
}
