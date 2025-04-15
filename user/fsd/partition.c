#include "partition.h"

partition_t partitions[MAX_PARTITIONS_NUM];
uint64_t partition_num;

uint64_t partition_read(uint64_t part_id, uint64_t offset, void *buf, uint64_t len)
{
}

uint64_t partition_write(uint64_t part_id, uint64_t offset, void *buf, uint64_t len)
{
}

void partition_init()
{
    memset(partitions, 0, sizeof(partitions));

    int fd = open("/scheme/block", 0, 0);
    if (fd < 0)
        return;

    uint64_t block_dev_num = ioctl(fd, SCHEME_IOCTL_GETSIZE, 0);
    if ((int64_t)block_dev_num < 0)
        return;

    close(fd);

    for (uint64_t i = 0; i < block_dev_num; i++)
    {
        char buf[16];
        sprintf(buf, "/scheme/block/%d", i);
        int blkdev_fd = open(buf, 0, 0);
        if (blkdev_fd < 0)
            return;

        partition_t *part = &partitions[partition_num];

        struct GPT_DPT *buffer = (struct GPT_DPT *)malloc(sizeof(struct GPT_DPT));
        lseek(fd, 512);
        read(fd, buffer, sizeof(struct GPT_DPT));

        if (memcmp(buffer->signature, GPT_HEADER_SIGNATURE, 8) || buffer->num_partition_entries == 0 || buffer->partition_entry_lba == 0)
            goto probe_mbr;

        struct GPT_DPTE *dptes = (struct GPT_DPT *)malloc(sizeof(struct GPT_DPTE) * buffer->num_partition_entries);
        lseek(fd, buffer->partition_entry_lba * 512);
        read(fd, buffer, sizeof(struct GPT_DPTE) * buffer->num_partition_entries);

        for (uint32_t j = 0; j < buffer->num_partition_entries; j++)
        {
            if (dptes[j].ending_lba < dptes[j].starting_lba)
                continue;

            part->blkdev_fd = blkdev_fd;
            part->starting_lba = dptes[j].starting_lba;
            part->ending_lba = dptes[j].ending_lba;
            part->type = GPT;
            partition_num++;
        }

        free(dptes);
        free(buffer);

        close(fd);

        continue;

    probe_mbr:

        struct MBR_DPT *boot_sector = (struct MBR_DPT *)malloc(sizeof(struct MBR_DPT));
        lseek(fd, 0);
        read(fd, boot_sector, sizeof(struct MBR_DPT));
        if (boot_sector->BS_TrailSig != 0xaa55)
        {
            part->blkdev_fd = blkdev_fd;
            part->starting_lba = 0;
            part->ending_lba = 0;
            part->type = ISO9660;
            partition_num++;
            goto ok;
        }

        for (int j = 0; j < MBR_MAX_PARTITION_NUM; j++)
        {
            if (boot_sector->DPTE[j].start_LBA == 0 || boot_sector->DPTE[j].sectors_limit == 0)
                continue;

            part->blkdev_fd = blkdev_fd;
            part->starting_lba = boot_sector->DPTE[j].start_LBA;
            part->ending_lba = boot_sector->DPTE[j].sectors_limit;
            part->type = MBR;
            partition_num++;
        }

    ok:
        free(boot_sector);
        close(fd);
    }

    printf("Found %d partitions", partition_num);
}
