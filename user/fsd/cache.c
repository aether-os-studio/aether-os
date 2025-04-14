#include "cache.h"

block_device_t block_devices[MAX_BLOCK_DEVICES_NUM];

uint64_t blkdev_readfrom(block_device_t *self, uint64_t lba, void *buf, uint64_t size)
{
    lseek(self->fd, lba * self->block_size);
    read(self->fd, buf, size);
}

uint64_t blkdev_writeto(block_device_t *self, uint64_t lba, void *buf, uint64_t size)
{
    lseek(self->fd, lba * self->block_size);
    write(self->fd, buf, size);
}

void cache_init()
{
    memset(block_devices, 0, sizeof(block_devices));

    int fd = open("/scheme/block", 0, 0);
    if (fd < 0)
        return;

    uint64_t dev_count = ioctl(fd, SCHEME_IOCTL_GETSIZE, 0);
    if ((int64_t)dev_count < 0)
    {
        printf("Cannot get block size");
        close(fd);
        return;
    }

    close(fd);

    for (uint64_t i = 0; i < dev_count; i++)
    {
        char buf[16];
        sprintf(buf, "/scheme/block/%d", i);
        fd = open(buf, 0, 0);

        if (fd < 0)
            continue;

        uint64_t block_size = ioctl(fd, SCHEME_IOCTL_GETBLKSIZE, 0);
        if ((int64_t)block_size < 0)
        {
            printf("Cannot get block size");
            close(fd);
            continue;
        }

        block_devices[i].fd = fd;
        block_devices[i].block_size = block_size;
        block_devices[i].read_from = blkdev_readfrom;
        block_devices[i].write_to = blkdev_writeto;
    }
}

uint64_t block_read(uint64_t dev, uint64_t offset, void *buffer, uint64_t size)
{
    if (block_devices[dev].fd == 0)
    {
        return 0;
    }

    block_device_t *blkdev = &block_devices[dev];

    uint64_t start = offset;
    uint64_t end = offset + size;

    uint64_t start_sector_read_start = start % blkdev->block_size;

    uint64_t start_sector_id = start / blkdev->block_size;
    uint64_t end_sector_id = (end - 1) / blkdev->block_size;

    uint64_t buffer_size = (end_sector_id - start_sector_id + 1) * blkdev->block_size;

    uint8_t *tmp = malloc(buffer_size);

    blkdev->read_from(blkdev, start_sector_id, tmp, buffer_size);

    memcpy(buffer, tmp + start_sector_read_start, size);

    free(tmp);

    return size;
}

uint64_t block_write(uint64_t dev, uint64_t offset, void *buffer, uint64_t size)
{
    if (block_devices[dev].fd == 0)
    {
        return 0;
    }

    block_device_t *blkdev = &block_devices[dev];

    uint64_t start = offset;
    uint64_t end = offset + size;

    uint64_t start_sector_read_start = start % blkdev->block_size;

    uint64_t start_sector_id = start / blkdev->block_size;
    uint64_t end_sector_id = (end - 1) / blkdev->block_size;

    uint64_t buffer_size = (end_sector_id - start_sector_id + 1) * blkdev->block_size;

    uint8_t *tmp = malloc(buffer_size);

    blkdev->read_from(blkdev, start_sector_id, tmp, buffer_size);

    memcpy(tmp + start_sector_read_start, buffer, size);

    blkdev->write_to(blkdev, start_sector_id, tmp, buffer_size);

    free(tmp);

    return size;
}
