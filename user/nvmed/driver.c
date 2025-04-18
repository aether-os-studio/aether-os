#include <stdio.h>
#include <stdlib.h>
#include "nvme.h"

#define NVME_CSTS_FATAL (1U << 1)
#define NVME_CSTS_RDY (1U << 0)

#define NVME_SQE_OPC_ADMIN_CREATE_IO_SQ 1U
#define NVME_SQE_OPC_ADMIN_CREATE_IO_CQ 5U
#define NVME_SQE_OPC_ADMIN_IDENTIFY 6U

#define NVME_SQE_OPC_IO_WRITE 1U
#define NVME_SQE_OPC_IO_READ 2U

#define NVME_ADMIN_IDENTIFY_CNS_ID_NS 0x00U
#define NVME_ADMIN_IDENTIFY_CNS_ID_CTRL 0x01U
#define NVME_ADMIN_IDENTIFY_CNS_ACT_NSL 0x02U

void *NVME_DMA_MEMORY = 0;

static inline char *LeadingWhitespace(char *beg, char *end)
{
    while (end > beg && *--end <= 0x20)
    {
        *end = 0;
    }
    while (beg < end && *beg <= 0x20)
    {
        beg++;
    }
    return beg;
}

void NVMEConfigureQ(NVME_CONTROLLER *ctrl, NVME_QUEUE_COMMON *q, uint32_t idx, uint32_t len)
{
    memset(q, 0, sizeof(NVME_QUEUE_COMMON));
    q->DBL = (uint32_t *)(((uint8_t *)ctrl->CAP) + 0x1000 + idx * ctrl->DST);
    q->MSK = len - 1;
}
int NVMEConfigureCQ(NVME_CONTROLLER *ctrl, NVME_COMPLETION_QUEUE *cq, uint32_t idx, uint32_t len)
{
    NVMEConfigureQ(ctrl, &cq->COM, idx, len);
    cq->CQE = 0;
    uint64_t phyAddr = 0;
    uint64_t pagCont = 1;
    phyAddr = (uint64_t)alloc_dma(1);
    cq->CQE = (NVME_COMPLETION_QUEUE_ENTRY *)physmap(phyAddr, 0x1000, PROT_READ | PROT_WRITE);
    memset(cq->CQE, 0, 0x1000);
    cq->COM.HAD = 0;
    cq->COM.TAL = 0;
    cq->COM.PHA = 1;
    return 0;
}
int NVMEConfigureSQ(NVME_CONTROLLER *ctrl, NVME_SUBMISSION_QUEUE *sq, uint32_t idx, uint32_t len)
{
    NVMEConfigureQ(ctrl, &sq->COM, idx, len);
    sq->SQE = 0;
    uint64_t phyAddr = 0;
    uint64_t pagCont = 1;
    phyAddr = (uint64_t)alloc_dma(1);
    sq->SQE = (NVME_SUBMISSION_QUEUE_ENTRY *)physmap(phyAddr, 0x1000, PROT_READ | PROT_WRITE);
    memset(sq->SQE, 0, 0x1000);
    sq->COM.HAD = 0;
    sq->COM.TAL = 0;
    sq->COM.PHA = 0;
    return 0;
}
int NVMEWaitingRDY(NVME_CONTROLLER *ctrl, uint32_t rdy)
{
    uint32_t csts;
    while (rdy != ((csts = ctrl->CAP->CST) & NVME_CSTS_RDY))
    {
        __asm__ __volatile__("pause");
    }
    return 0;
}
NVME_COMPLETION_QUEUE_ENTRY NVMEWaitingCMD(NVME_SUBMISSION_QUEUE *sq, NVME_SUBMISSION_QUEUE_ENTRY *e)
{
    NVME_COMPLETION_QUEUE_ENTRY errcqe;
    memset(&errcqe, 0xFF, sizeof(NVME_COMPLETION_QUEUE_ENTRY));

    if (((sq->COM.HAD + 1) % (sq->COM.MSK + 1ULL)) == sq->COM.TAL)
    {
        printf("SUBMISSION QUEUE IS FULL\n");
        return errcqe;
    }

    // Commit
    NVME_SUBMISSION_QUEUE_ENTRY *sqe = sq->SQE + sq->COM.TAL;
    memcpy(sqe, e, sizeof(NVME_SUBMISSION_QUEUE_ENTRY));
    sqe->CDW0 |= sq->COM.TAL << 16;

    // Doorbell
    sq->COM.TAL++;
    sq->COM.TAL %= (sq->COM.MSK + 1ULL);
    sq->COM.DBL[0] = sq->COM.TAL;

    // Check completion
    NVME_COMPLETION_QUEUE *cq = sq->ICQ;
    while ((cq->CQE[cq->COM.HAD].STS & 1) != cq->COM.PHA)
    {
        __asm__ __volatile__("pause");
    }

    // Consume CQE
    NVME_COMPLETION_QUEUE_ENTRY *cqe = cq->CQE + cq->COM.HAD;
    uint16_t cqNextHAD = (cq->COM.HAD + 1) % (cq->COM.MSK + 1ULL);
    if (cqNextHAD < cq->COM.HAD)
    {
        cq->COM.PHA ^= 1;
    }
    cq->COM.HAD = cqNextHAD;

    if (cqe->QHD != sq->COM.HAD)
    {
        sq->COM.HAD = cqe->QHD;
    }
    // Doorbell
    cq->COM.DBL[0] = cq->COM.HAD;
    return *cqe;
}

void nvme_rwfail(uint16_t status)
{
    printf("Status: %#010lx, status code: %#010lx, status code type: %#010lx\n", status, status & 0xFF, (status >> 8) & 0x7);
}

uint32_t NVMETransfer(NVME_NAMESPACE *ns, void *buf, uint64_t lba, uint32_t count, uint32_t write)
{
    if (!count)
        return 0;

    uint64_t bufAddr = (uint64_t)buf;
    uint32_t maxCount = (0x1000 / ns->BSZ) - ((bufAddr & 0xFFF) / ns->BSZ);
    if (count > maxCount)
        count = maxCount;
    if (count > ns->MXRS)
        count = ns->MXRS;
    uint64_t size = count * ns->BSZ;

    NVME_SUBMISSION_QUEUE_ENTRY sqe;
    memset(&sqe, 0, sizeof(NVME_SUBMISSION_QUEUE_ENTRY));
    sqe.CDW0 = write ? NVME_SQE_OPC_IO_WRITE : NVME_SQE_OPC_IO_READ;
    sqe.META = 0;
    sqe.DATA[0] = virttophys(bufAddr);
    sqe.DATA[1] = 0;
    sqe.NSID = ns->NSID;
    sqe.CDWA = lba;
    sqe.CDWB = lba >> 32;
    sqe.CDWC = (1UL << 31) | ((count - 1) & 0xFFFF);
    NVME_COMPLETION_QUEUE_ENTRY cqe = NVMEWaitingCMD(&ns->CTRL->ISQ, &sqe);
    if ((cqe.STS >> 1) & 0xFF)
    {
        if (write)
        {
            printf("NVME CANNOT WRITE\n");
        }
        else
            printf("NVME CANNOT READ\n");

        nvme_rwfail(cqe.STS);
        return -1;
    }
    return count;
}

void failed_nvme(NVME_CONTROLLER *ctrl)
{
    if (ctrl->ICQ.CQE)
    {
        free_dma(virttophys((uint64_t)ctrl->ICQ.CQE), 1);
    }
    if (ctrl->ISQ.SQE)
    {
        free_dma(virttophys((uint64_t)ctrl->ISQ.SQE), 1);
    }
    if (ctrl->ACQ.CQE)
    {
        free_dma(virttophys((uint64_t)ctrl->ACQ.CQE), 1);
    }
    if (ctrl->ASQ.SQE)
    {
        free_dma(virttophys((uint64_t)ctrl->ASQ.SQE), 1);
    }
}

void failed_namespace(NVME_IDENTIFY_NAMESPACE *identifyNS)
{
    free_dma(virttophys((uint64_t)identifyNS), 1);
}

int NvmeRead(void *dev, void *buf, size_t count, int idx, int flags)
{
    return NVMETransfer((NVME_NAMESPACE *)dev, buf, idx, count, false);
}

int NvmeWrite(void *dev, void *buf, size_t count, int idx, int flags)
{
    return NVMETransfer((NVME_NAMESPACE *)dev, buf, idx, count, true);
}

void nvme_driver_init(uint64_t bar0, uint64_t bar_size)
{
    NVME_CAPABILITY *cap = (NVME_CAPABILITY *)physmap(bar0, bar_size, PROT_READ | PROT_WRITE);

    if (!((cap->CAP >> 37) & 1))
    {
        printf("NVME CONTROLLER DOES NOT SUPPORT NVME COMMAND SET\n");
        return;
    }

    NVME_CONTROLLER *ctrl = (NVME_CONTROLLER *)malloc(sizeof(NVME_CONTROLLER));
    memset(ctrl, 0, sizeof(NVME_CONTROLLER));
    ctrl->CAP = cap;
    ctrl->WTO = 500 * ((cap->CAP >> 24) & 0xFF);

    // RST controller
    ctrl->CAP->CC = 0;
    if (NVMEWaitingRDY(ctrl, 0))
    {
        printf("NVME FATAL ERROR DURING CONTROLLER SHUTDOWN\n");
        failed_nvme(ctrl);
        return;
    }
    ctrl->DST = 4ULL << ((cap->CAP >> 32) & 0xF);

    int rc = NVMEConfigureCQ(ctrl, &ctrl->ACQ, 1, 0x1000 / sizeof(NVME_COMPLETION_QUEUE_ENTRY));
    if (rc)
    {
        failed_nvme(ctrl);
        return;
    }

    rc = NVMEConfigureSQ(ctrl, &ctrl->ASQ, 0, 0x1000 / sizeof(NVME_SUBMISSION_QUEUE_ENTRY));
    if (rc)
    {
        failed_nvme(ctrl);
        return;
    }
    ctrl->ASQ.ICQ = &ctrl->ACQ;

    ctrl->CAP->AQA = (ctrl->ACQ.COM.MSK << 16) | ctrl->ASQ.COM.MSK;
    ctrl->CAP->ASQ = virttophys((uint64_t)ctrl->ASQ.SQE);
    ctrl->CAP->ACQ = virttophys((uint64_t)ctrl->ACQ.CQE);

    ctrl->CAP->CC = 1 | (4 << 20) | (6 << 16);
    if (NVMEWaitingRDY(ctrl, 1))
    {
        printf("NVME FATAL ERROR DURING CONTROLLER ENABLING\n");
        failed_nvme(ctrl);
        return;
    }

    /* The admin queue is set up and the controller is ready. Let's figure out
       what namespaces we have. */
    // Identify Controller
    NVME_IDENTIFY_CONTROLLER *identify = 0;
    uint64_t pagCont = 1;
    identify = (NVME_IDENTIFY_CONTROLLER *)alloc_dma(1);
    identify = (NVME_IDENTIFY_CONTROLLER *)physmap((uint64_t)identify, 0x1000, PROT_READ | PROT_WRITE);
    memset(identify, 0, 0x1000);

    NVME_SUBMISSION_QUEUE_ENTRY sqe;
    memset(&sqe, 0, sizeof(NVME_SUBMISSION_QUEUE_ENTRY));
    sqe.CDW0 = NVME_SQE_OPC_ADMIN_IDENTIFY;
    sqe.META = 0;
    sqe.DATA[0] = virttophys((uint64_t)identify);
    sqe.DATA[1] = 0;
    sqe.NSID = 0;
    sqe.CDWA = NVME_ADMIN_IDENTIFY_CNS_ID_CTRL;
    NVME_COMPLETION_QUEUE_ENTRY cqe = NVMEWaitingCMD(&ctrl->ASQ, &sqe);
    if ((cqe.STS >> 1) & 0xFF)
    {
        printf("CANNOT IDENTIFY NVME CONTROLLER\n");
        failed_nvme(ctrl);
        return;
    }

    char buf[41];
    memcpy(buf, identify->SERN, sizeof(identify->SERN));
    buf[sizeof(identify->SERN)] = 0;
    char *serialN = LeadingWhitespace(buf, buf + sizeof(identify->SERN));
    memcpy(ctrl->SER, serialN, strlen(serialN));
    // printf(serialN);
    // LINEFEED();
    memcpy(buf, identify->MODN, sizeof(identify->MODN));
    buf[sizeof(identify->MODN)] = 0;
    serialN = LeadingWhitespace(buf, buf + sizeof(identify->MODN));
    memcpy(ctrl->MOD, serialN, strlen(serialN));
    // printf(serialN);
    // LINEFEED();

    ctrl->NSC = identify->NNAM;
    uint8_t mdts = identify->MDTS;
    free_dma(virttophys((uint64_t)identify), 1);

    if (ctrl->NSC == 0)
    {
        printf("NO NAMESPACE\n");
        failed_nvme(ctrl);
        return;
    }

    // Create I/O Queue
    // Create I/O CQ
    {
        uint32_t qidx = 3;
        uint32_t entryCount = 1 + (ctrl->CAP->CAP & 0xFFFF);
        if (entryCount > 0x1000 / sizeof(NVME_COMPLETION_QUEUE_ENTRY))
            entryCount = 0x1000 / sizeof(NVME_COMPLETION_QUEUE_ENTRY);
        if (NVMEConfigureCQ(ctrl, &ctrl->ICQ, qidx, entryCount))
        {
            printf("CANNOT INIT I/O CQ\n");
            failed_nvme(ctrl);
            return;
        }
        NVME_SUBMISSION_QUEUE_ENTRY ccq;
        memset(&ccq, 0, sizeof(NVME_SUBMISSION_QUEUE_ENTRY));
        ccq.CDW0 = NVME_SQE_OPC_ADMIN_CREATE_IO_CQ;
        ccq.META = 0;
        ccq.DATA[0] = virttophys((uint64_t)ctrl->ICQ.CQE);
        ccq.DATA[1] = 0;
        ccq.CDWA = (ctrl->ICQ.COM.MSK << 16) | (qidx >> 1);
        ccq.CDWB = 1;

        cqe = NVMEWaitingCMD(&ctrl->ASQ, &ccq);
        if ((cqe.STS >> 1) & 0xFF)
        {
            printf("CANNOT CREATE I/O CQ\n");
            failed_nvme(ctrl);
            return;
        }
    }

    // Create I/O SQ
    {
        uint32_t qidx = 2;
        uint32_t entryCount = 1 + (ctrl->CAP->CAP & 0xFFFF);
        if (entryCount > 0x1000 / sizeof(NVME_SUBMISSION_QUEUE_ENTRY))
            entryCount = 0x1000 / sizeof(NVME_SUBMISSION_QUEUE_ENTRY);
        if (NVMEConfigureSQ(ctrl, &ctrl->ISQ, qidx, entryCount))
        {
            printf("CANNOT INIT I/O SQ\n");
            failed_nvme(ctrl);
            return;
        }
        NVME_SUBMISSION_QUEUE_ENTRY csq;
        memset(&csq, 0, sizeof(NVME_SUBMISSION_QUEUE_ENTRY));
        csq.CDW0 = NVME_SQE_OPC_ADMIN_CREATE_IO_SQ;
        csq.META = 0;
        csq.DATA[0] = virttophys((uint64_t)ctrl->ISQ.SQE);
        csq.DATA[1] = 0;
        csq.CDWA = (ctrl->ICQ.COM.MSK << 16) | (qidx >> 1);
        csq.CDWB = ((qidx >> 1) << 16) | 1;

        cqe = NVMEWaitingCMD(&ctrl->ASQ, &csq);
        if ((cqe.STS >> 1) & 0xFF)
        {
            printf("CANNOT CREATE I/O SQ\n");
            failed_nvme(ctrl);
            return;
        }
        ctrl->ISQ.ICQ = &ctrl->ICQ;
    }

    /* Populate namespace IDs */
    for (uint32_t nsidx = 0; nsidx < ctrl->NSC; nsidx++)
    {
        // Probe Namespace
        uint32_t nsid = nsidx + 1;

        NVME_IDENTIFY_NAMESPACE *identifyNS = 0;
        pagCont = 1;
        identifyNS = (NVME_IDENTIFY_NAMESPACE *)alloc_dma(1);
        identifyNS = (NVME_IDENTIFY_NAMESPACE *)physmap((uint64_t)identifyNS, 0x1000, PROT_READ | PROT_WRITE);
        memset(identifyNS, 0, 0x1000);

        memset(&sqe, 0, sizeof(NVME_SUBMISSION_QUEUE_ENTRY));
        sqe.CDW0 = NVME_SQE_OPC_ADMIN_IDENTIFY;
        sqe.META = 0;
        sqe.DATA[0] = virttophys((uint64_t)identifyNS);
        sqe.DATA[1] = 0;
        sqe.NSID = nsid;
        sqe.CDWA = NVME_ADMIN_IDENTIFY_CNS_ID_NS;
        cqe = NVMEWaitingCMD(&ctrl->ASQ, &sqe);
        if ((cqe.STS >> 1) & 0xFF)
        {
            printf("CANNOT IDENTIFY NAMESPACE\n");
            failed_namespace(identifyNS);
            return;
        }

        uint8_t currentLBAFormat = identifyNS->FLBA & 0xF;
        if (currentLBAFormat > identifyNS->NLBA)
        {
            printf("Current LBA Format error\n");
            failed_namespace(identifyNS);
            return;
        }

        if (!identifyNS->SIZE)
        {
            failed_namespace(identifyNS);
            return;
        }

        if (!NVME_DMA_MEMORY)
        {
            pagCont = 1;
            NVME_DMA_MEMORY = (void *)alloc_dma(pagCont);
        }

        NVME_NAMESPACE *ns = (NVME_NAMESPACE *)malloc(sizeof(NVME_NAMESPACE));
        memset(ns, 0, sizeof(NVME_NAMESPACE));
        ns->CTRL = ctrl;
        ns->NSID = nsid;
        ns->NLBA = identifyNS->SIZE;

        NVME_LOGICAL_BLOCK_ADDRESS *fmt = identifyNS->LBAF + currentLBAFormat;

        ns->BSZ = 1ULL << fmt->DS;
        ns->META = fmt->MS;
        if (ns->BSZ > 0x1000)
        {
            printf("BLOCK SIZE > 0x1000 !!!\n");
            free(ns);
            failed_namespace(identifyNS);
            return;
        }

        if (mdts)
        {
            ns->MXRS = ((1ULL << mdts) * 0x1000) / ns->BSZ;
        }
        else
        {
            ns->MXRS = -1;
        }

        namespaces[nvme_device_num] = ns;

        char buf[30];
        sprintf(buf, "/scheme/block//scheme/nvmed/%d", nvme_device_num);

        int fd = open(buf, 0, 0);

        ioctl(fd, SCHEME_IOCTL_REGIST_BLKDEV, 512);

        close(fd);
    }

    return;

FAILED_NVME:
    free(ctrl);
}
