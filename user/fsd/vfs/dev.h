#pragma once

#include "../partition.h"
#include "vfs.h"

extern vfs_node_t dev_nodes[MAX_PARTITIONS_NUM];

void dev_init();
