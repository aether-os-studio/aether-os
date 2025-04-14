all:
	gcc $(CFLAGS) -c blkd.c -o blkd.o
	ld blkd.o $(shell find $(CWD)/obj/ -name "*.o") -o blkd.exec	
