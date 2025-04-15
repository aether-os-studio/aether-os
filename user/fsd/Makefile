all:
	gcc $(CFLAGS) -c fsd.c -o fsd.o
	gcc $(CFLAGS) -c partition.c -o partition.o
	gcc $(CFLAGS) -c vfs/vfs.c -o vfs/vfs.o
	gcc $(CFLAGS) -c vfs/dev.c -o vfs/dev.o
	gcc $(CFLAGS) -c iso9660/iso9660.c -o iso9660/iso9660.o
	ld fsd.o partition.o vfs/vfs.o vfs/dev.o iso9660/iso9660.o $(shell find $(CWD)/obj/ -name "*.o") -o fsd.exec	
