all:
	gcc $(CFLAGS) -c fsd.c -o fsd.o
	gcc $(CFLAGS) -c partition.c -o partition.o
	gcc $(CFLAGS) -c vfs/vfs.c -o vfs/vfs.o
	gcc $(CFLAGS) -c vfs/dev.c -o vfs/dev.o
	gcc $(CFLAGS) -c iso9660/iso9660.c -o iso9660/iso9660.o
	gcc $(CFLAGS) -c fatfs/diskio.c -o fatfs/diskio.o
	gcc $(CFLAGS) -c fatfs/ffsystem.c -o fatfs/ffsystem.o
	gcc $(CFLAGS) -c fatfs/ffunicode.c -o fatfs/ffunicode.o
	gcc $(CFLAGS) -c fatfs/ff.c -o fatfs/ff.o
	gcc $(CFLAGS) -c fatfs/fatfs.c -o fatfs/fatfs.o
	ld fsd.o partition.o vfs/vfs.o vfs/dev.o iso9660/iso9660.o fatfs/diskio.o fatfs/ffsystem.o fatfs/ffunicode.o fatfs/ff.o fatfs/fatfs.o $(shell find $(CWD)/obj/ -name "*.o") -o fsd.exec	
