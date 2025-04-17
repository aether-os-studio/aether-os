all:
	gcc $(CFLAGS) -I$(CWD)/pcid -c ps2d.c -o ps2d.o
	ld ps2d.o $(shell find $(CWD)/obj/ -name "*.o") -o ps2d.exec	
