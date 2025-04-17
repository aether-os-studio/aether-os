all:
	gcc $(CFLAGS) -I$(CWD)/pcid -c nvmed.c -o nvmed.o
	gcc $(CFLAGS) -I$(CWD)/pcid -c driver.c -o driver.o
	ld nvmed.o driver.o $(shell find $(CWD)/obj/ -name "*.o") -o nvmed.exec	
