all:
	gcc $(CFLAGS) -I$(CWD)/pcid -c ahcid.c -o ahcid.o
	gcc $(CFLAGS) -I$(CWD)/pcid -c utils.c -o utils.o
	gcc $(CFLAGS) -I$(CWD)/pcid -c ata.c -o ata.o
	gcc $(CFLAGS) -I$(CWD)/pcid -c atapi.c -o atapi.o
	gcc $(CFLAGS) -I$(CWD)/pcid -c scheme.c -o scheme.o
	ld ahcid.o utils.o ata.o atapi.o scheme.o $(shell find $(CWD)/obj/ -name "*.o") -o ahcid.exec	
