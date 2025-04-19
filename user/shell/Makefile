all:
	$(MAKE) -C pl_readline lib

	gcc $(CFLAGS) -Ipl_readline/include/ -c shell.c -o shell.o
	ld shell.o pl_readline/libplreadln.a $(shell find $(CWD)/obj/ -name "*.o") -o shell.exec	
