VPATH = src obj
DEPS = my_mat_math_5.h my_box.h
OBJ = test.o my_mat_math_5.o
LIBS = -lX11 -lGL -lm -lrt
CFLAGS = -g

a.out: $(OBJ)
	gcc $(addprefix obj/, $(^F)) $(LIBS) -o $@

$(OBJ): %.o: %.c $(DEPS)
	gcc $(CFLAGS) -I./src -c -o obj/$(@F) src/$(<F)
