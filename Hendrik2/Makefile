CC=gcc
CFLAGS=-I.
DEPS=bot.h map.h aux.h
OBJ=bot.o map.o aux.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

bot: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)