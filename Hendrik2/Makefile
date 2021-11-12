CC = g++
CFLAGS = -Wall -g
SFMLFLAGS = -lsfml-graphics -lsfml-window -lsfml-system -lm
OBJECTS = main.o game.o bot.o obstacle.o node.o
HEADERS = game.hpp bot.hpp obstacle.hpp node.hpp

run: main
	./$^

main: $(OBJECTS)
	$(CC) $(CFLAGS) -o main $(OBJECTS) $(SFMLFLAGS)

main.o: main.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c main.cpp

*.o: $(HEADERS)

