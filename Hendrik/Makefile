CC = g++
CFLAGS = -Wall -g
SFMLFLAGS = -lsfml-graphics -lsfml-window -lsfml-system -lm
OBJECTS = main.o game.o bot.o obstacle.o
HEADERS = game.hpp bot.hpp obstacle.hpp

main: $(OBJECTS)
	$(CC) $(CFLAGS) -o main $(OBJECTS) $(SFMLFLAGS)

main.o: main.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c main.cpp

*.o: $(HEADERS)