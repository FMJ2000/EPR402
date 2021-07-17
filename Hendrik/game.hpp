#ifndef GAME_HPP
#define GAME_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <stack>

#define GAME_NAME "Hendrik"
#define BACKGROUND_COLOR sf::Color(0xA3, 0x9C, 0x7C)
#define WALL_COLOR sf::Color(0x7D, 0x3E, 00)
#define WALL_SENSE_COLOR sf::Color(0x96, 0x4B, 0x00)

class Bot;

class Game {
	public:
		std::vector<sf::RectangleShape> walls;
		Bot * bot;
		sf::RenderWindow window;

		Game();
		~Game();
		void gameloop();
		void handleInput();
		void update(const float dt);
		void draw(const float dt);
		void createOuterWalls();
		std::pair<std::vector<float>, std::vector<float>> sense();
		std::vector<sf::RectangleShape> getViewObstacles();
};

#endif