#ifndef BOT_HPP
#define BOT_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <random>
#include "game.hpp"
#include "obstacle.hpp"

#define SENSOR_ANGLE 30
#define BOT_FILL_COLOR sf::Color(0xB1, 0xB1, 0xB1)
#define BOT_OUTLINE_COLOR sf::Color(0x47, 0x47, 0x47)
#define BOT_SENSOR_COLOR sf::Color(0x4B, 0x00, 0x96)
#define SHADE_COLOR sf::Color(0xFF, 0xF4, 0xC2)

static std::default_random_engine gen;
static std::normal_distribution<float> normal(0.f, 1.f);

class Bot {
	private:
		Game * game;
		float rotateSpeed;
		float moveSpeed;
		sf::CircleShape shadeBlock;
		std::vector<sf::CircleShape> shade;
		std::vector<Obstacle> obstacles;
		

	public:
		sf::CircleShape shape;
		sf::CircleShape point;
		std::vector<sf::RectangleShape> sensors;

		Bot(Game * game);
		~Bot() { }
		void update(const float dt);
		void draw(const float dt);
		void rotate(const int dir);
		void move(const int dir);
		void sense();
		bool intersect(sf::CircleShape bot, sf::RectangleShape wall);
		std::vector<sf::RectangleShape> getViewObstacles();
};

#endif