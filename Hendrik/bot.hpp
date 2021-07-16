#ifndef BOT_HPP
#define BOT_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <random>
#include "game.hpp"
#include "obstacle.hpp"

#define SENSOR_INT 0.3f
#define SENSOR_ANGLE 30
#define SENSOR_OFFSET 10.f
#define SENSOR_SAMPLES 5
#define SENSOR_MAX 500.f
#define BOT_FILL_COLOR sf::Color(0xB1, 0xB1, 0xB1)
#define BOT_OUTLINE_COLOR sf::Color(0x47, 0x47, 0x47)
#define BOT_SENSOR_COLOR sf::Color(0x32, 0xCD, 0x32)
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
		float currentTime;

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
		std::vector<sf::Vector2f> ellipticLocalization(std::vector<float> r1, std::vector<float> r2);
};

#endif