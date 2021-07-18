#ifndef BOT_HPP
#define BOT_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <random>
#include "game.hpp"
#include "obstacle.hpp"
#include "node.hpp"

#define BOT_RADIUS 30.f
#define SENSE_INTERVAL 0.3f
#define SENSOR_ANGLE 15
#define SENSOR_OFFSET 10.f
#define SENSOR_SAMPLES 3
#define SENSOR_MAX 500.f
#define OBSTACLE_MAX_DIST 3.f
#define OBSTACLE_PERIPH 200.f
#define BOT_FILL_COLOR sf::Color(0xB1, 0xB1, 0xB1)
#define BOT_OUTLINE_COLOR sf::Color(0x47, 0x47, 0x47)
#define BOT_SENSOR_COLOR sf::Color(0x32, 0xCD, 0x32)

static std::default_random_engine gen;
static std::normal_distribution<float> normal(0.f, 1.f);

enum state {
	MANUAL,
	NONE,
	SURVEY,
	PERIPHERAL,
	SWEEP
};

class Bot {
	private:
		Game * game;
		std::vector<Node> nodes;
		std::vector<Obstacle> obstacles;
		sf::Vector2f currentPos;
		float surveyRotateSpeed;
		float sensorTimestamp;
		float surveyTimestamp;

	public:
		float userRotateSpeed;
		float userMoveSpeed;
		state botState;
		sf::CircleShape shape;
		sf::CircleShape point;
		std::vector<sf::RectangleShape> sensors;

		Bot(Game * game);
		~Bot() { }
		void update(const float dt);
		void draw(const float dt);
		void rotate(const float angle);
		void move(const float distance);

		/* sense */
		void sense();
		bool intersect(sf::CircleShape bot, sf::RectangleShape wall);
		std::vector<sf::CircleShape> getPeripheralObstacles();
		float getAngle(sf::Vector2f location);
		std::vector<Obstacle *> getViewObstacles(std::vector<sf::Vector2f> locations);
		std::vector<sf::Vector2f> ellipticLocalization(std::vector<float> r1, std::vector<float> r2);

		/* navigation */
		void survey(const float dt, bool init = 0);
		void encircleRoom(const float dt, bool init = 0);
};

#endif