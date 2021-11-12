#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <SFML/Graphics.hpp>

#define MAX_LIVES 3
#define REQ_CONF 2
#define MAX_EVAL_DIST 200.f

class Obstacle {
	public:
		sf::CircleShape shape;
		int confirmations;
		unsigned int lives;
		bool found;
		float distance;

		Obstacle() { }
		Obstacle(sf::Vector2f pos);
		~Obstacle() { }
};

#endif