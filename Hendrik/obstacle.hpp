#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <SFML/Graphics.hpp>

#define MAX_LIVES 3
#define REQ_CONF 2

class Obstacle {
	public:
		sf::CircleShape shape;
		int confirmations;
		unsigned int lives;
		bool found;
		float distance;
		int index;

		Obstacle() { }
		Obstacle(sf::Vector2f pos);
		~Obstacle() { }
};

#endif