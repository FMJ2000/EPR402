#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <SFML/Graphics.hpp>

class Obstacle {
	public:
		sf::CircleShape shape;

		Obstacle() { }
		Obstacle(sf::Vector2f pos);
		~Obstacle() { }
};

#endif