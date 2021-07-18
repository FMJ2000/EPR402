#ifndef NODE_HPP
#define NODE_HPP

#include <SFML/Graphics.hpp>

#define BOT_RADIUS 30.f
#define SHADE_COLOR sf::Color(0xFF, 0xF4, 0xC2)

class Node {
	public:
		sf::CircleShape shape;

		Node() { }
		Node(sf::Vector2f pos);
		~Node() { }
};

#endif