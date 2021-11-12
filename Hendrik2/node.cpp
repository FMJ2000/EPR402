#include <SFML/Graphics.hpp>
#include "node.hpp"

Node::Node(sf::Vector2f pos) {
	this->shape.setRadius(BOT_RADIUS);
	this->shape.setFillColor(SHADE_COLOR);
	this->shape.setOrigin(sf::Vector2f(this->shape.getRadius(), this->shape.getRadius()));
	this->shape.setPosition(pos);
}