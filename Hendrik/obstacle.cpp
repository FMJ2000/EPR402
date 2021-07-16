#include <SFML/Graphics.hpp>
#include "obstacle.hpp"

Obstacle::Obstacle(sf::Vector2f pos) {
	this->shape.setPosition(pos);
	this->shape.setRadius(5.f);
	this->shape.setFillColor(sf::Color::Black);
}