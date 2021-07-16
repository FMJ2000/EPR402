#include <SFML/Graphics.hpp>
#include "obstacle.hpp"

Obstacle::Obstacle(sf::Vector2f pos) {
	this->confirmations = 0;
	this->lives = MAX_LIVES;
	this->found = false;
	this->shape.setPosition(pos);
	this->shape.setRadius(5.f);
	this->shape.setFillColor(sf::Color::Black);
}