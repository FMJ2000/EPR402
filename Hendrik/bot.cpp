#include <SFML/Graphics.hpp>
#include <math.h>
#include <iostream>
#include <random>
#include "bot.hpp"

Bot::Bot(Game * game) {
	this->game = game;
	this->rotateSpeed = 2.f;
	this->moveSpeed = 3.f;

	/* bot shape */
	//this->shape = sf::CircleShape(30.f, 7);
	this->shape.setRadius(30.f);
	this->shape.setOutlineColor(BOT_OUTLINE_COLOR);
	this->shape.setFillColor(BOT_FILL_COLOR);
	this->shape.setOutlineThickness(5.f);
	this->shape.setOrigin(sf::Vector2f(this->shape.getRadius(), this->shape.getRadius()));
	this->shape.setPosition(sf::Vector2f(this->game->window.getSize().x / 2, this->game->window.getSize().y / 2));

	/* bot direction pointer */
	this->point = sf::CircleShape(5.f, 3);
	this->point.setFillColor(sf::Color::White);
	this->point.setOrigin(sf::Vector2f(this->point.getRadius(), this->shape.getRadius() - this->point.getRadius()));
	this->point.setPosition(this->shape.getPosition());

	/* bot sensors */
	sf::RectangleShape sensor;
	sensor.setSize(sf::Vector2f(4.f, 4.f));
	sensor.setFillColor(BOT_SENSOR_COLOR);
	sensor.setPosition(this->shape.getPosition());

	this->shadeBlock.setRadius(30.f);
	this->shadeBlock.setFillColor(SHADE_COLOR);
	this->shadeBlock.setOrigin(sf::Vector2f(this->shadeBlock.getRadius(), this->shadeBlock.getRadius()));
}

void Bot::draw(const float dt) {
	for (auto block : this->shade) this->game->window.draw(block);
	this->game->window.draw(this->shape);
	this->game->window.draw(this->point);
}

void Bot::rotate(const int dir) {
	float noise = normal(gen);
	this->shape.rotate(dir * (this->rotateSpeed + noise));
	this->point.rotate(dir * (this->rotateSpeed + noise));
}

void Bot::move(const int dir) {
	sf::Vector2f originalPos = this->shape.getPosition();
	float noise = normal(gen) / this->moveSpeed;
	float x = dir * (this->moveSpeed + noise) * (sin(-M_PI * this->shape.getRotation() / 180.f));
	float y = dir * (this->moveSpeed + noise) * (cos(-M_PI * this->shape.getRotation() / 180.f));
	this->shape.move(sf::Vector2f(x, y));
	this->point.move(sf::Vector2f(x, y));

	/* check for collisions */
	for (auto wall : this->game->walls) {
		if (this->intersect(this->shape, wall)) {
			this->shape.setPosition(originalPos);
			this->point.setPosition(originalPos);
			break;
		}
	}	

	/* add shade */
	bool add = true;
	for (auto block : shade) {
		if (pow(block.getPosition().x - originalPos.x, 2) < 100.f && pow(block.getPosition().y - originalPos.y, 2) < 100.f) {
			add = false;
			break;
		}
	}
	if (add) {
		this->shadeBlock.setPosition(originalPos);
		this->shade.push_back(this->shadeBlock);
	}

	this->sense();
}

void Bot::sense() {
	std::vector<sf::RectangleShape> obstacles = this->getViewObstacles();

}

bool Bot::intersect(sf::CircleShape bot, sf::RectangleShape wall) {
	/* get vector distances */
	sf::Vector2f vDistances[4];
	vDistances[0] = sf::Vector2f(wall.getPosition().x - bot.getPosition().x, wall.getPosition().y - bot.getPosition().y);
	vDistances[1] = sf::Vector2f(wall.getPosition().x + wall.getSize().x - bot.getPosition().x, wall.getPosition().y - bot.getPosition().y);
	vDistances[2] = sf::Vector2f(wall.getPosition().x - bot.getPosition().x, wall.getPosition().y + wall.getSize().y - bot.getPosition().y);
	vDistances[3] = sf::Vector2f(wall.getPosition().x + wall.getSize().x - bot.getPosition().x, wall.getPosition().y + wall.getSize().y - bot.getPosition().y);

	/* convert to absolute distance */
	float distances[4];
	float sum;
	for (size_t i = 0; i < 4; i++) {
		distances[i] = sqrt(pow(vDistances[i].x, 2) + pow(vDistances[i].y, 2));
		if (distances[i] <= bot.getRadius()) return true;
		sum += distances[i];
	}
	if (sum / 4.f < wall.getSize().x) return true;
	
	return false;
}

std::vector<sf::RectangleShape> Bot::getViewObstacles() {
	float angle1 = (this->shape.getRotation() - SENSOR_ANGLE) * M_PI / 180.f;
	float angle2 = (this->shape.getRotation() + SENSOR_ANGLE) * M_PI / 180.f;
	if (angle1 > M_PI) angle1 -= 2 * M_PI;
	if (angle2 > M_PI + (2 * SENSOR_ANGLE) * M_PI / 180.f) angle2 -= 2 * M_PI;
	sf::Vector2f pos = this->shape.getPosition();
	std::vector<sf::RectangleShape> viewObstacles;

	for (size_t i = 0; i < this->game->walls.size(); i++) {
		sf::RectangleShape * wall = &this->game->walls[i];
		wall->setFillColor(WALL_COLOR);
		float angle = atan2(wall->getPosition().y - pos.y, wall->getPosition().x - pos.x) + M_PI / 2.f;
		if (angle > M_PI) angle -= 2 * M_PI;
		if (angle >= angle1 && angle <= angle2) {
			wall->setFillColor(WALL_SENSE_COLOR);
			viewObstacles.push_back(*wall);
		}
	}

	return viewObstacles;
}