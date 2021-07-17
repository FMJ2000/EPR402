#include <SFML/Graphics.hpp>
#include <math.h>
#include <iostream>
#include <time.h>
#include <random>
#include "game.hpp"
#include "bot.hpp"

Game::Game() {
	sf::ContextSettings settings;
	settings.antialiasingLevel = 4;
	this->window.create(sf::VideoMode(1200, 800), GAME_NAME, sf::Style::Default, settings);
	this->window.setFramerateLimit(60);
	this->bot = new Bot(this);
	createOuterWalls();
}

Game::~Game() {
	delete this->bot;
}

void Game::gameloop() {
	sf::Clock clock;
	while (this->window.isOpen()) {
		sf::Time elapsed = clock.restart();
		float dt = elapsed.asSeconds();
		
		handleInput();
		update(dt);
		draw(dt);
	}
}

void Game::handleInput() {
	sf::Event event;
	while (this->window.pollEvent(event)) {
		if (event.type == sf::Event::Closed) this->window.close();
		if (event.type == sf::Event::KeyPressed) {
			if (event.key.code == sf::Keyboard::Q) this->window.close();
		}
	}

	if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) this->bot->move(-1);
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) this->bot->move(1);
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) this->bot->rotate(-1);
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) this->bot->rotate(1);
}

void Game::update(const float dt) {
	this->bot->update(dt);
}

void Game::draw(const float dt) {
	this->window.clear(BACKGROUND_COLOR);
	for (auto wall : this->walls) this->window.draw(wall);
	this->bot->draw(dt);
	this->window.display();
}

void Game::createOuterWalls() {
	srand(time(NULL));
	float size = 30.f;
	sf::RectangleShape wall;
	wall.setSize(sf::Vector2f(size, size));
	wall.setFillColor(WALL_COLOR);

	/* outer walls */
	sf::Vector2f dimensions = sf::Vector2f(this->window.getSize());
	for (size_t i = 0; i < ceil(dimensions.x / size); i++) {
		wall.setPosition(sf::Vector2f(i * size, 0));
		this->walls.push_back(wall);
		wall.setPosition(sf::Vector2f(i * size, dimensions.y - size));
		this->walls.push_back(wall);
	}
	for (size_t i = 0; i < ceil(dimensions.y / size); i++) {
		wall.setPosition(sf::Vector2f(0, i * size));
		this->walls.push_back(wall);
		wall.setPosition(sf::Vector2f(dimensions.x - size, i * size));
		this->walls.push_back(wall);
	}

	/* obstacles */
	wall.setSize(sf::Vector2f(60.f, 60.f));
	for (size_t i = 0; i < 10; i++) {
		sf::Vector2f pos = sf::Vector2f(rand() % int(dimensions.x - wall.getSize().x), rand() % int(dimensions.y - wall.getSize().y));
		if (abs(pos.x - this->window.getSize().x / 2) <= 50.f) pos.x += 100.f;
		if (abs(pos.y - this->window.getSize().y / 2) <= 50.f) pos.y += 100.f;
		wall.setPosition(pos);
		this->walls.push_back(wall);
	}

	this->walls.push_back(wall);
}

std::pair<std::vector<float>, std::vector<float>> Game::sense() {
	/* acquire minimum points */
	std::vector<sf::RectangleShape> obstacles = this->getViewObstacles();
	std::pair<std::vector<float>, std::vector<float>> output(SENSOR_SAMPLES, SENSOR_SAMPLES);
	std::fill_n(output.first.begin(), SENSOR_SAMPLES, 10000);
	std::fill_n(output.second.begin(), SENSOR_SAMPLES, 10000);
	sf::Vector2f pos1 = this->bot->sensors[0].getTransform().transformPoint(sf::Vector2f(0, 0));
	sf::Vector2f pos2 = this->bot->sensors[1].getTransform().transformPoint(sf::Vector2f(0, 0));

	for (auto obstacle : obstacles) {
		std::vector<float> distance1v;
		std::vector<float> distance2v;

		distance1v.push_back(2 * sqrt(pow(obstacle.getPosition().x - pos1.x, 2) + pow(obstacle.getPosition().y - pos1.y, 2)));
		distance1v.push_back(2 * sqrt(pow(obstacle.getPosition().x + obstacle.getSize().x - pos1.x, 2) + pow(obstacle.getPosition().y - pos1.y, 2)));
		distance1v.push_back(2 * sqrt(pow(obstacle.getPosition().x - pos1.x, 2) + pow(obstacle.getPosition().y + obstacle.getSize().y - pos1.y, 2)));
		distance1v.push_back(2 * sqrt(pow(obstacle.getPosition().x + obstacle.getSize().x - pos2.x, 2) + pow(obstacle.getPosition().y + obstacle.getSize().y - pos1.y, 2)));
		distance2v.push_back(distance1v[0] / 2.f + sqrt(pow(obstacle.getPosition().x - pos2.x, 2) + pow(obstacle.getPosition().y - pos2.y, 2)));
		distance2v.push_back(distance1v[1] / 2.f + sqrt(pow(obstacle.getPosition().x + obstacle.getSize().x - pos2.x, 2) + pow(obstacle.getPosition().y - pos2.y, 2)));
		distance2v.push_back(distance1v[2] / 2.f + sqrt(pow(obstacle.getPosition().x - pos2.x, 2) + pow(obstacle.getPosition().y + obstacle.getSize().y - pos2.y, 2)));
		distance2v.push_back(distance1v[3] / 2.f + sqrt(pow(obstacle.getPosition().x + obstacle.getSize().x - pos1.x, 2) + pow(obstacle.getPosition().y + obstacle.getSize().y - pos2.y, 2)));
		
		std::sort(distance1v.begin(), distance1v.end());
		std::sort(distance2v.begin(), distance2v.end());
		int index1 = 0, index2 = 0;

		for (size_t i = 0; i < SENSOR_SAMPLES; i++) {
			if (output.first[i] > distance1v[index1]) {
				output.first[i] = distance1v[index1];
				if (index1 < distance1v.size() - 1) index1++;
				else break;
			}
		}
		for (size_t i = 0; i < SENSOR_SAMPLES; i++) {
			if (output.second[i] > distance2v[index2]) {
				output.second[i] = distance2v[index2];
				if (index2 < distance2v.size() - 1) index2++;
				else break;
			}
		}
	}
	std::sort(output.first.begin(), output.first.end());
	std::sort(output.second.begin(), output.second.end());
	for (size_t i = 0; i < output.first.size(); i++) {
		if (output.first[i] > SENSOR_MAX) {
			output.first.erase(output.first.begin() + i, output.first.end());
			break;
		}
	}
	for (size_t i = 0; i < output.second.size(); i++) {
		if (output.second[i] > SENSOR_MAX) {
			output.second.erase(output.second.begin() + i, output.second.end());
			break;
		}
	}

	return output;
}

std::vector<sf::RectangleShape> Game::getViewObstacles() {
	float angle1 = (this->bot->shape.getRotation() - SENSOR_ANGLE) * M_PI / 180.f;
	float angle2 = (this->bot->shape.getRotation() + SENSOR_ANGLE) * M_PI / 180.f;
	if (angle1 > M_PI) angle1 -= 2 * M_PI;
	if (angle2 > M_PI) angle2 -= 2 * M_PI;
	sf::Vector2f pos = this->bot->shape.getPosition();
	std::vector<sf::RectangleShape> viewObstacles;

	for (size_t i = 0; i < this->walls.size(); i++) {
		sf::RectangleShape * wall = &this->walls[i];
		wall->setFillColor(WALL_COLOR);
		float angle = atan2(wall->getPosition().y - pos.y, wall->getPosition().x - pos.x) + M_PI / 2.f;
		if (angle > M_PI) angle -= 2 * M_PI;
		if ((angle1 < angle2 && angle >= angle1 && angle <= angle2) ||
			(angle1 > angle2 && (angle >= angle1 || angle <= angle2))) {
			wall->setFillColor(WALL_SENSE_COLOR);
			viewObstacles.push_back(*wall);
		}
	}

	return viewObstacles;
}