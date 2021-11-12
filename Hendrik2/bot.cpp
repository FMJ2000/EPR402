#include <SFML/Graphics.hpp>
#include <math.h>
#include <iostream>
#include <random>
#include "bot.hpp"

Bot::Bot(Game * game) {
	this->game = game;
	this->userRotateSpeed = 2.f;
	this->userMoveSpeed = 3.f;
	this->rotateSpeed = 120.f;
	this->moveSpeed = 60.f;
	this->turnAngleDesired = 0.f;
	this->turnAngle = 0.f;
	this->lastTurn = 1;

	/* bot shape */
	//this->shape = sf::CircleShape(30.f, 7);
	this->shape.setRadius(BOT_RADIUS);
	this->shape.setOutlineColor(BOT_OUTLINE_COLOR);
	this->shape.setFillColor(BOT_FILL_COLOR);
	this->shape.setOutlineThickness(5.f);
	this->shape.setOrigin(sf::Vector2f(this->shape.getRadius(), this->shape.getRadius()));
	this->shape.setPosition(sf::Vector2f(this->game->window.getSize().x / 2, this->game->window.getSize().y / 2));
	this->currentPos = this->shape.getPosition();
	this->shadowPos = this->shape.getPosition();

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
	sensor.setOrigin(sf::Vector2f(SENSOR_OFFSET + sensor.getSize().x, this->shape.getRadius()));
	this->sensors.push_back(sensor);
	sensor.setOrigin(sf::Vector2f(-SENSOR_OFFSET, this->shape.getRadius()));
	this->sensors.push_back(sensor);

	/* state and timers */
	this->sensorTimestamp = 0.f;
	this->surveyTimestamp = 0.f;
	this->botState.empty();
	this->botState.push(PERIPHERAL);
	this->botState.push(SURVEY);
}

void Bot::update(const float dt) {
	/* check if time to sense */
	this->sensorTimestamp += dt;
	if (this->sensorTimestamp >= SENSE_INTERVAL) this->sense();

	/* issue action based on state */
	switch (this->botState.top()) {
		case TURN: this->turn(dt); break;
		case SURVEY: this->survey(dt); break;
		case PERIPHERAL: this->encircleRoom(dt); break;
		default: break;
	}

	/* add node */
	float distance = sqrt(pow(this->shape.getPosition().x - this->shadowPos.x, 2) + pow(this->shape.getPosition().y - this->shadowPos.y, 2));
	if (distance >= BOT_RADIUS) {
		nodes.push_back(Node(this->shadowPos));
		this->shadowPos = this->shape.getPosition();
	}
}

void Bot::draw(const float dt) {
	for (auto node : this->nodes) this->game->window.draw(node.shape);
	for (auto obstacle : this->obstacles) this->game->window.draw(obstacle.shape);
	this->game->window.draw(this->shape);
	this->game->window.draw(this->point);
	for (auto sensor : this->sensors) this->game->window.draw(sensor);
}

void Bot::rotate(const float angle) {
	float noise = normal(gen) * 0.2f;
	this->shape.rotate(angle + noise);
	this->point.rotate(angle + noise);
	for (int i = 0; i < this->sensors.size(); i++) this->sensors[i].rotate(angle + noise);
}

void Bot::move(const float distance) {
	sf::Vector2f originalPos = this->shape.getPosition();
	float noise = normal(gen) * 0.2f;
	float x = (distance + noise) * (sin(-M_PI * this->shape.getRotation() / 180.f));
	float y = (distance + noise) * (cos(-M_PI * this->shape.getRotation() / 180.f));
	this->shape.move(sf::Vector2f(x, y));
	this->point.move(sf::Vector2f(x, y));
	for (int i = 0; i < this->sensors.size(); i++) this->sensors[i].move(sf::Vector2f(x, y));

	/* check for collisions */
	for (auto wall : this->game->walls) {
		if (this->intersect(this->shape, wall)) {
			this->shape.setPosition(originalPos);
			this->point.setPosition(originalPos);
			for (int i = 0; i < this->sensors.size(); i++) this->sensors[i].setPosition(originalPos);
			break;
		}
	}
}

void Bot::sense() {
	std::pair<std::vector<float>, std::vector<float>> distances = this->game->sense();		
	std::vector<sf::Vector2f> locations = this->ellipticLocalization(distances.first, distances.second);
	std::vector<Obstacle *> viewObstacles = this->getViewObstacles();
	
	for (auto location : locations) {
		bool found = false;
		for (size_t i = 0; i < viewObstacles.size(); i++) {
			float distance = sqrt(pow(viewObstacles[i]->shape.getPosition().x - location.x, 2) + pow(viewObstacles[i]->shape.getPosition().y - location.y, 2));
			if (distance <= OBSTACLE_MAX_DIST) {
				viewObstacles[i]->confirmations++;
				viewObstacles[i]->found = true;
				viewObstacles[i]->shape.setPosition(sf::Vector2f((viewObstacles[i]->shape.getPosition().x + location.x) / 2, (viewObstacles[i]->shape.getPosition().y + location.y) / 2));
				found = true;
				break;
			}
		}
		if (!found) this->obstacles.push_back(Obstacle(location));
	}

	for (size_t i = 0; i < this->obstacles.size(); i++) {
		if (this->obstacles[i].confirmations < REQ_CONF) this->obstacles[i].lives--;
		if (this->obstacles[i].lives <= 0) this->obstacles.erase(this->obstacles.begin() + i);
	}
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

std::vector<sf::CircleShape> Bot::getPeripheralObstacles() {
	sf::Vector2f pos = this->shape.getPosition();
	std::vector<sf::CircleShape> output;

	for (size_t i = 0; i < this->obstacles.size(); i++) {
		float distance = sqrt(pow(this->obstacles[i].shape.getPosition().x - pos.x, 2) + pow(this->obstacles[i].shape.getPosition().y - pos.y, 2));
		if (distance <= OBSTACLE_PERIPH) output.push_back(this->obstacles[i].shape);
	}

	return output;
}

float Bot::getAngle(sf::Vector2f location) {
	float shapeAngle = this->shape.getRotation() * M_PI / 180.f;
	float angle = atan2(location.y - this->shape.getPosition().y, location.x - this->shape.getPosition().x) + M_PI / 2.f;
	if (angle > M_PI) angle -= 2 * M_PI;
	std::cout << "shapeAngle: " << shapeAngle << ", angle: " << angle << std::endl;
	return angle;
}

std::vector<Obstacle *> Bot::getViewObstacles() {
	float angle1 = (this->shape.getRotation() - SENSOR_ANGLE) * M_PI / 180.f;
	float angle2 = (this->shape.getRotation() + SENSOR_ANGLE) * M_PI / 180.f;
	if (angle1 > M_PI) angle1 -= 2 * M_PI;
	if (angle2 > M_PI) angle2 -= 2 * M_PI;
	sf::Vector2f pos = this->shape.getPosition();
	std::vector<Obstacle *> viewObstacles;

	for (size_t i = 0; i < this->obstacles.size(); i++) {
		Obstacle * obstacle = &this->obstacles[i];
		obstacle->shape.setFillColor(sf::Color::Black);
		float angle = atan2(obstacle->shape.getPosition().y - pos.y, obstacle->shape.getPosition().x - pos.x) + M_PI / 2.f;
		if (angle > M_PI) angle -= 2 * M_PI;
		if ((angle1 < angle2 && angle >= angle1 && angle <= angle2) ||
			(angle1 > angle2 && (angle >= angle1 || angle <= angle2))) {
			obstacle->distance = sqrt(pow(obstacle->shape.getPosition().x - this->shape.getPosition().x, 2) + pow(obstacle->shape.getPosition().y - this->shape.getPosition().y, 2));
			if (obstacle->distance <= MAX_EVAL_DIST) {
				obstacle->found = false;
				obstacle->shape.setFillColor(sf::Color::Yellow);
				viewObstacles.push_back(obstacle);
			}
		}
	}

	return viewObstacles;
}

std::vector<sf::Vector2f> Bot::ellipticLocalization(std::vector<float> r1, std::vector<float> r2) {
	std::vector<sf::Vector2f> output;
	float angle = (this->shape.getRotation() - 90.f) * M_PI / 180.f;
	sf::Transform pos = this->sensors[0].getTransform();

	for (size_t i = 0; i < r1.size(); i++) {
		for (size_t j = 0; j < r2.size(); j++) {
			float newX = (r1[i] * r2[j] - pow(r2[j], 2)) / (4 * SENSOR_OFFSET);
			float newY = (sqrt(pow(r2[j], 2) - pow(2 * SENSOR_OFFSET, 2)) * sqrt(pow(2 * SENSOR_OFFSET, 2) - pow(r1[i] - r2[j], 2))) / (4 * SENSOR_OFFSET);
			if (!isnan(newX) && !isnan(newY)) {
				//std::cout << newX << ", " << newY << std::endl;
				output.push_back(pos.transformPoint(sf::Vector2f(newX, -newY)) - this->sensors[0].getSize());
			}
		}
	}
	//if (output.size() > 0) std::cout << output[0].x << ", " << output[0].y << std::endl;
	return output;
}

void Bot::turn(const float dt) {
	this->rotate(this->lastTurn * this->rotateSpeed * dt);
	this->turnAngle += this->rotateSpeed * dt;
	if (this->turnAngle >= this->turnAngleDesired) {
		this->turnAngle = 0.f;
		this->botState.pop();
	}
}

void Bot::moveTo(const float dt) {
	float angle = this->shape.getRotation() - atan2(this->desiredPos.y - this->shape.getPosition().y, this->desiredPos.x - this->shape.getPosition().x);
	std::cout << angle << std::endl;
}

void Bot::survey(const float dt) {
	this->rotate(this->rotateSpeed * dt);
	this->surveyTimestamp += dt;
	if (this->surveyTimestamp >= (360.f / this->rotateSpeed)) this->botState.pop();
}

void Bot::encircleRoom(const float dt) {
	/* get obstacles and analyse */
	float angle = this->shape.getRotation() * M_PI / 180.f;
	if (angle > M_PI) angle -= 2 * M_PI;
	sf::Vector2f botPos = this->shape.getPosition();//sf::Vector2f(this->shape.getPosition().x + this->shape.getRadius() * sin(angle), this->shape.getPosition().y + this->shape.getRadius() * cos(angle));
	std::vector<Obstacle *> viewObstacles = getViewObstacles();
	float leftX = MAX_DIST, rightX = MAX_DIST, topY = MAX_DIST, minDist = MAX_DIST;
	for (auto obstacle : viewObstacles) {
		float distance = sqrt(pow(obstacle->shape.getPosition().x - botPos.x, 2) + pow(obstacle->shape.getPosition().y - botPos.y, 2));
		float gradient = atan2(obstacle->shape.getPosition().y - botPos.y, obstacle->shape.getPosition().x - botPos.x);
		if (gradient > M_PI) gradient -= 2 * M_PI;
		float y = distance * sin(abs(angle - gradient));
		float x = distance * cos(abs(angle - gradient));
		if (distance < minDist) minDist = distance;
		if (y < topY) topY = y;
		if (x < 0 && abs(x) < leftX) leftX = x;
		if (x >= 0 && x < rightX) rightX = x;
	}

	/* edge detection, if one edge is max, move into position next to other edge */
	if ((rightX == MAX_DIST && leftX < MAX_DIST) || (leftX == MAX_DIST && rightX < MAX_DIST)) {
		//this->desiredPos = sf::Vector2f()
	}
	
	if (topY >= BOT_RADIUS + 10.f) {
		this->move(-this->moveSpeed * dt);
		float distance = sqrt(pow(this->shape.getPosition().x - this->currentPos.x, 2) + pow(this->shape.getPosition().y - this->currentPos.y, 2));
		this->currentPos = this->shape.getPosition();
		//std::cout << "expected: " << this->moveSpeed * dt << ", distance: " << distance << std::endl;
		if (distance * 2 < this->moveSpeed * dt) {
			this->turnAngleDesired = rand() % 10 - 5;
			this->botState.push(TURN);
		}
	} else {
		this->turnAngleDesired = 30.f;
		this->botState.push(TURN);
	}

	std::cout << "pos: (" << botPos.x << ", " << botPos.y << "), d: " << minDist << ", y: " << topY << ", left: " << leftX << ", right: " << rightX << std::endl;
}
