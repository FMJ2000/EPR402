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
		wall.setPosition(pos);
		this->walls.push_back(wall);
	}

	this->walls.push_back(wall);
}