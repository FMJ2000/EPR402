#ifndef BOT_HPP
#define BOT_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <random>
#include <stack>
#include "game.hpp"
#include "obstacle.hpp"
#include "node.hpp"

#define PWM_T 0xFFF
#define VEL_MAX 48                     // maximum wheel spin speed (rad) if duty 100%
#define WHEEL_RADIUS 0.032
#define WHEEL_HOLES 20.0
#define ODO_SCALE M_PI / WHEEL_HOLES
#define CHASSIS_LENGTH 0.15
#define ALPHA 0.1
#define BETA M_PI / 2.0
#define GAMMA 0.1

// sensor fusion
#define K_ODO 0.8
#define K_ACC 0.2
#define K_GYRO 0.8
#define K_MAG 0.02
#define K_VEL 1.5

// pi controller
#define K_OFFSET 0.18
#define K_INT 0.09
#define K_TURN 4
#define K_DIST 0.005

// FIR filter
#define FIR_F 10.0
#define FIR_FP 0.2
#define FIR_FS 2.0
#define FIR_N 18

// mapping
#define DIR_DIV 0.785398				// 2*pi / 8
#define DIR_OFFSET 4.71239				// 3*pi / 2

// navigation
#define ERROR_COLLIDE 0.785498          // 45 deg
#define ERROR_MIN 0.3491                // 20 deg
#define ERROR_MAX 2.7925                // 160 deg
#define MIN_DIST 0.04
#define MIN_OBST_DIST 0.1
#define MIN_GOAL_DIST 0.1
#define MAX_GOAL_DIST 0.3
#define MIN_US_DIST 0.03                // minimum distance visible
#define MAX_US_DIST 0.9
#define MIN_PWM 0.1

#define Q_VAL 0.5
#define R_VAL 0.5
#define INPUTQ_SIZE 50
#define ADC_MAX 192
#define ADC_MIN 120
#define ADC_SCALE 100.0 / (ADC_MAX - ADC_MIN)
#define DIST_CORR_OFFSET 0.03
#define ODO_WEIGHT 0.8
 
#define US_SENSORS 3
#define SENSOR_OFFSET 40. * M_PI / 180.  // angle offset between two sensors
#define SENSOR_ANGLE 15. * M_PI / 180.
#define SOUND_SPEED 1.65e-4              // TMR5*SOUND_SPEED for distance
#define MAP_SIZE 1.28
#define MAP_RES 0.08
#define MAP_UNITS 16
#define MAP_UNITS_BIT 4
#define DEFAULT_VAL 0x07
#define US_SIGMA 0.03
#define NAV_SIGMA 0.2618 / 2
#define N_RANDOM_TRIES 100
#define BIAS_MAX 1000

#define BUF_LEN 1024
#define OLED_LINE_LEN 24
#define GOAL_LEN 5

// state
#define INIT 0
#define IDLE 1
#define NAVIGATE 2
#define BATTERY 3
#define STATE_MASK 0x3
#define STATE_MASK0 0x1
#define STATE_MASK1 0x2
#define STATE_UART_MASK 0x4
#define STATE_TX_MASK 0x8

#define BOT_FILL_COLOR sf::Color(0xB1, 0xB1, 0xB1)
#define BOT_OUTLINE_COLOR sf::Color(0x47, 0x47, 0x47)
#define BOT_SENSOR_COLOR sf::Color(0x32, 0xCD, 0x32)

static std::default_random_engine gen;
static std::normal_distribution<float> normal(0.f, 1.f);

enum state {
	MANUAL,
	NONE,
	SURVEY,
	PERIPHERAL,
	TURN,
	SWEEP
};

class Bot {
	private:
		Game * game;
		std::vector<Node> nodes;
		std::vector<Obstacle> obstacles;
		sf::Vector2f currentPos;
		sf::Vector2f desiredPos;
		sf::Vector2f shadowPos;
		float rotateSpeed;
		float moveSpeed;
		float sensorTimestamp;
		float surveyTimestamp;
		int lastTurn;

	public:
		float userRotateSpeed;
		float userMoveSpeed;
		float turnAngle;
		float turnAngleDesired;
		std::stack<state> botState;
		sf::CircleShape shape;
		sf::CircleShape point;
		std::vector<sf::RectangleShape> sensors;

		Bot(Game * game);
		~Bot() { }
		void update(const float dt);
		void draw(const float dt);
		void rotate(const float angle);
		void move(const float distance);

		/* sense */
		void sense();
		bool intersect(sf::CircleShape bot, sf::RectangleShape wall);
		std::vector<sf::CircleShape> getPeripheralObstacles();
		float getAngle(sf::Vector2f location);
		std::vector<Obstacle *> getViewObstacles();
		std::vector<sf::Vector2f> ellipticLocalization(std::vector<float> r1, std::vector<float> r2);

		/* navigation */
		void turn(const float dt);
		void moveTo(const float dt);
		void survey(const float dt);
		void encircleRoom(const float dt);
};

#endif