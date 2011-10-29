#ifndef STATE_H_
#define STATE_H_

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <stdint.h>
#include <map>

#include "Timer.h"
#include "Bug.h"
#include "Square.h"
#include "Location.h"
#include "Ant.h"

/*
    constants
*/
const int TDIRECTIONS = 4;
const char CDIRECTIONS[4] = {'N', 'E', 'S', 'W'};
const int DIRECTIONS[4][2] = { {-1, 0}, {0, 1}, {1, 0}, {0, -1} };      //{N, E, S, W}

/*
    struct to store current state information
*/
struct State
{
    /*
        Variables
    */
    int rows, cols,
        turn, turns,
        noPlayers;
    double attackradius, spawnradius, viewradius;
    double loadtime, turntime;
    std::vector<double> scores;
    bool gameover;
    int64_t seed;

    std::vector<std::vector<Square> > grid;
	std::vector<std::vector<Square> > gridNextTurn;
    std::vector<Location> enemyAnts, myHills, enemyHills, food;
	std::vector<Ant> myAnts;
    std::map<Location, Ant> antsMap;

    Timer timer;
    Bug bug;

    /*
        Functions
    */
    State();
    ~State();

    void setup();
    void reset();

    void makeMove(const Location &loc, int direction);
	void makeMoves();

	void moveAnt(Ant &a);

    double distance(const Location &loc1, const Location &loc2);
    Location getLocation(const Location &startLoc, int direction);

	int locDistance(const Location &loc1, const Location &loc2);
	int modDistance(int m, int x, int y);

	std::vector<Location> validNeighbors(const Location &current, const Location &start);

    void updateVisionInformation();

	bool locationEq(Location a, Location b);
	
	std::list<Location> reconstruct_path(std::map<Location, Location> cameFrom, Location prev);

	int heuristic_cost_estimate(Location start, Location goal);

	int directionFromPoints(Location point1, Location point2);

	std::list<Location> bfs(Location start, Location goal);

	bool passable(const Location &loc);

	Location randomLocation(Location origin, int min, int distance) ;

	int randomWithNeg(int min, int distance);

	bool passableNextTurn(const Location &loc);

	void setAntQueue(Ant &a, std::list<Location> q);

	bool isOnMyHill(const Location &current);

	bool xAwayFromMyHill(int dis, Location current);

	// Action functions

	bool checkDestinations(std::vector<Location> destinations, Location destination);

	void getFoods(std::vector<Ant*> &ants, std::list<Location> &food, int maxDistance);

	void killHills(std::vector<Ant*> &ants, std::vector<Location> &hills, int maxDistance);

	void explore(Ant &ant, int mExpDis, int maxExpDis);

	void goExplore(std::vector<Ant*> &ants, int mExpDis, int maxExpDis);

	void rerouteAnt(Ant &ant);
};

std::ostream& operator<<(std::ostream &os, const State &state);
std::istream& operator>>(std::istream &is, State &state);

#endif //STATE_H_
