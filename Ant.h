#ifndef ANT_H_
#define ANT_H_

#include <list>

#include "Location.h"

const std::string CROLES[4] = {"Explore","Food","Attack"};

/*
    struct for representing ants.
*/
struct Ant
{
	std::list<Location> queue;
    Location loc;
	int role;

	Ant()
    {
    };

    Ant(Location l)
    {
        loc = l;
		role = -1;
    };

	void setExplore() {
		role = 0;
	}

	void setFood() {
		role = 1;
	}

	void setAttack() {
		role = 2;
	}

	void updateQueue() {
		loc = queue.front();
		queue.pop_front();
	}

	bool idle() {
		return queue.empty();
	}

	bool exploring() {
		return (!queue.empty() && role == 0);
	}
	
	Location destination() {
		if (idle())
			return loc;
		return queue.back();
	}

	Location positionNextTurn() {
		if (idle())
			return loc;
		return queue.front();
	}
};

#endif //ANT_H_
