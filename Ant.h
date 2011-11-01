#ifndef ANT_H_
#define ANT_H_

#include <list>

#include "Location.h"

const std::string CROLES[4] = {"Explore","Food","Attack","Defend"};

/*
    struct for representing ants.
*/
struct Ant
{
	std::list<Location> queue;
    Location loc;
	Location rDestination;//Interrupted Destination
	Location iDestination;//Real destination
	int intRole, role, owner;

	Ant()
    {
    };

    Ant(Location l)
    {
        loc = l;
		role = -1;
		intRole = -1;
    };

	Ant(Location l, int o)
    {
        loc = l;
		role = -1;
		intRole = -1;
		owner = o;
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

	bool pathfindingIncomplete() {
		return !(iDestination == queue.back());
	}

	bool wasInterrupted() {
		return (intRole != -1);
	}

	void setDefend() {
		role = 3;
	}

	bool isDefending() {
		return role == 3;
	}

	bool isAttacking() {
		return role == 2;
	}

	bool isExploring() {
		return role == 0;
	}

	bool isGettingFood() {
		return role == 1;
	}

	void setIdle() {
		role = -1;
		queue.clear();
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
		return iDestination;
	}

	Location positionNextTurn() {
		if (idle())
			return loc;
		return queue.front();
	}
};

#endif //ANT_H_
