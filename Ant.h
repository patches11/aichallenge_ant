#ifndef ANT_H_
#define ANT_H_

#include <list>
#include <string>

#include "Location.h"

const std::string CROLES[4] = {"Explore","Food","Attack","Defend"};

/*
    struct for representing ants.
*/
struct Ant
{
	std::list<Location> queue;
    Location loc;
	Location rDestination;//Interrupted Destination, if Ant is sent to get close food
	Location iDestination;//Real destination, may not match queue.back() because of interrupted pathfinding
	int intRole, role, owner, defendCounter;
	int turnsRetreating, retreatCount;

	Ant()
    {
		role = -1;
		intRole = -1;
		owner = 0;
		defendCounter = 0;
		turnsRetreating = retreatCount = 0;
    };

    Ant(Location l)
    {
        loc = l;
		role = -1;
		intRole = -1;
		owner = 0;
		defendCounter = 0;
		turnsRetreating = retreatCount = 0;
    };

	Ant(Location l, int o)
    {
        loc = l;
		role = -1;
		intRole = -1;
		owner = o;
		defendCounter = 0;
		turnsRetreating = retreatCount = 0;
    };

	void countRetreating() {
		if (retreatCount > 0) {
			retreatCount--;
			turnsRetreating++;
		} else {
			turnsRetreating = 0;
		}
	}

	std::string roleText() {
		if (role == -1)
			return "idle";
		else
			return CROLES[role];
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
		defendCounter = 0;
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
		turnsRetreating = retreatCount = 0;
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
