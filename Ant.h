#ifndef ANT_H_
#define ANT_H_

#include <list>

#include "Location.h"

/*
    struct for representing ants.
*/
struct Ant
{
	std::list<Location> queue;
    Location loc;

	Ant()
    {
    };

    Ant(Location l)
    {
        loc = l;
    };

	void updateQueue() {
		loc = queue.front();
		queue.pop_front();
	}

	bool idle() {
		return queue.empty();
	}
	
	Location destination() {
		queue.back();
	}
};

#endif //ANT_H_
