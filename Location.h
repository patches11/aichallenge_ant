#ifndef LOCATION_H_
#define LOCATION_H_

#include <iostream>
#include <stdio.h>

/*
    struct for representing locations in the grid.
*/
struct Location
{
    int row, col;

    Location()
    {
        row = col = 0;
    };

    Location(int r, int c)
    {
        row = r;
        col = c;
    };

	friend bool operator< (Location first, Location second)
	{
	if (first.row < second.row)
		return true;
	if (first.row > second.row)
		return false;
	if (first.col < second.col)
		return true;
	return false;
	}

	friend bool operator== (Location first, Location second)
	{
		if (first.row == second.row && first.row == second.row)
			return true;
		return false;
	}

};

std::ostream& operator<<(std::ostream &os, const Location &loc);


#endif //LOCATION_H_
