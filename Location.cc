#include "State.h"

using namespace std;	

ostream& operator<<(ostream &os, const Location &loc)
{
	os << '(' << (int) loc.row << ',' << ' ' << (int) loc.col << ')';

    return os;
}