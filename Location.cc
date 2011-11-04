#include "State.h"

using namespace std;	

ostream& operator<<(ostream &os, const Location &loc)
{
	os << '(' << (int) loc.col << ',' << ' ' <<  (int) loc.row << ')';

    return os;
}