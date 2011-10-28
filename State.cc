#include "State.h"

using namespace std;

//constructor
State::State()
{
    gameover = 0;
    turn = 0;
    bug.open("./debug.txt");
};

//deconstructor
State::~State()
{
    bug.close();
};

//sets the state up
void State::setup()
{
    grid = vector<vector<Square> >(rows, vector<Square>(cols, Square()));
	gridNextTurn = vector<vector<Square> >(rows, vector<Square>(cols, Square()));
};

Location State::randomLocation(Location origin, int min, int distance) {
	Location loc = Location((origin.row + randomWithNeg(min, distance) + rows) % rows,
							(origin.col + randomWithNeg(min, distance) + cols) % cols);
	if (passable(loc) && xAwayFromMyHill(4,loc))
		return loc;
	return randomLocation(origin, min, distance);
}

bool State::xAwayFromMyHill(int dis, Location current) {
	for(int i = 0;i<(int)myHills.size();i++)
		if(locDistance(myHills[i],current) <= dis)
			return false;
	return true;
}

int State::randomWithNeg(int min, int distance) {
	int r = (rand() % (2*distance)) - distance;
	r += (r >= 0) ? min : (-min);
	return r;
}

//resets all non-water squares to land and clears the bots ant vector
void State::reset()
{
	antsMap.clear();
	for(int i = 0;i<(int)myAnts.size();i++) {
		antsMap[myAnts[i].loc] = myAnts[i];
	}
    myAnts.clear();
    enemyAnts.clear();
    myHills.clear();
    enemyHills.clear();
    food.clear();
    for(int row=0; row<rows; row++)
        for(int col=0; col<cols; col++)
            if(!grid[row][col].isWater) {
                grid[row][col].reset();
				gridNextTurn[row][col].reset();
			}
};

//outputs move information to the engine
void State::makeMove(const Location &loc, int direction)
{
    cout << "o " << loc.row << " " << loc.col << " " << CDIRECTIONS[direction] << endl;

    Location nLoc = getLocation(loc, direction);
    grid[nLoc.row][nLoc.col].ant = grid[loc.row][loc.col].ant;
    grid[loc.row][loc.col].ant = -1;
};

void State::moveAnt(Ant &a)
{
	Location loc = a.loc;
		
	int direction = directionFromPoints(a.loc,a.queue.front());

	bug << "moving ant at " << a.loc << CDIRECTIONS[direction] << endl;

	cout << "o " << loc.row << " " << loc.col << " " << CDIRECTIONS[direction] << endl;

    Location nLoc = getLocation(loc, direction);
    grid[nLoc.row][nLoc.col].ant = grid[loc.row][loc.col].ant;
    grid[loc.row][loc.col].ant = -1;

	a.updateQueue();
};

void State::makeMoves() {
	for(int i = 0;i<(int)myAnts.size();i++) {
		if(!myAnts[i].idle())
			moveAnt(myAnts[i]);
		else
			bug << "ant at " << myAnts[i].loc << " idle" << endl;
	}
}
  
  
int State::locDistance(const Location &loc1, const Location &loc2) {
	return modDistance(rows, loc1.row, loc2.row) + modDistance(cols, loc1.col, loc2.col);
}

int State::modDistance(int m, int x, int y) {
	int a = abs(x - y);
	return (a < (m - a)) ? a : (m - a);
}

//returns the euclidean distance between two locations with the edges wrapped
double State::distance(const Location &loc1, const Location &loc2)
{
    int d1 = abs(loc1.row-loc2.row),
        d2 = abs(loc1.col-loc2.col),
        dr = min(d1, rows-d1),
        dc = min(d2, cols-d2);
    return sqrt(dr*dr + dc*dc + 0.0);
};

//returns the new location from moving in a given direction with the edges wrapped
Location State::getLocation(const Location &loc, int direction)
{
    return Location( (loc.row + DIRECTIONS[direction][0] + rows) % rows,
                     (loc.col + DIRECTIONS[direction][1] + cols) % cols );
};

vector<Location> State::validNeighbors(const Location &current, const Location &start) {
	vector<Location> valid;

	for(int i = 0;i<4;i++) {
		Location loc = getLocation(current, i);
		if (((current == start) ? passableNextTurn(loc) : passable(loc)) && !isOnMyHill(loc))
			valid.push_back(loc);
	}
		
	return valid;
};

bool State::isOnMyHill(const Location &current) {
	for(int i = 0;i<(int)myHills.size();i++)
		if(myHills[i] == current)
			return true;
	return false;
}

bool State::passableNextTurn(const Location &loc) {
	if (grid[loc.row][loc.col].isWater)
		return false;
	if (gridNextTurn[loc.row][loc.col].ant == 0)
		return false;
	return true;
};

bool State::passable(const Location &loc) {
	return (!grid[loc.row][loc.col].isWater);
};


/*
    This function will update update the lastSeen value for any squares currently
    visible by one of your live ants.

    BE VERY CAREFUL IF YOU ARE GOING TO TRY AND MAKE THIS FUNCTION MORE EFFICIENT,
    THE OBVIOUS WAY OF TRYING TO IMPROVE IT BREAKS USING THE EUCLIDEAN METRIC, FOR
    A CORRECT MORE EFFICIENT IMPLEMENTATION, TAKE A LOOK AT THE GET_VISION FUNCTION
    IN ANTS.PY ON THE CONTESTS GITHUB PAGE.
*/
void State::updateVisionInformation()
{
    std::queue<Location> locQueue;
    Location sLoc, cLoc, nLoc;

    for(int a=0; a<(int) myAnts.size(); a++)
    {
		sLoc = myAnts[a].loc;
        locQueue.push(sLoc);

        std::vector<std::vector<bool> > visited(rows, std::vector<bool>(cols, 0));
        grid[sLoc.row][sLoc.col].isVisible = 1;
        visited[sLoc.row][sLoc.col] = 1;

        while(!locQueue.empty())
        {
            cLoc = locQueue.front();
            locQueue.pop();

            for(int d=0; d<TDIRECTIONS; d++)
            {
                nLoc = getLocation(cLoc, d);

                if(!visited[nLoc.row][nLoc.col] && distance(sLoc, nLoc) <= viewradius)
                {
                    grid[nLoc.row][nLoc.col].isVisible = 1;
                    locQueue.push(nLoc);
                }
                visited[nLoc.row][nLoc.col] = 1;
            }
        }
    }
};

/*
    This is the output function for a state. It will add a char map
    representation of the state to the output stream passed to it.

    For example, you might call "cout << state << endl;"
*/
ostream& operator<<(ostream &os, const State &state)
{
    for(int row=0; row<state.rows; row++)
    {
        for(int col=0; col<state.cols; col++)
        {
            if(state.grid[row][col].isWater)
                os << '%';
            else if(state.grid[row][col].isFood)
                os << '*';
            else if(state.grid[row][col].isHill)
                os << (char)('A' + state.grid[row][col].hillPlayer);
            else if(state.grid[row][col].ant >= 0)
                os << (char)('a' + state.grid[row][col].ant);
            else if(state.grid[row][col].isVisible)
                os << '.';
            else
                os << '?';
        }
        os << endl;
    }

    return os;
};

//input function
istream& operator>>(istream &is, State &state)
{
    int row, col, player;
    string inputType, junk;

    //finds out which turn it is
    while(is >> inputType)
    {
        if(inputType == "end")
        {
            state.gameover = 1;
            break;
        }
        else if(inputType == "turn")
        {
            is >> state.turn;
            break;
        }
        else //unknown line
            getline(is, junk);
    }

    if(state.turn == 0)
    {
        //reads game parameters
        while(is >> inputType)
        {
            if(inputType == "loadtime")
                is >> state.loadtime;
            else if(inputType == "turntime") {
                is >> state.turntime;
				state.bug << "turntime: " << state.turntime << " ms"  << endl;
			}
            else if(inputType == "rows")
                is >> state.rows;
            else if(inputType == "cols")
                is >> state.cols;
            else if(inputType == "turns")
                is >> state.turns;
            else if(inputType == "player_seed") {
                is >> state.seed;
				//srand ( state.seed );
				srand ( time(NULL) );
			}
            else if(inputType == "viewradius2")
            {
                is >> state.viewradius;
                state.viewradius = sqrt(state.viewradius);
            }
            else if(inputType == "attackradius2")
            {
                is >> state.attackradius;
                state.attackradius = sqrt(state.attackradius);
            }
            else if(inputType == "spawnradius2")
            {
                is >> state.spawnradius;
                state.spawnradius = sqrt(state.spawnradius);
            }
            else if(inputType == "ready") //end of parameter input
            {
                state.timer.start();
                break;
            }
            else    //unknown line
                getline(is, junk);
        }
    }
    else
    {
        //reads information about the current turn
        while(is >> inputType)
        {
            if(inputType == "w") //water square
            {
                is >> row >> col;
                state.grid[row][col].isWater = 1;
            }
            else if(inputType == "f") //food square
            {
                is >> row >> col;
                state.grid[row][col].isFood = 1;
                state.food.push_back(Location(row, col));
            }
            else if(inputType == "a") //live ant square
            {
                is >> row >> col >> player;
                state.grid[row][col].ant = player;
                if(player == 0) {
					Location loc = Location(row, col);
					state.bug << "new ant at " << loc;
					map<Location,Ant>::iterator it = state.antsMap.find(loc);
					if (it == state.antsMap.end()) {
						state.bug << " not found in map" << endl;
						state.myAnts.push_back(Ant(loc));
					}
					else {
						state.bug << " found in map" << endl;
						state.myAnts.push_back(it -> second);
					}
					Location nLoc = state.myAnts.back().positionNextTurn();
					state.gridNextTurn[nLoc.row][nLoc.col].ant = 0;
				} else
                    state.enemyAnts.push_back(Location(row, col));
            }
            else if(inputType == "d") //dead ant square
            {
                is >> row >> col >> player;
                state.grid[row][col].deadAnts.push_back(player);
            }
            else if(inputType == "h")
            {
                is >> row >> col >> player;
                state.grid[row][col].isHill = 1;
                state.grid[row][col].hillPlayer = player;
                if(player == 0)
                    state.myHills.push_back(Location(row, col));
                else
                    state.enemyHills.push_back(Location(row, col));

            }
            else if(inputType == "players") //player information
                is >> state.noPlayers;
            else if(inputType == "scores") //score information
            {
                state.scores = vector<double>(state.noPlayers, 0.0);
                for(int p=0; p<state.noPlayers; p++)
                    is >> state.scores[p];
            }
            else if(inputType == "go") //end of turn input
            {
                if(state.gameover)
                    is.setstate(std::ios::failbit);
                else
                    state.timer.start();
                break;
            }
            else //unknown line
                getline(is, junk);
        }
    }

    return is;
};

// Pathfinding 

void State::setAntQueue(Ant &a, list<Location> q) {
	Location oLoc = a.positionNextTurn();
	a.queue = q;
	Location nLoc = a.positionNextTurn();
	
	gridNextTurn[nLoc.row][nLoc.col].ant = 0;
    gridNextTurn[oLoc.row][oLoc.col].ant = -1;
}

bool State::locationEq(Location a, Location b) {
	if (a.row == b.row && a.col == b.col)
		return true;
	return false;
};

list<Location> State::reconstruct_path(map<Location, Location> cameFrom, Location prev) {
	if (cameFrom.count(prev)>0) {
		list<Location> path = reconstruct_path(cameFrom, cameFrom[prev]);
		path.push_back(prev);
		return path;
	} else {
		list<Location> path;
		path.push_back(prev);
		return path;
	}
}

int State::heuristic_cost_estimate(Location start, Location goal) {
	return locDistance(start, goal);
};

//const char CDIRECTIONS[4] = {'N', 'E', 'S', 'W'};
int State::directionFromPoints(Location point1, Location point2) {
	if (point1.row == 0 && point2.row == (rows-1) )
		return 0;
	if (point2.row == 0 && point1.row == (rows-1) )
		return 2 ;
	if (point1.col == 0 && point2.col == (cols-1) ) 
		return 3; 
	if (point2.col == 0 && point1.col == (cols-1))  
		return 1;
	if (point1.row > point2.row )
		return 0;
	if (point2.row > point1.row ) 
		return 2;
	if (point1.col > point2.col )  
		return 3;
	return 1;
};

class heuristicCompare
{
  Location goal;
  int rows;
  int cols;

	public:
  heuristicCompare(const int& rowsp, const int& colsp, const Location& goalparam)
    {
		goal=goalparam;
		rows = rowsp;
		cols = colsp;
  }

  bool operator() (const Location& lhs, const Location& rhs) const
  {
    return locDistance(lhs, goal) > locDistance(rhs, goal);
  }

  int locDistance(const Location& loc1, const Location& loc2) const {
	return modDistance(rows, loc1.row, loc2.row) + modDistance(cols, loc1.col, loc2.col);
  }

	int modDistance(int m, int x, int y) const {
		int a = abs(x - y);
		return (a < (m - a)) ? a : (m - a);
	}
};

list<Location> State::bfs(Location start, Location goal) {
	typedef priority_queue<Location, vector<Location>, heuristicCompare> mypq_type;
	mypq_type openSet (heuristicCompare(rows, cols, goal));
	map<Location, bool> openSetMap;
	map<Location, bool> closedSet;
	map<Location, Location> cameFrom;
	map<Location, int> g_score;
	map<Location, int> f_score;
	map<Location, int> h_score;

	bug << "pathfinding from " << start << " to " << goal << endl;

	openSet.push(start);
	
	g_score[start] = 0;
	h_score[start] = heuristic_cost_estimate(start, goal);
	f_score[start] = g_score[start] + h_score[start];
	
	while (!openSet.empty()) {
		Location current = openSet.top();
		
		openSet.pop();
		if (locationEq(current, goal))
			return reconstruct_path(cameFrom, goal);
			
		closedSet[current]=true;
		vector<Location> validNeighborsV = validNeighbors(current, start);
		
		for(int i = 0;i < (int)validNeighborsV.size();i++) {
			Location neighbor = validNeighborsV[i];
			
			if (closedSet.count(neighbor)>0)
				continue;
				
			int tentative_g_score = g_score[current] + locDistance(current,neighbor);
			bool tentativeIsBetter = false;
			
			if (! (openSetMap.count(neighbor)>0)) {
				openSet.push(neighbor);
				openSetMap[neighbor] = true;
				tentativeIsBetter = true;
			} else if (tentative_g_score < g_score[neighbor]) {
				tentativeIsBetter = true;
			} 
			
			if (tentativeIsBetter) {
				cameFrom[neighbor] = current;
				g_score[neighbor] = tentative_g_score;
				h_score[neighbor] = heuristic_cost_estimate(neighbor, goal);
				f_score[neighbor] = g_score[neighbor] + h_score[neighbor];
			}
		}

	}

	bug << "no path found" << endl;
	list<Location> empty;
	return empty;
};


// Action functions 

bool State::checkDestinations(vector<Location> destinations, Location destination) {
	for(int i = 0;i<(int)destinations.size();i++)
		if(destination == destinations[i])
			return true;
	return false;
}

void State::getFoods(vector<Ant*> &ants, list<Location> &food, int maxDistance) {
	vector<Ant*> tooFarAnts; 

	while(!ants.empty())
	{
		if (food.empty())
			break;

		Ant *a = ants.back();

		Location minFood = food.front();

		for (list<Location>::iterator foodIt=food.begin(); foodIt != food.end(); foodIt++ ){
			if ( locDistance((*a).loc, *foodIt) < locDistance((*a).loc, minFood))
				minFood = *foodIt;
		}

		if (locDistance((*a).loc,minFood) > maxDistance) {
			bug << "ant too far " << locDistance((*a).loc,minFood) << endl;
			tooFarAnts.push_back(a);
			ants.pop_back();
			continue;
		}

		food.remove(minFood);

		list<Location> path = bfs((*a).loc, minFood);

		if (! path.empty())
		{

#ifdef DEBUG
			list<Location>::iterator it;
			bug << "food path: ";
			for ( it=path.begin() ; it != path.end(); it++ )
				bug << *it << " ";
			bug << endl;
#endif

			path.pop_front();
			(*a).setFood();
			setAntQueue((*a), path);
		}
	}

	while(!tooFarAnts.empty()) {
		ants.push_back(tooFarAnts.back());
		tooFarAnts.pop_back();
	}
}

void State::killHills(vector<Ant*> &ants, vector<Location> &hills, int maxDistance) {
	if (!hills.empty())
		while(!ants.empty())
		{
			Ant *a = ants.back();

			ants.pop_back();

			list<Location> path = bfs((*a).loc, hills.back());

			#ifdef DEBUG
						list<Location>::iterator it;
						bug << "kill path: ";
						for ( it=path.begin() ; it != path.end(); it++ )
							bug << *it << " ";
						bug << endl;
			#endif

			if (! path.empty())
			{
				path.pop_front();
				(*a).setAttack();
				setAntQueue((*a), path);
			}
		}
}

void State::explore(Ant &ant, int mExpDis, int maxExpDis) {
	Location exploreDest = randomLocation(ant.loc, mExpDis, maxExpDis);

	bug << "exploring from " << ant.loc << " to " << exploreDest  << endl;

	list<Location> path = bfs(ant.loc, exploreDest);

	#ifdef DEBUG
		list<Location>::iterator it;
		bug << "explore path: ";
		for ( it=path.begin() ; it != path.end(); it++ )
			bug << *it << " ";
		bug << endl;
	#endif

	if (! path.empty())
	{
		path.pop_front();
		ant.setExplore();
		setAntQueue(ant, path);
	}
}

void State::goExplore(vector<Ant*> &ants, int mExpDis, int maxExpDis)
{
	//explore with additional ants if we have any
	while(!ants.empty())
	{
		Ant *a = ants.back();

		ants.pop_back();

		explore(*a, mExpDis, maxExpDis);
	}
}