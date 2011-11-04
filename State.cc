#include "State.h"

using namespace std;

#define attackDistanceBuffer 8
#define searchStepLimit 550
#define minExploreDistanceFromHill 2
#define expLamda 0.55

//constructor
State::State()
{
    gameover = 0;
    turn = 0;
    bug.open("./debug.txt");
	hillsAtRisk = false;
	hasUnexplored = true;
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

bool State::outOfTime(double marginOfError) {
	return (timer.getTime() >= (turntime - marginOfError));
}

Location State::randomLocation(Location origin, int min, int distance) {
	Location loc = Location((origin.row + randomWithNeg(min, distance) + rows) % rows,
							(origin.col + randomWithNeg(min, distance) + cols) % cols);
	if (passable(loc) && xAwayFromMyHill(minExploreDistanceFromHill,loc))
		return loc;
	return randomLocation(origin, min, distance);
}

Location State::randomLocation() {
	vector<Location> unexplored;
	for (int row = 0;row<rows;row++)
		for(int col = 0;col<cols;col++)
			if (!grid[row][col].isExplored)
				unexplored.push_back(Location(row,col));
	if (unexplored.empty()) {
		hasUnexplored = false;
		return Location(-1,-1);
	}
	return unexplored.at(rand() % unexplored.size());
}

Location State::randomLocationExp(Location origin, int min, int distance) {
	Location loc = Location((origin.row + randomWithNegExp(min, distance) + rows) % rows,
							(origin.col + randomWithNegExp(min, distance) + cols) % cols);
	if (passable(loc) && xAwayFromMyHill(minExploreDistanceFromHill,loc))
		return loc;
	return randomLocationExp(origin, min, distance);
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
	bug << "random: " << r << endl;
	return r;
}

int State::randomWithNegExp(int min, int distance) {
	bool positive = rand() % 2 == 0 ? true : false;
	double x = (-lnApprox(((double)rand()/(double)RAND_MAX),25))/(expLamda);
	bug << "random Exp ln: " << x;
	x = x*distance + min;
	if (!positive)
		x = -x;
	bug << " result: " << (int)x << endl;
	return (int)x;
}

double State::lnApprox(double x, int steps) {
	double result = x - 1;
	for(int i=2;i<steps;i++)
		if (i % 2 == 0)
			result -= pow(x-1,steps)/steps;
		else
			result += pow(x-1,steps)/steps;
	return result;
}

double State::pow(double x, int y) {
	double result = x;
	if (y == 0)
		return 1;
	for(int i = 0;i<y;i++)
		result *= x;
	return result;
}

//resets all non-water squares to land and clears the bots ant vector
void State::reset()
{
	antsMap.clear();
	for(int i = 0;i<(int)myAnts.size();i++) {
		antsMap[myAnts[i].loc] = myAnts[i];
	}
	hillsAtRisk = false;
    myAnts.clear();
    enemyAnts.clear();
    myHills.clear();
    enemyHills.clear();
    food.clear();
    for(int row=0; row<rows; row++)
        for(int col=0; col<cols; col++)
            if(!grid[row][col].isWater) {
                grid[row][col].reset();
				gridNextTurn[row][col].ant = 0;
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

	bug << "moving ant at " << a.loc << " " << CDIRECTIONS[direction] << " " << a.roleText() << endl;

	cout << "o " << loc.row << " " << loc.col << " " << CDIRECTIONS[direction] << endl;

	a.countRetreating();

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
    return sqrt(distanceSq(loc1, loc2));
};

//returns the square of the euclidean distance between two locations with the edges wrapped
double State::distanceSq(const Location &loc1, const Location &loc2)
{
    int d1 = abs(loc1.row-loc2.row),
        d2 = abs(loc1.col-loc2.col),
        dr = min(d1, rows-d1),
        dc = min(d2, cols-d2);
    return dr*dr + dc*dc;
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
		if (((current == start) ? passableNextTurn(loc) : passable(loc)) && !isOnMyHill(loc))//avoiding routefinding on the hill can cause an ant to get stuck if the hill is near a single passable square with no other way out
			valid.push_back(loc);
	}
		
	return valid;
};

vector<Location> State::validNeighbors(const Location &current) {
	vector<Location> valid;

	for(int i = 0;i<4;i++) {
		Location loc = getLocation(current, i);
		if (passable(loc))
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
	if (gridNextTurn[loc.row][loc.col].ant>0)
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

        std::vector<std::vector<bool> > visited(rows, std::vector<bool>(cols, false));
        grid[sLoc.row][sLoc.col].isVisible = true;
        visited[sLoc.row][sLoc.col] = true;

        while(!locQueue.empty())
        {
            cLoc = locQueue.front();
            locQueue.pop();

            for(int d=0; d<TDIRECTIONS; d++)
            {
                nLoc = getLocation(cLoc, d);

                if(!visited[nLoc.row][nLoc.col] && distance(sLoc, nLoc) <= viewradius)
                {
                    grid[nLoc.row][nLoc.col].isVisible = true;
					grid[nLoc.row][nLoc.col].isExplored = true;
                    locQueue.push(nLoc);
                }
                visited[nLoc.row][nLoc.col] = true;
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
				srand ( (int) time(NULL) );
			}
            else if(inputType == "viewradius2")
            {
                is >> state.viewradius;
				state.viewradius2 = state.viewradius;
                state.viewradius = sqrt(state.viewradius);
            }
            else if(inputType == "attackradius2")
            {
                is >> state.attackradius;
				state.attackradius2 = state.attackradius;
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
                state.grid[row][col].isWater = true;
            }
            else if(inputType == "f") //food square
            {
                is >> row >> col;
                state.grid[row][col].isFood = true;
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
						state.myAnts.push_back(Ant(loc,0));
					}
					else {
						state.bug << " found in map" << endl;
						state.myAnts.push_back(it -> second);
					}
					Location nLoc = state.myAnts.back().positionNextTurn();
					state.gridNextTurn[nLoc.row][nLoc.col].ant++;
				} else
                    state.enemyAnts.push_back(Ant(Location(row, col),player));
            }
            else if(inputType == "d") //dead ant square
            {
                is >> row >> col >> player;
                state.grid[row][col].deadAnts.push_back(player);
            }
            else if(inputType == "h")
            {
                is >> row >> col >> player;
                state.grid[row][col].isHill = true;
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

void State::setAntQueue(Ant &a, list<Location> q, Location destination) {
	Location oLoc = a.positionNextTurn();
	a.queue = q;
	a.iDestination = destination;
	Location nLoc = a.positionNextTurn();
	
	gridNextTurn[nLoc.row][nLoc.col].ant++;
    gridNextTurn[oLoc.row][oLoc.col].ant--;
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

struct LocationContainer
{
	Location l;
	int f_score;
	int g_score;

	LocationContainer() {
		f_score = 0;
	}

	public:

	LocationContainer(Location ls, int f, int g) {
		l = ls;
		f_score = f;
		g_score = g;
	}

	int getf_score() const {
		return f_score;
	}
};

class locContainerCompare 
{

public:

	locContainerCompare() {

	}

	bool operator() (const LocationContainer& lhs, const LocationContainer& rhs)
	{
	 return (lhs.f_score > rhs.f_score);
	}
};


/*

function A*(start,goal)
     closedset := the empty set    // The set of nodes already evaluated.
     openset := {start}    // The set of tentative nodes to be evaluated, initially containing the start node
     came_from := the empty map    // The map of navigated nodes.
 
     g_score[start] := 0    // Cost from start along best known path.
     h_score[start] := heuristic_cost_estimate(start, goal)
     f_score[start] := g_score[start] + h_score[start]    // Estimated total cost from start to goal through y.
 
     while openset is not empty
         x := the node in openset having the lowest f_score[] value
         if x = goal
             return reconstruct_path(came_from, came_from[goal])
 
         remove x from openset
         add x to closedset
         foreach y in neighbor_nodes(x)
             if y in closedset
                 continue
             tentative_g_score := g_score[x] + dist_between(x,y)
 
             if y not in openset
                 add y to openset
                 tentative_is_better := true
             else if tentative_g_score < g_score[y]
                 tentative_is_better := true
             else
                 tentative_is_better := false
 
             if tentative_is_better = true
                 came_from[y] := x
                 g_score[y] := tentative_g_score
                 h_score[y] := heuristic_cost_estimate(y, goal)
                 f_score[y] := g_score[y] + h_score[y]
 
     return failure
 
 function reconstruct_path(came_from, current_node)
     if came_from[current_node] is set
         p = reconstruct_path(came_from, came_from[current_node])
         return (p + current_node)
     else
         return current_node

*/
// Has some problems; doesn't seem to always find the shortest path
// I think the problem is with the tentativeIsBetter section
// Consider limiting steps to ~750
// data: 170 steps 3ms
// 1937 steps 62ms
list<Location> State::bfs(Location start, Location goal) {
	//typedef priority_queue<Location, vector<Location>, heuristicCompare> mypq_type;
	//mypq_type openSet (heuristicCompare(rows,cols,goal));
	priority_queue<LocationContainer, vector<LocationContainer>, locContainerCompare> openSet;
	map<Location, bool> openSetMap;
	map<Location, bool> closedSet;
	map<Location, Location> cameFrom;
	map<Location, int> g_score;

	bug << "pathfinding from " << start << " to " << goal << endl;
	
	g_score[start] = 0;
	openSet.push(LocationContainer(start, heuristic_cost_estimate(start, goal), 0));
	openSetMap[start] = true;

	int steps = 0;

	while (!openSet.empty()) {
		steps++;
		LocationContainer current = openSet.top();
		
		openSet.pop();
		if (locationEq(current.l, goal) || steps >= searchStepLimit) {
			bug << "steps taken: " << steps << endl;
			return reconstruct_path(cameFrom, current.l);
		}
			
		closedSet[current.l]=true;
		vector<Location> validNeighborsV = validNeighbors(current.l, start);
		
		for(int i = 0;i < (int)validNeighborsV.size();i++) {
			Location neighbor = validNeighborsV[i];
			
			if (closedSet.count(neighbor)>0)
				continue;
				
			int tentative_g_score = current.g_score + locDistance(current.l,neighbor);
			bool tentativeIsBetter = false;
			
			if (! (openSetMap.count(neighbor)>0)) {
				tentativeIsBetter = true;
			} else if (tentative_g_score < g_score[neighbor]) {
				tentativeIsBetter = true;
			} 
			
			if (tentativeIsBetter) {
				cameFrom[neighbor] = current.l;
				g_score[neighbor] = tentative_g_score;
				openSet.push(LocationContainer(neighbor,heuristic_cost_estimate(neighbor, goal)+tentative_g_score, tentative_g_score));
				openSetMap[neighbor] = true;
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

void State::getCloseFoods(vector<Ant*> &ants, list<Location> &food, int maxDistance, bool retainCurrentDestination) {
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

		ants.pop_back();

		food.remove(minFood);

		list<Location> path = bfs((*a).loc, minFood);

		if (! path.empty())
		{
			path.pop_front();
			if (retainCurrentDestination && !(*a).idle()) {
				(*a).rDestination = (*a).destination();
				(*a).intRole = (*a).role;
			}
			(*a).setFood();
			setAntQueue((*a), path, minFood);
		}
	}

	while(!tooFarAnts.empty()) {
		ants.push_back(tooFarAnts.back());
		tooFarAnts.pop_back();
	}
}

void State::killCloseHills(vector<Ant*> &ants, int maxDistance, bool retainCurrentDestination) {
	vector<Ant*> tooFarAnts; 

	while(!ants.empty())
	{
		if (enemyHills.empty())
			break;

		Ant *a = ants.back();

		Location minHill = enemyHills.front();

		for (vector<Location>::iterator hillIt=enemyHills.begin(); hillIt != enemyHills.end(); hillIt++ ){
			if ( locDistance((*a).loc, *hillIt) < locDistance((*a).loc, minHill))
				minHill = *hillIt;
		}

		if (locDistance((*a).loc,minHill) > maxDistance) {
			bug << "ant too far " << locDistance((*a).loc,minHill) << endl;
			tooFarAnts.push_back(a);
			ants.pop_back();
			continue;
		}

		ants.pop_back();

		list<Location> path = bfs((*a).loc, minHill);

		if (! path.empty())
		{
			path.pop_front();
			if (retainCurrentDestination && !(*a).idle()) {
				(*a).rDestination = (*a).destination();
				(*a).intRole = (*a).role;
			}
			(*a).setFood();
			setAntQueue((*a), path, minHill);
		}
	}

	while(!tooFarAnts.empty()) {
		ants.push_back(tooFarAnts.back());
		tooFarAnts.pop_back();
	}
}

void State::getFoods(vector<Ant*> &ants, list<Location> &food, int maxDistance) {
	vector<Ant*> idleAnts; 

	while(!food.empty() && !ants.empty())
	{

		Location currFood = food.front();

		Ant *a = ants.front();

		for(list<Location>::iterator foodIt=food.begin(); foodIt != food.end(); foodIt++)
			for (vector<Ant*>::iterator antIt=ants.begin(); antIt != ants.end(); antIt++ )
				if ((**antIt).idle() && (!(*a).idle() || locDistance((**antIt).loc, *foodIt) < locDistance((*a).loc, currFood))) {
					a = *antIt;
					currFood = *foodIt;
				}

		food.remove(currFood);

		if (locDistance((*a).loc,currFood) > maxDistance) {
			bug << "ant too far " << locDistance((*a).loc,currFood) << endl;
			continue;
		}

		if (!(*a).idle()) {
			bug << "no more idle ants";
			break;
		}

		list<Location> path = bfs((*a).loc, currFood);

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
			setAntQueue((*a), path, currFood);
		}
	}

	for (vector<Ant*>::iterator antIt=ants.begin(); antIt != ants.end(); antIt++ ){
		if ((**antIt).idle())
			idleAnts.push_back(*antIt);
	}

	ants.clear();

	while(!idleAnts.empty()) {
		ants.push_back(idleAnts.back());
		idleAnts.pop_back();
	}
}

void State::killHills(vector<Ant*> &ants, vector<Location> &hills, int antsPerHillPerTurn) {
	int antsSentHere = 0;
	Location currentHill;
	while (!hills.empty()) {
		currentHill = hills.back();
		antsSentHere = 0;
		while((!ants.empty()) && (antsSentHere < antsPerHillPerTurn))
		{
			Ant *a = ants.back();

			ants.pop_back();

			list<Location> path = bfs((*a).loc, currentHill);

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
				setAntQueue((*a), path, currentHill);
				antsSentHere++;
			}
		}
		hills.pop_back();
	}
}

void State::explore(Ant &ant, int mExpDis, int maxExpDis, bool closePoint) {
	//Testing out using an exponential distribution for exploring
	Location exploreDest;

	if (closePoint)
		exploreDest = randomLocationExp(ant.loc, mExpDis, maxExpDis);
	else
		exploreDest = randomLocation();

	if (exploreDest.row == -1 && exploreDest.col == -1)
		return;

	bug << "exploring from " << ant.loc << " to " << exploreDest  << endl;

	list<Location> path = bfs(ant.loc, exploreDest);

	if (! path.empty())
	{
		path.pop_front();
		ant.setExplore();
		setAntQueue(ant, path, exploreDest);
	}
}

void State::goExplore(vector<Ant*> &ants, int mExpDis, int maxExpDis)
{
	//explore with additional ants if we have any
	while(!ants.empty())
	{
		Ant *a = ants.back();

		ants.pop_back();

		explore(*a, mExpDis, maxExpDis, true);
	}
}

void State::goExploreUnexplored(vector<Ant*> &ants, int antsToTake)
{
	if (!hasUnexplored)
		return;
	//explore with additional ants if we have any
	for(int i = 0;i<antsToTake && !ants.empty();i++)
	{
		Ant *a = ants.back();

		ants.pop_back();

		explore(*a, 0, 0, false);
	}
}

void State::rerouteAnt(Ant &ant) {
	list<Location> path = bfs(ant.loc, ant.destination());

	#ifdef DEBUG
		list<Location>::iterator it;
		bug << "reroute path: ";
		for ( it=path.begin() ; it != path.end(); it++ )
			bug << *it << " ";
		bug << endl;
	#endif

	if (path.empty()) {
		ant.setIdle();
	}
	else {
		path.pop_front();
	
		setAntQueue(ant, path, ant.destination());
	}
}

bool State::xAwayFromMyHills(Ant &ant, double buffer) {
	for(int i = 0;i<(int)myHills.size();i++)
		if (distanceSq(ant.loc,myHills[i]) < viewradius2 + buffer)
			return false;
	return true;
}

void State::defendHill(int antsPerTurn, double buffer) {

	vector<int> exclude;
	vector<Location> closeAnts;
	vector<Location> neighbors;
	map<Location, bool> closedSet;

	for(int i = 0;i<(int)myHills.size();i++) {
		bool atRisk = false;

		neighbors.push_back(myHills[i]);

		while(!neighbors.empty())
		{
			Location current = neighbors.back();
			neighbors.pop_back();

			closedSet[current] = true;

			vector<Location> validNeighborsV = validNeighbors(current);

			for(int j = 0;j < (int)validNeighborsV.size();j++) {
				if (closedSet.count(validNeighborsV[j])>0)
					continue;
				if (distanceSq(validNeighborsV[j],myHills[i]) < viewradius2 + buffer) {
					neighbors.push_back(validNeighborsV[j]);
					if (grid[validNeighborsV[j].row][validNeighborsV[j].col].ant > 0) {
						atRisk = true;
						closeAnts.push_back(validNeighborsV[j]);
					}
				}
			}
		}
		neighbors.clear();
		closedSet.clear();

		if (atRisk) {
			hillsAtRisk = true;
			bug << "hill " << myHills[i] << " at risk" << endl;
			for(int antsTaken = 0;antsTaken < antsPerTurn && antsTaken < (int)closeAnts.size();antsTaken++) {

				if (exclude.size() == myAnts.size())
					break;
			
				//TODO fix this not perfect
				int index = 0;
				for(int j = 0;j<(int)myAnts.size();j++) {
					bool cont = false;
					for(int w = 0;w<(int)exclude.size();w++)
						if (exclude[w] == j)
							cont = true;
					if (cont)
						continue;
					if (distanceSq(myAnts[j].loc,myHills[i]) < distanceSq(myAnts[index].loc,myHills[i]) && !myAnts[j].isDefending())
						index = j;
				}

				exclude.push_back(index);

				vector<Location> hLocs = validNeighbors(myHills[i]);

				if (!hLocs.empty())
				{
					Location hLoc = hLocs[rand() % hLocs.size()];

					list<Location> path = bfs(myAnts[index].loc, hLoc);

					bug << "sending ant at " << myAnts[index].loc << endl;

					if (! path.empty())
					{
						path.pop_front();
						if (!myAnts[index].idle()) {
							myAnts[index].rDestination = myAnts[index].destination();
							myAnts[index].intRole = myAnts[index].role;
						}
						myAnts[index].setDefend();
						setAntQueue(myAnts[index], path, hLoc);
					}
				}
			}
		}
		exclude.clear();
	}
}

/*
# we pre-calculate the number of enemies around each ant to make it faster

        # maps ants to nearby enemies
        nearby_enemies = {}
        for ant in self.current_ants.values():
            nearby_enemies[ant] = self.nearby_ants(ant.loc, self.attackradius, ant.owner)

        # determine which ants to kill
        ants_to_kill = []
        for ant in self.current_ants.values():
            # determine this ants weakness (1/power)
            weakness = len(nearby_enemies[ant])
            # an ant with no enemies nearby can't be attacked
            if weakness == 0:
                continue
            # determine the most powerful nearby enemy
            min_enemy_weakness = min(len(nearby_enemies[enemy]) for enemy in nearby_enemies[ant])
            # ant dies if it is weak as or weaker than an enemy weakness
            if min_enemy_weakness <= weakness:
                ants_to_kill.append(ant)

*/

//Not Working
vector<Ant> State::myAntsWhoWillAntDie() {
	map<Location, vector<Ant> > nearby_enemies;

	vector<Ant> antsWhoWillDie;

	for(int i = 0;i<(int)myAnts.size();i++) {
		nearby_enemies[myAnts[i].loc] = nearbyAnts(myAnts[i].loc, myAnts[i].owner);
	}
	for(int i = 0;i<(int)enemyAnts.size();i++) {
		nearby_enemies[enemyAnts[i].loc] = nearbyAnts(enemyAnts[i].loc, enemyAnts[i].owner);
	}

	for(int i = 0;i<(int)myAnts.size();i++) {
		vector<Ant> enemies = nearby_enemies[myAnts[i].loc];
		int weakness = enemies.size();

		if (weakness == 0)
			continue;

		int max_enemy_weakness = 0;

		for(int j = 0;j< (int)enemies.size();j++) {
			int tWeak = nearby_enemies[enemies[j].loc].size();
			if(tWeak > max_enemy_weakness)
				max_enemy_weakness = tWeak;
		}

		if (max_enemy_weakness >= weakness)
			antsWhoWillDie.push_back(myAnts[i]);
	}

	return antsWhoWillDie;
}

// Need will die next turn which will use grid next turn in nearbyAnts next turn
bool State::willAntDie(Location loc) {
	vector<Ant> enemies = nearbyAnts(loc, 0);
	int weakness = enemies.size();

	if (weakness == 0)
		return false;

	int min_enemy_weakness = 0;

	for(int j = 0;j< (int)enemies.size();j++) {
		int tWeak = nearbyAnts(enemies[j].loc, enemies[j].owner).size();
		if(tWeak < min_enemy_weakness)
			min_enemy_weakness = tWeak;
	}

	if (min_enemy_weakness <= weakness)
		return true;

	return false;
}

vector<Ant> State::nearbyAnts(Location loc, int owner) {
	vector<Location> neighbors;
	vector<Ant> close;

	map<Location, bool> closedSet;

	neighbors.push_back(loc);
	while(!neighbors.empty())
	{
		Location current = neighbors.back();
		neighbors.pop_back();

		closedSet[current] = true;

		vector<Location> validNeighborsV = validNeighbors(current);
		
		for(int i = 0;i < (int)validNeighborsV.size();i++) {
			if (closedSet.count(validNeighborsV[i])>0)
				continue;
			if (distanceSq(validNeighborsV[i],loc) < attackradius2+attackDistanceBuffer) {
				neighbors.push_back(validNeighborsV[i]);
				if (grid[validNeighborsV[i].row][validNeighborsV[i].col].ant != -1 && (grid[validNeighborsV[i].row][validNeighborsV[i].col].ant != owner))
					close.push_back(Ant(validNeighborsV[i], grid[validNeighborsV[i].row][validNeighborsV[i].col].ant));
			}
		}
	}

	return close;
}

Location State::nearestEnemy(Ant &ant) {
	vector<Location> neighbors;
	map<Location, bool> closedSet;

	neighbors.push_back(ant.loc);
	while(!neighbors.empty())
	{
		Location current = neighbors.back();
		neighbors.pop_back();

		closedSet[current] = true;

		vector<Location> validNeighborsV = validNeighbors(current);

		for(int i = 0;i < (int)validNeighborsV.size();i++) {
			if (closedSet.count(validNeighborsV[i])>0)
				continue;
			if (distanceSq(validNeighborsV[i],ant.loc) < 2*viewradius2) {
				neighbors.push_back(validNeighborsV[i]);
				if (grid[validNeighborsV[i].row][validNeighborsV[i].col].ant != -1 && (grid[validNeighborsV[i].row][validNeighborsV[i].col].ant != ant.owner))
					return validNeighborsV[i];
			}
		}
	}

	return ant.loc;
}

void State::retreatAntFromNearestEnemy(Ant &ant) {
	Location nearest = nearestEnemy(ant);

	bug << "nearest enemy at: " << nearest << endl;

	// No enemy within sight of this ant
	if (nearest == ant.loc)
		return;

	Location retreat = retreatLocation(ant, nearest);

	if (retreat == ant.loc)
		return;

	ant.retreatCount += 2;
	
	ant.queue.push_front(ant.loc);

	ant.queue.push_front(retreat);
}

//const int DIRECTIONS[4][2] = { {-1, 0}, {0, 1}, {1, 0}, {0, -1} };      //{N, E, S, W}
Location State::retreatLocation(Ant &ant, Location nearest) {
	
	vector<Location> validNeighborsV = validNeighbors(ant.loc, ant.loc);
	
	if (validNeighborsV.empty())
		return ant.loc;

	Location farthest = validNeighborsV.front();

	for (int i = 1;i<(int)validNeighborsV.size();i++)
		if (distanceSq(nearest,validNeighborsV[i]) > distanceSq(nearest,farthest))
			farthest = validNeighborsV[i];

	return farthest;
}


int State::calcExploreDistance(int modifier, int divisor) {
	int t = (cols*rows/(noPlayers*noPlayers))/divisor;

	return t + modifier;
}

int State::getExploreDistance() {
	return exploreDistance;
}

int State::getMinExploreDistance() {
	return minExploreDistance;
}

void State::setExploreDistance(int modifier, int divisor) {
	exploreDistance = calcExploreDistance(modifier, divisor);
	bug << "set explore distance at " << exploreDistance << endl;
}

void State::setMinExploreDistance(int modifier, int divisor) {
	minExploreDistance = calcExploreDistance(modifier, divisor);
	bug << "set min explore distance at " << minExploreDistance << endl;
}