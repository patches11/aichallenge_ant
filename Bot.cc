#include "Bot.h"

using namespace std;

// Add max distance to kill hill?
// Use a non - uniform distribution for explore?

#define exploreDistanceModifier 5
#define minExploreDistanceModifier 2
#define maxFoodDistance 30
#define maxExploreFoodDistance 5
#define maxExploreKillHillDistance 3
#define maxAntsToKillHillPerTurn 10
#define timeWindowMs 100
#define defendAntsPerTurn 2
#define hillBuffer 2
#define defendTurns 1
#define minAntsFoodingToKillPercent 0.30
#define exploreDistanceDivisor 200
#define minExploreDistanceDivisor 500

//constructor
Bot::Bot()
{

};


//plays a single game of Ants.
void Bot::playGame(int argc, char *argv[])
{
    //reads the game parameters and sets up
    cin >> state;
    state.setup();
	state.setExploreDistance(exploreDistanceModifier, exploreDistanceDivisor);
	state.setMinExploreDistance(minExploreDistanceModifier, minExploreDistanceDivisor);
    endTurn();

    //continues making moves while the game is not over
    while(cin >> state)
    {
        state.updateVisionInformation();
        makeMoves();
        endTurn();
    }
};

//makes the bots moves for the turn
void Bot::makeMoves()
{
    state.bug << "turn " << state.turn << ":" << endl;
	state.bug << "ants " << (int)state.myAnts.size() << endl;
    state.bug << state << endl;

	state.bug << "defending hills with close ants" << endl;
	double time1 = state.timer.getTime();
	state.defendHill(defendAntsPerTurn, hillBuffer);
	state.bug << "defendHill time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;

	vector<Location> destinations;
	vector<Ant*> idleAnts;
	vector<Ant*> exploringOrFoodingAnts;
	list<Location> idleFoods;
	
	int killCount, defendCount, foodCount, exploreCount;
	killCount = defendCount = foodCount = exploreCount = 0;

	for (int i = 0;i<(int)state.myAnts.size();i++) {
		if (state.myAnts[i].isDefending() && !state.hillsAtRisk) {
			state.myAnts[i].setIdle();
		}
		if (!state.myAnts[i].idle()) {
			Location destination = state.myAnts[i].destination();
			if (state.myAnts[i].isAttacking() && state.grid[destination.row][destination.col].isVisible && !state.grid[destination.row][destination.col].isHill) {
				state.myAnts[i].setIdle();
				idleAnts.push_back(&state.myAnts[i]);
			}else if (state.myAnts[i].isGettingFood() && state.grid[destination.row][destination.col].isVisible && !state.grid[destination.row][destination.col].isFood) {
				state.myAnts[i].setIdle();
				idleAnts.push_back(&state.myAnts[i]);
			} else if (state.passable(destination)) {
				if (state.myAnts[i].isAttacking())
					killCount++;
				else if (state.myAnts[i].isDefending())
					defendCount++;
				else if (state.myAnts[i].isGettingFood())
					foodCount++;
				else if (state.myAnts[i].isExploring())
					exploreCount++;
				destinations.push_back(destination);
			}
			else {
				state.myAnts[i].setIdle();
				idleAnts.push_back(&state.myAnts[i]);
			}
			if (!state.myAnts[i].wasInterrupted() && (state.myAnts[i].isExploring() || state.myAnts[i].isGettingFood()))
				exploringOrFoodingAnts.push_back(&state.myAnts[i]);
		} else {
			if (state.myAnts[i].isDefending() && state.myAnts[i].defendCounter < defendTurns) {
				state.myAnts[i].defendCounter++;
			} else if (state.myAnts[i].wasInterrupted()) {
				list<Location> path = state.bfs(state.myAnts[i].loc, state.myAnts[i].rDestination);

				state.myAnts[i].role = state.myAnts[i].intRole;
				state.myAnts[i].intRole = -1;

				if (path.empty()) {
					state.myAnts[i].setIdle();
					idleAnts.push_back(&state.myAnts[i]);
				}
				else {
					path.pop_front();
	
					state.setAntQueue(state.myAnts[i], path, state.myAnts[i].rDestination);
				}
			}
			else
				idleAnts.push_back(&state.myAnts[i]);
		}
	}

	for(int i = 0;i<(int)state.food.size();i++){
		if (!state.checkDestinations(destinations, state.food[i]))
			idleFoods.push_back(state.food[i]);
	}

	state.bug << "killing close hills with exploring or fooding ants" << endl;
	time1 = state.timer.getTime();
	state.killCloseHills(exploringOrFoodingAnts, maxExploreKillHillDistance, true);
	state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;

	state.bug << "getting close food with exploring or fooding ants" << endl;
	time1 = state.timer.getTime();
	state.getCloseFoods(exploringOrFoodingAnts, idleFoods, maxExploreFoodDistance, true);
	state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;

	if (state.outOfTime(timeWindowMs))
		return;

	// Only kill hills if more than 30% of our ants are getting food or exploring
	if ((foodCount+exploreCount)/((double)state.myAnts.size()) > minAntsFoodingToKillPercent ) {
		state.bug << "killing hills with idle ants" << endl;
		time1 = state.timer.getTime();
		state.killHills(idleAnts, state.enemyHills, maxAntsToKillHillPerTurn);
		state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;
	}

	if (state.outOfTime(timeWindowMs))
		return;

	state.bug << "getting food with idle ants" << endl;
	time1 = state.timer.getTime();
    state.getFoods(idleAnts, idleFoods, maxFoodDistance);
	state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;

	if (state.outOfTime(timeWindowMs))
		return;

	state.bug << "exploring with remaining ants" << endl;
	time1 = state.timer.getTime();
	state.goExplore(idleAnts, state.getMinExploreDistance(), state.getExploreDistance());
	state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;

	if (state.outOfTime(timeWindowMs))
		return;

	//If ants are going to collide next turn then re-route to destination
	state.bug << "re routing ants" << endl;
	time1 = state.timer.getTime();
	//TODO - retreat if ant will lose a battle
	//Todo - We still have collisions, fix this code
	for (int i = 0;i<(int)state.myAnts.size();i++) {
		if (state.outOfTime(timeWindowMs))
			return;
		if (!state.myAnts[i].idle() && state.myAnts[i].pathfindingIncomplete() && state.myAnts[i].queue.size() == 1) {
			// Ant was interrupted and hasn't reached its destination and we are on the last move of this interruption
			state.rerouteAnt(state.myAnts[i]);
		}
		if (state.myAnts[i].idle() && state.isOnMyHill(state.myAnts[i].loc)) {
			state.explore(state.myAnts[i], 4, 6);
		}
		if (!state.myAnts[i].idle() && !state.passable(state.myAnts[i].positionNextTurn())) {
			state.rerouteAnt(state.myAnts[i]);
		}
		for (int j = i+1;j<(int)state.myAnts.size();j++)
			if (state.myAnts[i].positionNextTurn() ==  state.myAnts[j].positionNextTurn()) {
				if (state.myAnts[j].idle()) {
					state.bug << "reseting route because of collision, ant at: " << state.myAnts[i].loc << " other ant at " << state.myAnts[j].loc << endl;
					state.bug << "locations next turn: " << state.myAnts[i].positionNextTurn() << " other ant at " << state.myAnts[j].positionNextTurn() << endl;
					
					state.rerouteAnt(state.myAnts[i]);

					break;
				} else {
					state.bug << "reseting route because of collision, ant at: " << state.myAnts[j].loc << " other ant at " << state.myAnts[i].loc << endl;
					state.bug << "locations next turn: " << state.myAnts[j].positionNextTurn() << " other ant at " << state.myAnts[i].positionNextTurn() << endl;
					state.rerouteAnt(state.myAnts[j]);
				}
			}
	}
	state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;

    state.bug << "turn time taken: " << state.timer.getTime() << "ms" << endl << endl;
};

//finishes the turn
void Bot::endTurn()
{
    if(state.turn > 0) {
		//Move ants with a move queue
        state.makeMoves();
		state.reset();
	}
    state.turn++;

    cout << "go" << endl;
};
