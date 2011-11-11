#include "Bot.h"

using namespace std;

// Add max distance to kill hill?
// Should try to route around areas where we will die: flank
// Remember enemy hills we have seen to kill later
// We may want to do something so we don't kill too many ants early in the game trying to take out a hill
// When my hill dies may still have anys defending it and they will stick there.
// When killing hill we should have some sort of flag like stop retreating, I don't think anything is gained once the ants have massed by waiting vs. Just going at that point.

//constructor
Bot::Bot()
{
	// Defaults

	//int
	exploreDistanceModifier = 3;
	minExploreDistanceModifier = 1;
	maxFoodDistance = 30;
	maxExploreFoodDistance = 5;
	maxExploreKillHillDistance = 3;
	maxAntsToKillHillPerTurn = 10;
	timeWindowMs = 100;
	defendAntsPerTurn = 2;
	hillBuffer = 2;
	defendTurns = 4;
	exploreDistanceDivisor = 2000;
	minExploreDistanceDivisor = 5000;
	maxTurnsToRetreat = 10;
	attackDistanceBuffer = 8;
	searchStepLimit = 550;
	minExploreDistanceFromHill = 2;
	turnsTillNotAtRisk = 3;
	maxDefendingAnts = 10;
	minAntsToGoUnexplored = 100;
	antsToGoToUnexplored = 3;

	//double
	minAntsFoodingToKillPercent = 0.30;
	expLamda = 0.55;

	//bool
	useDefendCounter = false;
	exploreUnexplored = true;
	idleAntsForExcessiveRetreating = true;
	useRetreatForKillingAnts = true;
	useSquareOfPlayers = true;
	useExponentialExploring = true;
	noMaxTurnsToRetreat = true;
};


//plays a single game of Ants.
void Bot::playGame(int argc, char *argv[])
{
    //reads the game parameters and sets up
    cin >> state;
    state.setup();

	//read command line arguments
	for(int i = 0; i < argc; i++) {
		if (argv[i] == "--attackDistanceBuffer")
			attackDistanceBuffer = atoi(argv[++i]);
		else if (argv[i] == "--exploreDistanceModifier")
			exploreDistanceModifier = atoi(argv[++i]);
		else if (argv[i] == "--minExploreDistanceModifier")
			minExploreDistanceModifier = atoi(argv[++i]);
		else if (argv[i] == "--maxFoodDistance")
			maxFoodDistance = atoi(argv[++i]);
		else if (argv[i] == "--maxExploreFoodDistance")
			maxExploreFoodDistance = atoi(argv[++i]);
		else if (argv[i] == "--maxExploreKillHillDistance")
			maxExploreKillHillDistance = atoi(argv[++i]);
		else if (argv[i] == "--maxAntsToKillHillPerTurn")
			maxAntsToKillHillPerTurn = atoi(argv[++i]);
		else if (argv[i] == "--timeWindowMs")
			timeWindowMs = atoi(argv[++i]);
		else if (argv[i] == "--defendAntsPerTurn")
			defendAntsPerTurn = atoi(argv[++i]);
		else if (argv[i] == "--hillBuffer")
			hillBuffer = atoi(argv[++i]);
		else if (argv[i] == "--defendTurns")
			defendTurns = atoi(argv[++i]);
		else if (argv[i] == "--exploreDistanceDivisor")
			exploreDistanceDivisor = atoi(argv[++i]);
		else if (argv[i] == "--minExploreDistanceDivisor")
			minExploreDistanceDivisor = atoi(argv[++i]);
		else if (argv[i] == "--maxTurnsToRetreat")
			maxTurnsToRetreat = atoi(argv[++i]);
		else if (argv[i] == "--attackDistanceBuffer")
			attackDistanceBuffer = atoi(argv[++i]);
		else if (argv[i] == "--searchStepLimit")
			searchStepLimit = atoi(argv[++i]);
		else if (argv[i] == "--minExploreDistanceFromHill")
			minExploreDistanceFromHill = atoi(argv[++i]);
		else if (argv[i] == "--turnsTillNotAtRisk")
			turnsTillNotAtRisk = atoi(argv[++i]);
		else if (argv[i] == "--maxDefendingAnts")
			maxDefendingAnts = atoi(argv[++i]);
		else if (argv[i] == "--minAntsToGoUnexplored")
			minAntsToGoUnexplored = atoi(argv[++i]);
		else if (argv[i] == "--antsToGoToUnexplored")
			antsToGoToUnexplored = atoi(argv[++i]);
		else if (argv[i] == "--minAntsFoodingToKillPercent")
			minAntsFoodingToKillPercent = atof(argv[++i]);
		else if (argv[i] == "--expLamda")
			expLamda = atof(argv[++i]);
		else if (argv[i] == "--useDefendCounter")
			useDefendCounter =argv[++i] == "true" ? true : false;
		else if (argv[i] == "--exploreUnexplored")
			exploreUnexplored =argv[++i] == "true" ? true : false;
		else if (argv[i] == "--idleAntsForExcessiveRetreating")
			idleAntsForExcessiveRetreating =argv[++i] == "true" ? true : false;
		else if (argv[i] == "--useRetreatForKillingAnts")
			useRetreatForKillingAnts =argv[++i] == "true" ? true : false;
		else if (argv[i] == "--useExponentialExploring")
			useExponentialExploring =argv[++i] == "true" ? true : false;
	}
    
	//State configuration
	state.setExploreDistance(exploreDistanceModifier, exploreDistanceDivisor);
	state.setMinExploreDistance(minExploreDistanceModifier, minExploreDistanceDivisor);
	state.setAttackDistanceBuffer(attackDistanceBuffer);
	state.setSearchStepLimit(searchStepLimit);
	state.setMinExploreDistanceFromHill(minExploreDistanceFromHill);
	state.setExpLamda(expLamda);
	state.setTurnsTillNotAtRisk(turnsTillNotAtRisk);
	state.setMaxDefendingAnts(maxDefendingAnts);
	state.setUseExponentialExploring(useExponentialExploring);

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

//Have some timeouts and crashes...
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
		//Slow?
		if (state.myAnts[i].isDefending() && !(state.hillsAtRisk[state.myAnts[i].hillDefending] > 0)) {
			state.setAntIdle(state.myAnts[i]);
		}
		if (!state.myAnts[i].idle()) {
			Location destination = state.myAnts[i].destination();
			if (state.myAnts[i].isAttacking() && state.grid[destination.row][destination.col].isVisible && !state.grid[destination.row][destination.col].isHill) {
				state.setAntIdle(state.myAnts[i]);
				idleAnts.push_back(&state.myAnts[i]);
			}else if (state.myAnts[i].isGettingFood() && state.grid[destination.row][destination.col].isVisible && !state.grid[destination.row][destination.col].isFood) {
				state.setAntIdle(state.myAnts[i]);
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
				state.setAntIdle(state.myAnts[i]);
				idleAnts.push_back(&state.myAnts[i]);
			}
			if (!state.myAnts[i].wasInterrupted() && (state.myAnts[i].isExploring() || state.myAnts[i].isGettingFood()))
				exploringOrFoodingAnts.push_back(&state.myAnts[i]);
		} else {
			if (state.myAnts[i].isDefending() && state.myAnts[i].defendCounter < defendTurns) {
				if (useDefendCounter)
					state.myAnts[i].defendCounter++;
			} else 
			if (state.myAnts[i].wasInterrupted()) {
				list<Location> path = state.bfs(state.myAnts[i].loc, state.myAnts[i].rDestination);

				state.myAnts[i].role = state.myAnts[i].intRole;
				state.myAnts[i].intRole = -1;

				if (path.empty()) {
					state.setAntIdle(state.myAnts[i]);
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

	// Only kill hills if more than minAntsFoodingToKillPercent% of our ants are getting food or exploring
	if ((foodCount+exploreCount)/((double)state.myAnts.size()) > minAntsFoodingToKillPercent ) {
		state.bug << "killing hills with idle ants" << endl;
		time1 = state.timer.getTime();
		state.killHills(idleAnts, state.enemyHills, maxAntsToKillHillPerTurn);
		state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;
	}

	if (state.outOfTime(timeWindowMs))
		return;

	if (exploreUnexplored && (int)state.myAnts.size() > minAntsToGoUnexplored) {
		state.bug << "exploring unexplored areas" << endl;
		time1 = state.timer.getTime();
		state.goExploreUnexplored(idleAnts, antsToGoToUnexplored);
		state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;
	}

	state.bug << "getting food with idle ants" << endl;
	time1 = state.timer.getTime();
    state.getFoods(idleAnts, idleFoods, maxFoodDistance);
	state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;

	if (state.outOfTime(timeWindowMs))
		return;

	state.bug << "exploring with remaining ants" << endl;
	time1 = state.timer.getTime();
	state.goExplore(idleAnts, state.getMinExploreDistance(),state.getExploreDistance());
	state.bug << "time taken: " << state.timer.getTime() - time1 << "ms" << endl << endl;

	if (state.outOfTime(timeWindowMs))
		return;

	//If ants are going to collide next turn then re-route to destination
	state.bug << "re routing ants" << endl;
	time1 = state.timer.getTime();
	//TODO - retreat if ant will lose a battle
	//Todo - We still have collisions, fix this code
	// Collisions especially when retreating & when defending
	for (int i = 0;i<(int)state.myAnts.size();i++) {
		if (state.outOfTime(timeWindowMs))
			return;
		if (!state.myAnts[i].idle() && state.myAnts[i].pathfindingIncomplete() && state.myAnts[i].queue.size() == 1) {
			// Ant was interrupted and hasn't reached its destination and we are on the last move of this interruption
			state.rerouteAnt(state.myAnts[i]);
		}
		if (state.myAnts[i].idle() && state.isOnMyHill(state.myAnts[i].loc)) {
			state.explore(state.myAnts[i], 4, 6, true);
		}
		if (!state.myAnts[i].idle() && !state.passable(state.myAnts[i].positionNextTurn())) {
			state.rerouteAnt(state.myAnts[i]);
		}
		if (state.willAntDie(state.myAnts[i].positionNextTurn()) && (state.myAnts[i].isExploring() || state.myAnts[i].isGettingFood() || state.myAnts[i].idle() || (useRetreatForKillingAnts && state.myAnts[i].isAttacking())) && state.xAwayFromMyHills(state.myAnts[i],hillBuffer)) {
			if (noMaxTurnsToRetreat || state.myAnts[i].turnsRetreating < maxTurnsToRetreat)
				state.retreatAntFromNearestEnemy(state.myAnts[i]);
			else if (idleAntsForExcessiveRetreating) {
				// I think this is not working, ants seem to die
				state.setAntIdle(state.myAnts[i]);
				state.retreatAntFromNearestEnemy(state.myAnts[i]);
			}
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
