#include "Bot.h"

using namespace std;

#define exploreDistance 15
#define minExploreDistance 3

//constructor
Bot::Bot()
{

};

//plays a single game of Ants.
void Bot::playGame()
{
    //reads the game parameters and sets up
    cin >> state;
    state.setup();
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
    state.bug << state << endl;

	vector<Location> destinations;

	for (int i = 0;i<(int)state.myAnts.size();i++) {
		if (!state.myAnts[i].idle())
			destinations.push_back(state.myAnts[i].destination());
	}

	vector<Ant*> idleAnts;

	for(int i = 0;i<(int)state.myAnts.size();i++){
		if (state.myAnts[i].idle())
			idleAnts.push_back(&state.myAnts[i]);
	}

	vector<Ant*> exploringAnts;

	for(int i = 0;i<(int)state.myAnts.size();i++){
		if (state.myAnts[i].exploring())
			exploringAnts.push_back(&state.myAnts[i]);
	}

	list<Location> idleFoods;

	for(int i = 0;i<(int)state.food.size();i++){
		if (!state.checkDestinations(destinations, state.food[i]))
			idleFoods.push_back(state.food[i]);
	}

	state.bug << "getting close food with exploring ants" << endl;
	state.getFoods(exploringAnts, idleFoods, 4);

	state.bug << "getting food with idle ants" << endl;
    state.getFoods(idleAnts, idleFoods, 30);

	state.bug << "killing hills with idle ants" << endl;
	state.killHills(idleAnts, state.enemyHills, 0);

	state.bug << "exploring with remaining ants" << endl;
	state.goExplore(idleAnts, minExploreDistance, exploreDistance);

	//If ants are going to collide next turn then re-route to destination
	for (int i = 0;i<(int)state.myAnts.size();i++) {
		if (state.myAnts[i].idle() && state.isOnMyHill(state.myAnts[i].loc)) {
			state.explore(state.myAnts[i], 4, 6);
		}
		for (int j = i+1;j<(int)state.myAnts.size();j++)
			if (state.myAnts[i].positionNextTurn() ==  state.myAnts[j].positionNextTurn()) {
				if (state.myAnts[j].idle()) {
					state.bug << "reseting route because of collision, ant at: " << state.myAnts[i].loc << " other ant at " << state.myAnts[j].loc << endl;
					state.bug << "locations next turn: " << state.myAnts[i].positionNextTurn() << " other ant at " << state.myAnts[j].positionNextTurn() << endl;
					
					list<Location> path = state.bfs(state.myAnts[i].loc, state.myAnts[i].destination());

					path.pop_front();

					state.setAntQueue(state.myAnts[i], path);

					break;
				} else {
					state.bug << "reseting route because of collision, ant at: " << state.myAnts[j].loc << " other ant at " << state.myAnts[i].loc << endl;
					state.bug << "locations next turn: " << state.myAnts[j].positionNextTurn() << " other ant at " << state.myAnts[i].positionNextTurn() << endl;
					list<Location> path = state.bfs(state.myAnts[j].loc, state.myAnts[j].destination());

					path.pop_front();

					state.setAntQueue(state.myAnts[j], path);
				}
			}
	}

    state.bug << "time taken: " << state.timer.getTime() << "ms" << endl << endl;
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
