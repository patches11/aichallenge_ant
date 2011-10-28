#include "Bot.h"

using namespace std;

#define exploreDistance 5

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
    
    int antIndex = 0;

    //get foood
    for(int food=0; food<(int)state.food.size(); food++)
    {
    	if (antIndex >= (int)state.myAnts.size() || !state.myAnts[antIndex].idle())
    		break;
    			
		list<Location> path = state.bfs(state.myAnts[antIndex].loc, state.food[food]);
		
		if (! path.empty())
		{

			#ifdef DEBUG
				list<Location>::iterator it;
				state.bug << "food path: ";
				for ( it=path.begin() ; it != path.end(); it++ )
					state.bug << *it << " ";
				state.bug << endl;
			#endif

			path.pop_front();

			state.myAnts[antIndex].queue = path;
		}
        
        antIndex++;
    }

	//explore with additional ants if we have any
	for(;antIndex<(int)state.myAnts.size();antIndex++) {
		if (!state.myAnts[antIndex].idle())
    		break;

		Location antLoc = state.myAnts[antIndex].loc;

		Location exploreDest = state.randomLocation(state.myAnts[antIndex].loc,exploreDistance);

		state.bug << "exploring from " << antLoc << " to " << exploreDest  << endl;

		list<Location> path = state.bfs(antLoc, exploreDest);

		if (! path.empty())
		{

			#ifdef DEBUG
				list<Location>::iterator it;
				state.bug << "explore path: ";
				for ( it=path.begin() ; it != path.end(); it++ )
					state.bug << *it << " ";
				state.bug << endl;
			#endif

			path.pop_front();

			state.myAnts[antIndex].queue = path;
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
