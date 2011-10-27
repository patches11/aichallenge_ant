#include "Bot.h"

using namespace std;

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

    //picks out moves for each ant
    for(int food=0; food<(int)state.food.size(); food++)
    {
    	if (antIndex >= (int)state.myAnts.size())
    		break;
    			
        vector<Location> path = state.bfs(state.myAnts[antIndex], state.food[food]);

		if (! path.empty())
		{

			#ifdef DEBUG
			state.bug << "path: ";
				for(int i = 0;i < (int)path.size();i++)
					state.bug << path[i] << " ";
				state.bug << endl;
			#endif

			state.makeMove(state.myAnts[antIndex], state.directionFromPoints(state.myAnts[antIndex],path[1]));
		}
        
        antIndex++;
    }

    state.bug << "time taken: " << state.timer.getTime() << "ms" << endl << endl;
};

//finishes the turn
void Bot::endTurn()
{
    if(state.turn > 0)
        state.reset();
    state.turn++;

    cout << "go" << endl;
};
