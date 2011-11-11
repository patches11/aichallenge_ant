#ifndef BOT_H_
#define BOT_H_

#include "State.h"

/*
    This struct represents your bot in the game of Ants
*/
struct Bot
{
    State state;

    Bot();

    void playGame(int argc, char *argv[]);    //plays a single game of Ants

    void makeMoves();   //makes moves for a single turn
    void endTurn();     //indicates to the engine that it has made its moves

	int exploreDistanceModifier, minExploreDistanceModifier , maxFoodDistance, maxExploreFoodDistance, maxExploreKillHillDistance, 
		maxAntsToKillHillPerTurn, timeWindowMs, defendAntsPerTurn, hillBuffer, defendTurns, exploreDistanceDivisor, 
		minExploreDistanceDivisor, maxTurnsToRetreat, attackDistanceBuffer, searchStepLimit, minExploreDistanceFromHill, 
		turnsTillNotAtRisk, maxDefendingAnts, minAntsToGoUnexplored, antsToGoToUnexplored;
		
	double minAntsFoodingToKillPercent, expLamda;

	bool useDefendCounter, exploreUnexplored, idleAntsForExcessiveRetreating, useRetreatForKillingAnts, useSquareOfPlayers, 
			useExponentialExploring, noMaxTurnsToRetreat;
};

#endif //BOT_H_
