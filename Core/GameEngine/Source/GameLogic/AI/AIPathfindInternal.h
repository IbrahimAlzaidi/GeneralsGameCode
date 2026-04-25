#pragma once

#include "GameLogic/AIPathfind.h"

struct TCheckMovementInfo
{
	// Input
	ICoord2D					cell;
	PathfindLayerEnum layer;
	Int								radius;
	Bool							centerInCell;
	Bool							considerTransient;
	LocomotorSurfaceTypeMask acceptableSurfaces;
	// Output
	Int								allyFixedCount;
	Bool							enemyFixed;
	Bool							allyMoving;
	Bool							allyGoal;
};

constexpr const Int COST_ORTHOGONAL = 10;
constexpr const Int COST_DIAGONAL = 14;
constexpr const Real COST_TO_DISTANCE_FACTOR = 1.0f / 10.0f;
constexpr const Real COST_TO_DISTANCE_FACTOR_SQR = COST_TO_DISTANCE_FACTOR * COST_TO_DISTANCE_FACTOR;

inline Int IABS(Int x) { return x >= 0 ? x : -x; }
