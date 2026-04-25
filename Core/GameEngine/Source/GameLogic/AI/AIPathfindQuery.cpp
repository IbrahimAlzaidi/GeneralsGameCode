#include "PreRTS.h"

#include "GameLogic/AIPathfind.h"
#include "AIPathfindInternal.h"

#include "Common/CRCDebug.h"
#include "Common/GlobalData.h"
#include "Common/LatchRestore.h"
#include "Common/Player.h"
#include "Common/PerfTrace.h"
#include "Common/ThingTemplate.h"
#include "Common/RandomValue.h"

#include "GameClient/Line2D.h"

#include "GameLogic/AI.h"
#include "GameLogic/GameLogic.h"
#include "GameLogic/Object.h"
#include "GameLogic/PartitionManager.h"
#include "GameLogic/TerrainLogic.h"
#include "GameLogic/Weapon.h"
#include "GameLogic/Module/AIUpdate.h"

#include "Common/PerfMetrics.h"

extern Bool s_useFixedPathfinding;
extern Bool s_forceCleanCells;

static inline UnsignedInt getPerfTraceFrame()
{
	return TheGameLogic ? TheGameLogic->getFrame() : 0;
}

constexpr const UnsignedInt MAX_CELL_COUNT = 500;
constexpr const UnsignedInt MAX_SAFE_PATH_CELL_COUNT = 2000;



/**
 * Does any broken bridge join from and to?
 * True means that if bridge BridgeID is repaired, there is a land path from to to..
 */
Bool Pathfinder::findBrokenBridge(const LocomotorSet& locoSet,
																	const Coord3D *from, const Coord3D *to, ObjectID *bridgeID)
{
	// See if terrain or building is blocking the destination.
	PathfindLayerEnum destinationLayer = TheTerrainLogic->getLayerForDestination(to);
	PathfindLayerEnum fromLayer = TheTerrainLogic->getLayerForDestination(from);

	Int zone1, zone2;
	*bridgeID = INVALID_ID;

	PathfindCell *parentCell = getClippedCell(fromLayer, from);
	PathfindCell *goalCell = getClippedCell(destinationLayer, to);
	zone1 = m_zoneManager.getEffectiveZone(locoSet.getValidSurfaces(), false, parentCell->getZone());
	zone2 =  m_zoneManager.getEffectiveZone(locoSet.getValidSurfaces(), false, goalCell->getZone());
	zone1 = m_zoneManager.getEffectiveTerrainZone(zone1);
	zone2 = m_zoneManager.getEffectiveTerrainZone(zone2);
	zone1 = m_zoneManager.getEffectiveZone(locoSet.getValidSurfaces(), false, zone1);
	zone2 =  m_zoneManager.getEffectiveZone(locoSet.getValidSurfaces(), false, zone2);

	// If the terrain is connected using this locomotor set, we can path somehow.
	if (zone1 == zone2) {
		// There is not terrain blocking the from & to.
		return false;
	}

	// Check broken bridges.
	Int i;
	for (i=0; i<=LAYER_LAST; i++) {
		if (m_layers[i].isDestroyed()) {
			if (m_layers[i].connectsZones(&m_zoneManager, locoSet, zone1, zone2)) {
				*bridgeID = m_layers[i].getBridgeID();
				return true;
			}
		}
	}
	return false;
}

/**
 * Does any path exist from 'from' to 'to' given the locomotor set
 * This is the quick check, only looks at whether the terrain is possible or
 * impossible to path over.  Doesn't take other units into account.
 * False means it is impossible to path.
 * True means it is possible given the terrain, but there may be units in the way.
 */
Bool Pathfinder::clientSafeQuickDoesPathExist( const LocomotorSet& locomotorSet,
																const Coord3D *from,
																const Coord3D *to )
{
	// See if terrain or building is blocking the destination.
	PathfindLayerEnum destinationLayer = TheTerrainLogic->getLayerForDestination(to);
	if (!validMovementPosition(false, destinationLayer, locomotorSet, to)) {
		return false;
	}
	PathfindLayerEnum fromLayer = TheTerrainLogic->getLayerForDestination(from);
	Int zone1, zone2;

	PathfindCell *parentCell = getClippedCell(fromLayer, from);
	PathfindCell *goalCell = getClippedCell(destinationLayer, to);
	if (goalCell->getType()==PathfindCell::CELL_CLIFF) {
		return false; // No goals on cliffs.
	}
	Bool doingTerrainZone = false;
	zone1 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), false, parentCell->getZone());

	if (parentCell->getType() == PathfindCell::CELL_OBSTACLE) {
		doingTerrainZone = true;
#if !(RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING)
		if (zone1 == PathfindZoneManager::UNINITIALIZED_ZONE) {
			// We are in a building that just got placed, and zones haven't been updated yet. [8/8/2003]
			// It is better to return a false positive than a false negative. jba.
			return true;
		}
#endif
	}
	zone2 =  m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), false, goalCell->getZone());
	if (goalCell->getType() == PathfindCell::CELL_OBSTACLE) {
		doingTerrainZone = true;
	}
	if (doingTerrainZone) {
		zone1 = parentCell->getZone();
		zone1 = m_zoneManager.getEffectiveTerrainZone(zone1);
		zone1 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), false, zone1);
		zone1 = m_zoneManager.getEffectiveTerrainZone(zone1);
		zone2 = goalCell->getZone();
		zone2 = m_zoneManager.getEffectiveTerrainZone(zone2);
		zone2 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), false, zone2);
		zone2 = m_zoneManager.getEffectiveTerrainZone(zone2);
	}
	// If the terrain is connected using this locomotor set, we can path somehow.
	if (zone1 == zone2) {
		// There is not terrain blocking the from & to.
		return true;
	}
	return FALSE;  // no path exists

}

/**
 * Does any path exist from 'from' to 'to' given the locomotor set
 * This is the quick check, only looks at whether the terrain is possible or
 * impossible to path over.  Doesn't take other units into account.
 * False means it is impossible to path.
 * True means it is possible given the terrain, but there may be units in the way.
 */
Bool Pathfinder::clientSafeQuickDoesPathExistForUI( const LocomotorSet& locomotorSet,
																const Coord3D *from,
																const Coord3D *to )
{
	// See if terrain or building is blocking the destination.
	PathfindLayerEnum destinationLayer = TheTerrainLogic->getLayerForDestination(to);
	PathfindLayerEnum fromLayer = TheTerrainLogic->getLayerForDestination(from);
	Int zone1, zone2;

	PathfindCell *parentCell = getClippedCell(fromLayer, from);
	PathfindCell *goalCell = getClippedCell(destinationLayer, to);
	if (goalCell->getType()==PathfindCell::CELL_CLIFF) {
		return false; // No goals on cliffs.
	}

	zone1 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), false, parentCell->getZone());
	zone2 =  m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), false, goalCell->getZone());

	if (zone1 == PathfindZoneManager::UNINITIALIZED_ZONE ||
			zone2 == PathfindZoneManager::UNINITIALIZED_ZONE) {
		// We are in a building that just got placed, and zones haven't been updated yet. [8/8/2003]
		// It is better to return a false positive than a false negative. jba.
		return true;
	}
	/* Do the effective terrain zone.  This feedback is for the ui, so we won't take structures into account,
		because if they are visible it will be obvious, and if they are stealthed they should be invisible to the
		pathing as well. jba. */
	zone1 = parentCell->getZone();
	zone1 = m_zoneManager.getEffectiveTerrainZone(zone1);
	zone1 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), false, zone1);
	zone1 = m_zoneManager.getEffectiveTerrainZone(zone1);
	zone2 = goalCell->getZone();
	zone2 = m_zoneManager.getEffectiveTerrainZone(zone2);
	zone2 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), false, zone2);
	zone2 = m_zoneManager.getEffectiveTerrainZone(zone2);

	if (zone1 == PathfindZoneManager::UNINITIALIZED_ZONE) {
		// We are in a building that just got placed, and zones haven't been updated yet. [8/8/2003]
		// It is better to return a false positive than a false negative. jba.
		return true;
	}
	// If the terrain is connected using this locomotor set, we can path somehow.
	if (zone1 == zone2) {
		// There is not terrain blocking the from & to.
		return true;
	}
	return FALSE;  // no path exists

}

/**
 * Does any path exist from 'from' to 'to' given the locomotor set
 * This is the careful check, looks at whether the terrain, buindings and units are possible or
 * impossible to path over.  Takes other units into account.
 * False means it is impossible to path.
 * True means it is possible to path.
 */
Bool Pathfinder::slowDoesPathExist( Object *obj,
																const Coord3D *from,
																const Coord3D *to,
																ObjectID ignoreObject)
{
	AIUpdateInterface *ai = obj->getAI();
	if (ai==nullptr) {
		return false;
	}
	const LocomotorSet &locoSet = ai->getLocomotorSet();
	m_ignoreObstacleID = ignoreObject;
	Path *path = findPath(obj, locoSet, from, to);
	m_ignoreObstacleID = INVALID_ID;
	Bool found = (path!=nullptr);

	deleteInstance(path);
	path = nullptr;

	return found;
}

void Pathfinder::clip( Coord3D *from, Coord3D *to )
{
	ICoord2D fromCell, toCell;
	ICoord2D clipFromCell, clipToCell;
	fromCell.x = REAL_TO_INT_FLOOR(from->x/PATHFIND_CELL_SIZE);
	fromCell.y = REAL_TO_INT_FLOOR(from->y/PATHFIND_CELL_SIZE);
	toCell.x = REAL_TO_INT_FLOOR(to->x/PATHFIND_CELL_SIZE);
	toCell.y = REAL_TO_INT_FLOOR(to->y/PATHFIND_CELL_SIZE);
	if (ClipLine2D(&fromCell, &toCell, &clipFromCell, &clipToCell,&m_extent)) {
		if (fromCell.x!=clipFromCell.x || fromCell.y != clipFromCell.y) {
			from->x = clipFromCell.x*PATHFIND_CELL_SIZE_F + 0.05f;
			from->y = clipFromCell.y*PATHFIND_CELL_SIZE_F + 0.05f;
		}
		if (toCell.x!=clipToCell.x || toCell.y != clipToCell.y) {
			to->x = clipToCell.x*PATHFIND_CELL_SIZE_F + 0.05f;
			to->y = clipToCell.y*PATHFIND_CELL_SIZE_F + 0.05f;
		}
	}

}

struct TightenPathStruct
{
	Object *obj;
	const LocomotorSet *locomotorSet;
	PathfindLayerEnum layer;
	Int		radius;
	Bool	center;
	Bool	foundNewDest;
	Coord3D orgDestPos;
	Coord3D newDestPos;
};


/*static*/ Int Pathfinder::tightenPathCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	TightenPathStruct* d = (TightenPathStruct*)userData;
	if (from == nullptr || to==nullptr) return 0; // failure
	if (d->layer != to->getLayer()) {
		return 0; // failure
	}

#if RETAIL_COMPATIBLE_CRC
	// TheSuperHackers @bugfix Caball009 27/02/2026 This was originally uninitialized.
	// The uninitialized values that retail uses here are usually close to zero as long as foundNewDest == false, otherwise it uses the new values of newDestPos.
	// newDestPos is zero initialized by the caller, so there is no need to check foundNewDest here.
	Coord3D pos = d->newDestPos;
#else
	Coord3D pos = d->orgDestPos;
#endif

	if (!TheAI->pathfinder()->checkForAdjust(d->obj, *d->locomotorSet, true, to_x, to_y, to->getLayer(), d->radius, d->center, &pos, nullptr))
	{
		return 0; // failure
	}
	d->foundNewDest = true;
	d->newDestPos = pos;

	return 0; // success but continue
}

/* Returns the cost, which is in the same units as coord3d distance. */
void Pathfinder::tightenPath(Object *obj, const LocomotorSet& locomotorSet, Coord3D *from,
		const Coord3D *to)
{
	TightenPathStruct info;

	getRadiusAndCenter(obj, info.radius, info.center);
	info.layer = TheTerrainLogic->getLayerForDestination(from);
	info.obj = obj;
	info.locomotorSet = &locomotorSet;
	info.foundNewDest = false;
	info.orgDestPos = *to;
#if RETAIL_COMPATIBLE_CRC
	info.newDestPos.zero();
#endif
	iterateCellsAlongLine(*from, *to, info.layer, tightenPathCallback, &info);
	if (info.foundNewDest) {
		*from = info.newDestPos;
	}
}


/* Returns the cost, which is in the same units as coord3d distance. */
Int Pathfinder::checkPathCost(Object *obj, const LocomotorSet& locomotorSet, const Coord3D *from,
		const Coord3D *rawTo)
{
	//CRCDEBUG_LOG(("Pathfinder::checkPathCost()"));
	if (m_isMapReady == false) return 0;
	enum {MAX_COST = 0x7fff0000};
	if (!obj) return MAX_COST;

	Int cellCount = 0;

	Coord3D adjustTo = *rawTo;
	Coord3D *to = &adjustTo;
	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));
	// create unique "mark" values for open and closed cells for this pathfind invocation

	Bool isCrusher = obj ? obj->getCrusherLevel() > 0 : false;

	PathfindLayerEnum goalLayer = TheTerrainLogic->getLayerForDestination(to);
	// determine goal cell
	PathfindCell *goalCell = getClippedCell( goalLayer,  to );
	if (goalCell == nullptr)
		return MAX_COST;


	Bool center;
	Int radius;
	getRadiusAndCenter(obj, radius, center);

	// determine start cell
	ICoord2D startCellNdx;
	worldToCell(from, &startCellNdx);
	PathfindLayerEnum fromLayer = TheTerrainLogic->getLayerForDestination(from);
	PathfindCell *parentCell = getCell( fromLayer, from );
	if (parentCell == nullptr)
		return MAX_COST;
	ICoord2D pos2d;
	worldToCell(to, &pos2d);
	if (!goalCell->allocateInfo(pos2d)) {
		return MAX_COST;
	}

	if (parentCell!=goalCell) {
		if (!parentCell->allocateInfo(startCellNdx)) {
			goalCell->releaseInfo();
			return MAX_COST;
		}
	}

	if (validMovementPosition( isCrusher, locomotorSet.getValidSurfaces(), parentCell ) == false) {
		parentCell->releaseInfo();
		goalCell->releaseInfo();
		return MAX_COST;
	}

	parentCell->startPathfind(goalCell);

	// initialize "open" list to contain start cell
	beginOpenSearch(parentCell);

	// "closed" list is initially empty
	m_closedList.reset();

	//
	// Continue search until "open" list is empty, or
	// until goal is found.
	//
	while (hasOpenCells())
	{
		// take head cell off of open list - it has lowest estimated total path cost
		parentCell = popNextOpenCell();
		if (!parentCell) {
			break;
		}

		// put parent cell onto closed list - its evaluation is finished - Retail compatible behaviour
#if RETAIL_COMPATIBLE_PATHFINDING
		if (!s_useFixedPathfinding) {
			parentCell->putOnClosedList( m_closedList );
		}
#endif

		if (parentCell==goalCell) {
			Int cost = parentCell->getTotalCost();
			m_isTunneling = false;
			cleanOpenAndClosedLists();
#if RETAIL_COMPATIBLE_PATHFINDING
			if (s_useFixedPathfinding)
#endif
			{
				parentCell->releaseInfo();
			}
			return cost;
		}

		// put parent cell onto closed list - its evaluation is finished - Fixed behaviour
#if RETAIL_COMPATIBLE_PATHFINDING
		if (s_useFixedPathfinding)
#endif
		{
			parentCell->putOnClosedList( m_closedList );
		}

		if (cellCount > MAX_CELL_COUNT) {
			continue;
		}
		// Check to see if we can change layers in this cell.
		checkChangeLayers(parentCell);

		// expand search to neighboring orthogonal cells
		static ICoord2D delta[] =
		{
			{ 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 },
			{ 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 }
		};
		const Int numNeighbors = 8;
		const Int firstDiagonal = 4;
		ICoord2D newCellCoord;
		PathfindCell *newCell;
		const Int adjacent[5] = {0, 1, 2, 3, 0};
		Bool neighborFlags[8] = { 0 };

		UnsignedInt newCostSoFar = 0;

		for( int i=0; i<numNeighbors; i++ )
		{
			neighborFlags[i] = false;
			// determine neighbor cell to try
			newCellCoord.x = parentCell->getXIndex() + delta[i].x;
			newCellCoord.y = parentCell->getYIndex() + delta[i].y;

			// get the neighboring cell
			newCell = getCell(parentCell->getLayer(), newCellCoord.x, newCellCoord.y );

			// check if cell is on the map
			if (newCell == nullptr)
				continue;

			// check if this neighbor cell is already on the open (waiting to be tried)
			// or closed (already tried) lists
			Bool onList = false;
			if (newCell->hasInfo()) {
				if (newCell->getOpen() || newCell->getClosed())
				{
					// already on one of the lists
					onList = true;
				}
			}
			if (i>=firstDiagonal) {
				// make sure one of the adjacent sides is open.
				if (!neighborFlags[adjacent[i-4]] && !neighborFlags[adjacent[i-3]]) {
					continue;
				}
			}

			if (!validMovementPosition( isCrusher, locomotorSet.getValidSurfaces(), newCell, parentCell )) {
				continue;
			}

			neighborFlags[i] = true;

			if (!newCell->allocateInfo(newCellCoord)) {
				// Out of cells for pathing...
#if RETAIL_COMPATIBLE_PATHFINDING
				if (s_useFixedPathfinding)
#endif
				{
					cleanOpenAndClosedLists();
					parentCell->releaseInfo();
					goalCell->releaseInfo();
				}
 				return cellCount;
			}
			cellCount++;

			newCostSoFar = newCell->costSoFar( parentCell );
			newCell->setBlockedByAlly(false);

			// check if this neighbor cell is already on the open (waiting to be tried)
			// or closed (already tried) lists
			if (onList)
			{
				// already on one of the lists - if existing costSoFar is less,
				// the new cell is on a longer path, so skip it
				if (newCell->getCostSoFar() <= newCostSoFar)
					continue;
			}

			// keep track of path we're building - point back to cell we moved here from
			newCell->setParentCell(parentCell) ;

			// store cost of this path
			newCell->setCostSoFar(newCostSoFar);

			Int costRemaining = 0;
			if (goalCell) {
				costRemaining = newCell->costToGoal( goalCell );
			}

			newCell->setTotalCost(newCell->getCostSoFar() + costRemaining) ;

			// if newCell was on closed list, remove it from the list
			if (newCell->getClosed())
				newCell->removeFromClosedList( m_closedList );

			// if the newCell was already on the open list, remove it so it can be re-inserted in order
			if (newCell->getOpen())
				newCell->removeFromOpenList( m_openList );

#if RETAIL_COMPATIBLE_PATHFINDING
			// TheSuperHacker @info This is here to catch a retail pathfinding crash point and to recover from it
			// A cell has gotten onto the open list without pathfinding info due to a danling m_open pointer on the previous listed cell so we need to force a cleanup
			if (!s_useFixedPathfinding && m_openList.getHead() && !m_openList.getHead()->hasInfo()) {
				s_useFixedPathfinding = true;
				forceCleanCells();
				return MAX_COST;
			}
#endif

			// insert newCell in open list such that open list is sorted, smallest total path cost first
			newCell->putOnSortedOpenList( m_openList );
		}
	}

	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding) {
		if (goalCell->hasInfo() && !goalCell->getClosed() && !goalCell->getOpen()) {
			goalCell->releaseInfo();
		}
		cleanOpenAndClosedLists();
	}
	else
#endif
	{
		cleanOpenAndClosedLists();
		parentCell->releaseInfo();
		goalCell->releaseInfo();
	}
	return MAX_COST;
}


/**
 * Find a short, valid path between the FROM location and a location NEAR the to location.
 * Uses A* algorithm.
 */
Path *Pathfinder::findClosestPath( Object *obj, const LocomotorSet& locomotorSet, const Coord3D *from,
																	Coord3D *rawTo, Bool blocked, Real pathCostMultiplier, Bool moveAllies)
{
	PerfTrace::ScopedPathTimer timer(getPerfTraceFrame(), PerfTrace::PATH_TIMER_ASTAR_EXPAND);
	//CRCDEBUG_LOG(("Pathfinder::findClosestPath()"));
#ifdef DEBUG_LOGGING
	Int startTimeMS = ::GetTickCount();
#endif
	Bool isHuman = true;
	if (obj && obj->getControllingPlayer() && (obj->getControllingPlayer()->getPlayerType()==PLAYER_COMPUTER)) {
		isHuman = false; // computer gets to cheat.
	}

	if (locomotorSet.getValidSurfaces() == 0) {
		DEBUG_CRASH(("Attempting to path immobile unit."));
		return nullptr;
	}

	if (m_isMapReady == false) return nullptr;

	m_isTunneling = false;

	if (!obj) return nullptr;

	Bool canPathThroughUnits = false;
	if (obj && obj->getAIUpdateInterface()) {
		canPathThroughUnits = obj->getAIUpdateInterface()->canPathThroughUnits();
	}
	Bool centerInCell;
	Int radius;
	getRadiusAndCenter(obj, radius, centerInCell);

	Coord3D adjustTo = *rawTo;
	Coord3D *to = &adjustTo;
	if (!centerInCell) {
		adjustTo.x += PATHFIND_CELL_SIZE_F/2;
		adjustTo.y += PATHFIND_CELL_SIZE_F/2;
	}
	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));
	// create unique "mark" values for open and closed cells for this pathfind invocation

	Bool isCrusher = obj ? obj->getCrusherLevel() > 0 : false;


	Coord3D clipFrom = *from;
	clip(&clipFrom, &adjustTo);

	PathfindLayerEnum destinationLayer = TheTerrainLogic->getLayerForDestination(to);
	// determine goal cell
	PathfindCell *goalCell = getClippedCell( destinationLayer,  to );
	if (goalCell == nullptr)
		return nullptr;

	if (goalCell->getZone()==0 && destinationLayer==LAYER_WALL) {
		return nullptr;
	}

	Bool goalOnObstacle = false;
	if (m_ignoreObstacleID != INVALID_ID) {
		// Check for object on structure.
		// srj sez: check for obstacle on AIRFIELD... only want to do this for things
		// that are "parked" on the airfield, but not for things hovering over an obstacle
		// (eg, a chinook over a supply dock).
		Object *goalObj = TheGameLogic->findObjectByID(m_ignoreObstacleID);
		if (goalObj) {
			PathfindCell *ignoreCell = getClippedCell(goalObj->getLayer(), goalObj->getPosition());
			if ( (goalCell->getObstacleID()==ignoreCell->getObstacleID()) && (goalCell->getObstacleID() != INVALID_ID) ) {
				Object* newObstacle = TheGameLogic->findObjectByID(goalCell->getObstacleID());
#if RTS_GENERALS
				if (newObstacle != nullptr && newObstacle->isKindOf(KINDOF_AIRFIELD))
#else
				if (newObstacle != nullptr && newObstacle->isKindOf(KINDOF_FS_AIRFIELD))
#endif
				{
					m_ignoreObstacleID = goalCell->getObstacleID();
					goalOnObstacle = true;
				}
				else
				{
					if (m_ignoreObstacleID == goalCell->getObstacleID()) {
						goalOnObstacle = true;
					}
				}
			}
		}
	}

	// determine start cell
	ICoord2D startCellNdx;
	worldToCell(from, &startCellNdx);
 	PathfindCell *parentCell = getClippedCell( obj->getLayer(), &clipFrom );
	if (parentCell == nullptr)
		return nullptr;

	if (validMovementPosition( isCrusher, locomotorSet.getValidSurfaces(), parentCell ) == false) {
		m_isTunneling = true; // We can't move from our current location.  So relax the constraints.
	}
	TCheckMovementInfo info;
	info.cell = startCellNdx;
	info.layer = obj->getLayer();
	info.centerInCell = centerInCell;
	info.radius = radius;
	info.considerTransient = blocked;
	info.acceptableSurfaces = locomotorSet.getValidSurfaces();
	if (!checkForMovement(obj, info) || info.enemyFixed) {
		m_isTunneling = true; // We can't move from our current location.  So relax the constraints.
	}

	Bool gotHierarchicalPath = false;
	if (m_isTunneling) {
		m_zoneManager.setAllPassable(); // can't optimize.
	}	else {
		m_zoneManager.clearPassableFlags();
		Path *hPat = findClosestHierarchicalPath(isHuman, locomotorSet, from, rawTo, false);
		if (hPat) {
			deleteInstance(hPat);
			gotHierarchicalPath = true;
		}	else {
			m_zoneManager.setAllPassable();
		}
	}
	const Bool startedStuck = m_isTunneling;

	ICoord2D pos2d;
	worldToCell(to, &pos2d);
	if (!goalCell->allocateInfo(pos2d)) {
		return nullptr;
	}
	if (parentCell!=goalCell) {
		worldToCell(&clipFrom, &pos2d);
		if (!parentCell->allocateInfo(pos2d)) {
#if RETAIL_COMPATIBLE_PATHFINDING
			if (s_useFixedPathfinding)
#endif
			{
				goalCell->releaseInfo();
			}
			return nullptr;
		}
	}
	parentCell->startPathfind(goalCell);

	PathfindCell *closesetCell = nullptr;
	Real closestDistanceSqr = FLT_MAX;
	Real closestDistScreenSqr = FLT_MAX;

	// initialize "open" list to contain start cell
	beginOpenSearch(parentCell);

	// "closed" list is initially empty
	m_closedList.reset();
	Int count = 0;

	//
	// Continue search until "open" list is empty, or
	// until goal is found.
	//
	Bool foundGoal = false;
	while (hasOpenCells())
	{
		Real dx;
		Real dy;
		Real distSqr;
		// take head cell off of open list - it has lowest estimated total path cost
		parentCell = popNextOpenCell();
		if (!parentCell) {
			break;
		}

		if (parentCell == goalCell)
		{
			// success - found a path to the goal
			if (!goalOnObstacle) {
				// See if the goal is a valid destination.  If not, accept closest cell.
				if (closesetCell!=nullptr && !canPathThroughUnits && !checkDestination(obj, parentCell->getXIndex(), parentCell->getYIndex(), parentCell->getLayer(), radius, centerInCell)) {
#if RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING
					break;
#else
					foundGoal = true;
					// Continue processing the open list to find a possibly closer cell. jba. [8/25/2003]
					continue;
#endif
				}
			}

			Bool show = TheGlobalData->m_debugAI;
#ifdef INTENSE_DEBUG
			Int count = 0;
			PathfindCell *cur;
			for (cur = m_closedList.getHead(); cur; cur=cur->getNextOpen()) {
				count++;
			}
			if (count>1000) {
				show = true;
				DEBUG_LOG(("FCP - cells %d obj %s %x", count, obj->getTemplate()->getName().str(), obj));
#ifdef STATE_MACHINE_DEBUG
				if( obj->getAIUpdateInterface() )
				{
					DEBUG_LOG(("State %s",  obj->getAIUpdateInterface()->getCurrentStateName().str()));
				}
#endif
				TheScriptEngine->AppendDebugMessage("Big path FCP", false);
			}
#endif
			if (show)
				debugShowSearch(true);
			m_isTunneling = false;
			// construct and return path
			Path *path = buildActualPath( obj, locomotorSet.getValidSurfaces(), from, goalCell, centerInCell, blocked);
#if RETAIL_COMPATIBLE_PATHFINDING
			if (!s_useFixedPathfinding) {
				parentCell->releaseInfo();
				goalCell->releaseInfo();
				cleanOpenAndClosedLists();
			}
			else
#endif
			{
				cleanOpenAndClosedLists();
				parentCell->releaseInfo();
				goalCell->releaseInfo();
			}

			return path;
		}
		// put parent cell onto closed list - its evaluation is finished
		parentCell->putOnClosedList( m_closedList );
		if (!m_isTunneling && checkDestination(obj, parentCell->getXIndex(), parentCell->getYIndex(), parentCell->getLayer(), radius, centerInCell)) {
			if (!startedStuck || validMovementPosition( isCrusher, locomotorSet.getValidSurfaces(), parentCell )) {
				dx = IABS(goalCell->getXIndex()-parentCell->getXIndex());
				dy = IABS(goalCell->getYIndex()-parentCell->getYIndex());
				distSqr = dx*dx+dy*dy;
				if (distSqr<closestDistScreenSqr) {
					closestDistScreenSqr = distSqr;
				}
				distSqr += (parentCell->getCostSoFar()*(parentCell->getCostSoFar()*COST_TO_DISTANCE_FACTOR_SQR))*pathCostMultiplier;
				if (distSqr < closestDistanceSqr) {
					closesetCell = parentCell;
					closestDistanceSqr = distSqr;
				}
			}
		}

		dx = IABS(goalCell->getXIndex()-parentCell->getXIndex());
		dy = IABS(goalCell->getYIndex()-parentCell->getYIndex());
		distSqr = dx*dx+dy*dy;
		// If we are 2x farther than the closest location already found, don't continue.
		if (distSqr > closestDistScreenSqr*4) {
			Bool skip = false;
			if (!gotHierarchicalPath) {
				skip = true;
			}
			if (count>2000) {
				skip = true;
			}
			if (closestDistScreenSqr < 10*10*PATHFIND_CELL_SIZE_F) {
				skip = true;
			}
			if (skip) {
				continue;
			}
		}
		// If we haven't already found the goal cell, continue examining. [8/25/2003]
		if (!foundGoal) {
			// Check to see if we can change layers in this cell.
			checkChangeLayers(parentCell);
			count += examineNeighboringCells(parentCell, goalCell, locomotorSet, isHuman, centerInCell, radius, startCellNdx, obj, NO_ATTACK);
		}
	}

	if (closesetCell) {
		// success - found a path to near the goal

		Bool show = TheGlobalData->m_debugAI;

#ifdef INTENSE_DEBUG
		if (count>5000) {
			show = true;
			DEBUG_LOG(("FCP CC cells %d obj %s %x", count, obj->getTemplate()->getName().str(), obj));
#ifdef STATE_MACHINE_DEBUG
			if( obj->getAIUpdateInterface() )
			{
				DEBUG_LOG(("State %s",  obj->getAIUpdateInterface()->getCurrentStateName().str()));
			}
#endif

			DEBUG_LOG(("%d Pathfind(findClosestPath) chugged from (%f,%f) to (%f,%f) --", TheGameLogic->getFrame(), from->x, from->y, to->x, to->y));
			DEBUG_LOG(("Unit '%s', time %f", obj->getTemplate()->getName().str(), (::GetTickCount()-startTimeMS)/1000.0f));
#ifdef INTENSE_DEBUG
			TheScriptEngine->AppendDebugMessage("Big path FCP CC", false);
#endif
		}
#endif
		if (show)
			debugShowSearch(true);

		m_isTunneling = false;
		rawTo->x = closesetCell->getXIndex()*PATHFIND_CELL_SIZE_F + PATHFIND_CELL_SIZE_F/2.0f;
		rawTo->y = closesetCell->getYIndex()*PATHFIND_CELL_SIZE_F + PATHFIND_CELL_SIZE_F/2.0f;
		// construct and return path
		Path *path = buildActualPath( obj, locomotorSet.getValidSurfaces(), from, closesetCell, centerInCell, blocked );
#if RETAIL_COMPATIBLE_PATHFINDING
		if (!s_useFixedPathfinding) {
			goalCell->releaseInfo();
			cleanOpenAndClosedLists();
		}
		else
#endif
		{
			cleanOpenAndClosedLists();
			parentCell->releaseInfo();
			goalCell->releaseInfo();
		}
		return path;
	}

	// failure - goal cannot be reached
#ifdef DEBUG_LOGGING
	Bool valid;
	valid = validMovementPosition( isCrusher, obj->getLayer(), locomotorSet, to ) ;

	DEBUG_LOG(("Pathfind(findClosestPath) failed from (%f,%f) to (%f,%f), original valid %d --", TheGameLogic->getFrame(), from->x, from->y, to->x, to->y, valid));
	DEBUG_LOG(("Unit '%s', time %f", obj->getTemplate()->getName().str(), (::GetTickCount()-startTimeMS)/1000.0f));
#endif
#if defined(RTS_DEBUG)
	if (TheGlobalData->m_debugAI)
		debugShowSearch(false);
#endif
#ifdef DUMP_PERF_STATS
	TheGameLogic->incrementOverallFailedPathfinds();
#endif
	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding) {
		goalCell->releaseInfo();
		cleanOpenAndClosedLists();
	}
	else
#endif
	{
		cleanOpenAndClosedLists();
		parentCell->releaseInfo();
		goalCell->releaseInfo();
	}
	return nullptr;
}


void Pathfinder::adjustCoordToCell(Int cellX, Int cellY, Bool centerInCell, Coord3D &pos, PathfindLayerEnum layer)
{
	if (centerInCell) {
		pos.x = ((Real)cellX + 0.5f) * PATHFIND_CELL_SIZE_F;
		pos.y = ((Real)cellY + 0.5f) * PATHFIND_CELL_SIZE_F;
	} else {
		pos.x = ((Real)cellX+0.05) * PATHFIND_CELL_SIZE_F;
		pos.y = ((Real)cellY+0.05) * PATHFIND_CELL_SIZE_F;
	}
	pos.z = TheTerrainLogic->getLayerHeight( pos.x, pos.y, layer );
}


/**
 * Work backwards from goal cell to construct final path.
 */
Path *Pathfinder::buildActualPath( const Object *obj, LocomotorSurfaceTypeMask acceptableSurfaces, const Coord3D *fromPos,
																	PathfindCell *goalCell, Bool center, Bool blocked )
{
	DEBUG_ASSERTCRASH( goalCell, ("Pathfinder::buildActualPath: goalCell == nullptr") );

	Path *path = newInstance(Path);

	if (goalCell->getPinched() && goalCell->getParentCell() && !goalCell->getParentCell()->getPinched()) {
		goalCell = goalCell->getParentCell();
	}

	prependCells(path, fromPos, goalCell, center);

	// cleanup the path by checking line of sight
	path->optimize(obj, acceptableSurfaces, blocked);

#if defined(RTS_DEBUG)
	if (TheGlobalData->m_debugAI==AI_DEBUG_PATHS)
	{
		extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
 		RGBColor color;
		color.blue = 0;
		color.red = color.green = 1;
		Coord3D pos;
		PathNode *node = path->getFirstNode();
		for( ; node; node = node->getNext() )
		{

			// create objects to show path - they decay

			pos = *node->getPosition();
			color.red = color.green = 1;
			if (node->getLayer() != LAYER_GROUND) {
				color.red = 0;
			}
			addIcon(&pos, PATHFIND_CELL_SIZE_F*.25f, 200, color);
		}

		// show optimized path
		for( node = path->getFirstNode(); node; node = node->getNextOptimized() )
		{
			pos = *node->getPosition();
			addIcon(&pos, PATHFIND_CELL_SIZE_F*.8f, 200, color);
		}
		setDebugPath(path);
	}
#endif
	return path;
}

/**
 * Work backwards from goal cell to construct final path.
 */
void Pathfinder::prependCells( Path *path, const Coord3D *fromPos,
																	PathfindCell *goalCell, Bool center )
{
	// traverse path cells in REVERSE order, creating path in desired order
	// skip the LAST node, as that will be in the same cell as the unit itself - so use the unit's position
	Coord3D pos;
	PathfindCell *cell, *prevCell = nullptr;
	Bool goalCellNull = (goalCell->getParentCell()==nullptr);
	for( cell = goalCell; cell->getParentCell(); cell = cell->getParentCell() )
	{
		m_zoneManager.setPassable(cell->getXIndex(), cell->getYIndex(), true);
		adjustCoordToCell(cell->getXIndex(), cell->getYIndex(), center, pos, cell->getLayer());
		if (prevCell && cell->getXIndex()==prevCell->getXIndex() && cell->getYIndex()==prevCell->getYIndex()) {
			// transitioning layers.
			PathfindLayerEnum layer = cell->getLayer();
			if (layer==LAYER_GROUND) {
				layer = prevCell->getLayer();
			}
			DEBUG_ASSERTCRASH(layer!=LAYER_GROUND, ("Should have at 1 non-ground layer. jba"));
			path->getFirstNode()->setLayer(layer);
			continue;
		}

		Bool canOptimize = true;
		if (cell->getType() == PathfindCell::CELL_CLIFF) {
			if (prevCell && prevCell->getType() != PathfindCell::CELL_CLIFF) {
				if (path->getFirstNode()) {
					path->getFirstNode()->setCanOptimize(false);
				}
			}
		}	else {
			if (prevCell && prevCell->getType() == PathfindCell::CELL_CLIFF) {
				canOptimize = false;
			}
		}

		path->prependNode( &pos, cell->getLayer() );
		path->getFirstNode()->setCanOptimize(canOptimize);
		if (cell->isBlockedByAlly()) {
			path->setBlockedByAlly(true);
		}
		if (prevCell) {
			prevCell->clearParentCell();
		}
		prevCell = cell;
	}

#if RETAIL_COMPATIBLE_PATHFINDING
	// TheSuperHackers @info This pathway is here for retail compatibility, it is to catch when a starting cell has a dangling parent that contains no pathing information
	// Beyond this point a retail client will crash due to a null pointer access within cell->getXIndex()
	// To recover from this we set the cell to the previous cell, which should be the actual starting cell then set to use the fixed pathing and perform a forced cleanup
	if (!s_useFixedPathfinding) {
		if (!cell->hasInfo()) {
			cell = prevCell;

			m_zoneManager.setPassable(cell->getXIndex(), cell->getYIndex(), true);
			if (goalCellNull) {
				// Very short path.
				adjustCoordToCell(cell->getXIndex(), cell->getYIndex(), center, pos, cell->getLayer());
				path->prependNode( &pos, cell->getLayer() );
			}
			// put actual start position as first node on the path, so it begins right at the unit's feet
			if (fromPos->x != path->getFirstNode()->getPosition()->x || fromPos->y != path->getFirstNode()->getPosition()->y) {
				path->prependNode( fromPos, cell->getLayer() );
			}

			s_useFixedPathfinding = true;
			forceCleanCells();
			return;
		}
	}
#endif

	m_zoneManager.setPassable(cell->getXIndex(), cell->getYIndex(), true);
	if (goalCellNull) {
		// Very short path.
		adjustCoordToCell(cell->getXIndex(), cell->getYIndex(), center, pos, cell->getLayer());
		path->prependNode( &pos, cell->getLayer() );
	}
	// put actual start position as first node on the path, so it begins right at the unit's feet
	if (fromPos->x != path->getFirstNode()->getPosition()->x || fromPos->y != path->getFirstNode()->getPosition()->y) {
		path->prependNode( fromPos, cell->getLayer() );
	}

}

void Pathfinder::setDebugPath(Path *newDebugpath)
{
	if (TheGlobalData->m_debugAI)
	{
		// copy the path for debugging
		deleteInstance(debugPath);
		debugPath = newInstance(Path);

		for( PathNode *copyNode = newDebugpath->getFirstNode(); copyNode; copyNode = copyNode->getNextOptimized() )
			debugPath->appendNode( copyNode->getPosition(), copyNode->getLayer() );
	}

}

/**
 * Given two world-space points, call callback for each cell.
 * Uses Bresenham line algorithm from www.gamedev.net.
 */
Int Pathfinder::iterateCellsAlongLine( const Coord3D& startWorld, const Coord3D& endWorld,
																			PathfindLayerEnum layer, CellAlongLineProc proc, void* userData )
{
	ICoord2D start, end;
	worldToCell( &startWorld, &start );
	worldToCell( &endWorld, &end );
	return iterateCellsAlongLine(start, end, layer, proc, userData);
}
/**
 * Given two world-space points, call callback for each cell.
 * Uses Bresenham line algorithm from www.gamedev.net.
 */
Int Pathfinder::iterateCellsAlongLine( const ICoord2D &start, const ICoord2D &end,
																			PathfindLayerEnum layer, CellAlongLineProc proc, void* userData )
{
	Int delta_x = abs(end.x - start.x);			// The difference between the x's
	Int delta_y = abs(end.y - start.y);			// The difference between the y's
	Int x = start.x;												// Start x off at the first pixel
	Int y = start.y;												// Start y off at the first pixel

	Int xinc1, xinc2;
	if (end.x >= start.x)								// The x-values are increasing
	{
		xinc1 = 1;
		xinc2 = 1;
	}
	else																// The x-values are decreasing
	{
		xinc1 = -1;
		xinc2 = -1;
	}

	Int yinc1, yinc2;
	if (end.y >= start.y)               // The y-values are increasing
	{
		yinc1 = 1;
		yinc2 = 1;
	}
	else																// The y-values are decreasing
	{
		yinc1 = -1;
		yinc2 = -1;
	}

	Bool checkY = true;
	Int den, num, numadd, numpixels;
	if (delta_x >= delta_y)							// There is at least one x-value for every y-value
	{
		xinc1 = 0;												// Don't change the x when numerator >= denominator
		yinc2 = 0;												// Don't change the y for every iteration
		den = delta_x;
		num = delta_x / 2;
		numadd = delta_y;
		numpixels = delta_x;							// There are more x-values than y-values
	}
	else																// There is at least one y-value for every x-value
	{
		checkY = false;
		xinc2 = 0;												// Don't change the x for every iteration
		yinc1 = 0;												// Don't change the y when numerator >= denominator
		den = delta_y;
		num = delta_y / 2;
		numadd = delta_x;
		numpixels = delta_y;							// There are more y-values than x-values
	}

	PathfindCell* from = nullptr;
	for (Int curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		PathfindCell* to = getCell( layer, x, y );
		if (to==nullptr) return 0;

		Int ret = (*proc)(this, from, to, x, y, userData);
		if (ret != 0)
			return ret;

		num += numadd;										// Increase the numerator by the top of the fraction
		if (num >= den)										// Check if numerator >= denominator
		{
			num -= den;											// Calculate the new numerator value
			x += xinc1;											// Change the x as appropriate
			y += yinc1;											// Change the y as appropriate
			from = to;
			to = getCell( layer, x, y );
			if (to==nullptr) return 0;
			Int ret = (*proc)(this, from, to, x, y, userData);
			if (ret != 0)
				return ret;
		}
		x += xinc2;												// Change the x as appropriate
		y += yinc2;												// Change the y as appropriate

		from = to;
	}

	return 0;
}

//-----------------------------------------------------------------------------

static ObjectID getSlaverID(const Object* o)
{
	for (BehaviorModule** update = o->getBehaviorModules(); *update; ++update)
	{
		SlavedUpdateInterface* sdu = (*update)->getSlavedUpdateInterface();
		if (sdu != nullptr)
		{
			return sdu->getSlaverID();
		}
	}

	return INVALID_ID;
}

static ObjectID getContainerID(const Object* o)
{
	const Object* container = o ? o->getContainedBy() : nullptr;
	return container ? container->getID() : INVALID_ID;
}

struct segmentIntersectsStruct
{
	Object *theTallBuilding;
	ObjectID ignoreBuilding;
};

/*static*/ Int Pathfinder::segmentIntersectsBuildingCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	segmentIntersectsStruct* d = (segmentIntersectsStruct*)userData;

	if (to != nullptr && (to->getType() == PathfindCell::CELL_OBSTACLE))
	{
		Object *obj = TheGameLogic->findObjectByID(to->getObstacleID());
		if (obj && obj->isKindOf(KINDOF_AIRCRAFT_PATH_AROUND)) {
			if (obj->getID() == d->ignoreBuilding) {
				return 0;
			}
			d->theTallBuilding = obj;
			return 1;
		}
	}

	return 0;	// keep going
}



struct ViewBlockedStruct
{
	const Object *obj;
	const Object *objOther;
};


/*static*/ Int Pathfinder::lineBlockedByObstacleCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	const ViewBlockedStruct* d = (const ViewBlockedStruct*)userData;

	if (to != nullptr && (to->getType() == PathfindCell::CELL_OBSTACLE))
	{

		// we never block our own view!
		if (to->isObstaclePresent(d->obj->getID()))
			return 0;

		// nor does the object we're trying to see!
		if (to->isObstaclePresent(d->objOther->getID()))
			return 0;

		// if the obstacle is our container, ignore it as an obstacle.
		if (to->isObstaclePresent(getContainerID(d->obj)))
			return 0;

		// @todo: if the obstacle is objOther's container, AND it's a "visible" container, ignore it.

		// if the obstacle is the item to which we are slaved, ignore it as an obstacle.
		if (to->isObstaclePresent(getSlaverID(d->obj)))
			return 0;

		// if the obstacle is the item to which objOther is slaved, ignore it as an obstacle.
		if (to->isObstaclePresent(getSlaverID(d->objOther)))
			return 0;

		// if the obstacle is transparent, ignore it, since this callback is only used for line-of-sight. (srj)
		if (to->isObstacleTransparent())
			return 0;

		return 1;	// bail early
	}

	return 0;	// keep going
}

struct ViewAttackBlockedStruct
{
	const Object *obj;
	const Object *victim;
	const PathfindCell *victimCell;
	Int		skipCount;
};

/*static*/ Int Pathfinder::attackBlockedByObstacleCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	ViewAttackBlockedStruct* d = (ViewAttackBlockedStruct*)userData;

	if (d->skipCount>0) {
		d->skipCount--;
		return 0;
	}
	if (to != nullptr && (to->getType() == PathfindCell::CELL_OBSTACLE))
	{
		// we never block our own view!
		if (to->isObstaclePresent(d->obj->getID()))
			return 0;

		if (d->victim) {
			// nor does the object we're trying to attack!
			if (to->isObstaclePresent(d->victim->getID()))
				return 0;
			// if the obstacle is the item to which objOther is slaved, ignore it as an obstacle.
			if (to->isObstaclePresent(getSlaverID(d->victim)))
				return 0;
		}

		// if the obstacle is our container, ignore it as an obstacle.
		if (to->isObstaclePresent(getContainerID(d->obj)))
			return 0;

		// @todo: if the obstacle is objOther's container, AND it's a "visible" container, ignore it.

		// if the obstacle is the item to which we are slaved, ignore it as an obstacle.
		if (to->isObstaclePresent(getSlaverID(d->obj)))
			return 0;

		if (to->isObstacleTransparent())
			return 0;
		//Kris: Added the check for victimCell because in China01 -- after the intro, NW of your
		//base is a cream colored building that lies in a negative coord. When you order units to
		//force attack it, it crashes.
		if( d->victimCell && to->isObstaclePresent( d->victimCell->getObstacleID() ) )
		{
			// Victim is inside the bounds of another object.  We don't let this block us,
			// as usually it is on the edge and it looks like we should be able to shoot it. jba.
			return 0;
		}
		return 1;	// bail early
	}

	return 0;	// keep going
}

//-----------------------------------------------------------------------------
Bool Pathfinder::isViewBlockedByObstacle(const Object* obj, const Object* objOther)
{
	ViewBlockedStruct info;
	info.obj = obj;
	info.objOther = objOther;
	if (objOther && objOther->isSignificantlyAboveTerrain()) {
		return false; // We don't check los to flying objects.  jba.
	}
#if 1
	return isAttackViewBlockedByObstacle(obj, *obj->getPosition(), objOther, *objOther->getPosition());
#else
	PathfindLayerEnum layer = objOther->getLayer();
	if (layer==LAYER_GROUND) {
		layer = obj->getLayer();
	}
	Int ret = iterateCellsAlongLine(*obj->getPosition(), *objOther->getPosition(),
		layer, lineBlockedByObstacleCallback, &info);
	return ret != 0;
#endif
}


//-----------------------------------------------------------------------------
Bool Pathfinder::isAttackViewBlockedByObstacle(const Object* attacker, const Coord3D& attackerPos, const Object* victim, const Coord3D& victimPos)
{
	//CRCDEBUG_LOG(("Pathfinder::isAttackViewBlockedByObstacle() - attackerPos is (%g,%g,%g) (%X,%X,%X)",
	//	attackerPos.x, attackerPos.y, attackerPos.z,
	//	AS_INT(attackerPos.x),AS_INT(attackerPos.y),AS_INT(attackerPos.z)));
	//CRCDEBUG_LOG(("Pathfinder::isAttackViewBlockedByObstacle() - victimPos is (%g,%g,%g) (%X,%X,%X)",
	//	victimPos.x, victimPos.y, victimPos.z,
	//	AS_INT(victimPos.x),AS_INT(victimPos.y),AS_INT(victimPos.z)));
	// Global switch to turn this off in case it doesn't work.
	if (!TheAI->getAiData()->m_attackUsesLineOfSight)
	{
		//CRCDEBUG_LOG(("Pathfinder::isAttackViewBlockedByObstacle() 1"));
		return false;
	}

	// If the attacker doesn't need line of sight, isn't blocked.
	if (!attacker->isKindOf(KINDOF_ATTACK_NEEDS_LINE_OF_SIGHT))
	{
		//CRCDEBUG_LOG(("Pathfinder::isAttackViewBlockedByObstacle() 2"));
		return false;
	}

// srj sez: this is a good start at taking terrain into account for attacks, but findAttackPath needs to be smartened also
#define LOS_TERRAIN
#ifdef LOS_TERRAIN
	const Weapon* w = attacker->getCurrentWeapon();
	if (attacker->isKindOf(KINDOF_IMMOBILE)) {
		// Don't take terrain blockage into account, since we can't move around it. jba.
		w = nullptr;
	}
	if (w)
	{
		Bool viewBlocked;
		if (victim)
			viewBlocked = !w->isClearGoalFiringLineOfSightTerrain(attacker, attackerPos, victim);
		else
			viewBlocked = !w->isClearGoalFiringLineOfSightTerrain(attacker, attackerPos, victimPos);

		if (viewBlocked)
		{
			//CRCDEBUG_LOG(("Pathfinder::isAttackViewBlockedByObstacle() 3"));
			return true;
		}
	}
#endif

	ViewAttackBlockedStruct info;
	info.obj = attacker;
	info.victim = victim;
	PathfindLayerEnum layer = LAYER_GROUND;
	if (victim) {
		layer = victim->getLayer();
	}
	info.victimCell = getCell(layer, &victimPos);

	info.skipCount = 0;
	if (attacker->getLayer() != LAYER_GROUND)
	{
		info.skipCount = 3;	/// srj -- someone wanna tell me what this magic number means?
												/// jba - Yes, it means that if someone is on a bridge, or rooftop, they can see
												///      3 pathfind cells out of whatever they are standing on.
												/// srj -- awesome! thank you very much :-)
		if (layer==LAYER_GROUND) {
			layer = attacker->getLayer();
		}
	}

	Int ret = iterateCellsAlongLine(attackerPos, victimPos, layer, attackBlockedByObstacleCallback, &info);
	//CRCDEBUG_LOG(("Pathfinder::isAttackViewBlockedByObstacle() 4"));
	return ret != 0;
}

static void computeNormalRadialOffset(const Coord3D& from,	Coord3D& insert, const Coord3D& to,
																			Object *obj, Real radius)
{
	Real crossProduct;
	Real dx = to.x - from.x;
	Real dy = to.y -from.y;
	Coord3D objPos = *obj->getPosition();


	Real objDx = objPos.x - from.x;
	Real objDy = objPos.y - from.y;

	crossProduct = dx*objDy - dy*objDx;

	Coord3D fromToNormal;
	fromToNormal.z = 0;
	if (crossProduct>0) {
		fromToNormal.x = dy;
		fromToNormal.y = -dx;
	}	else {
		fromToNormal.x = -dy;
		fromToNormal.y = dx;
	}
	fromToNormal.normalize();
	Real length = radius;
	insert = *obj->getPosition();
	insert.x += fromToNormal.x*length;
	insert.y += fromToNormal.y*length;

}

//-----------------------------------------------------------------------------
Bool Pathfinder::segmentIntersectsTallBuilding(const PathNode *curNode,
										PathNode *nextNode,  ObjectID ignoreBuilding, Coord3D *insertPos1,  Coord3D *insertPos2,  Coord3D *insertPos3 )
{
	segmentIntersectsStruct info;
	info.theTallBuilding = nullptr;
	info.ignoreBuilding = ignoreBuilding;

	Coord3D fromPos = *curNode->getPosition();
	Coord3D toPos = *nextNode->getPosition();

	Int i;
	for (i=0; i<2; i++) {
		Int ret = iterateCellsAlongLine(fromPos, toPos, LAYER_GROUND, segmentIntersectsBuildingCallback, &info);
		if (ret!=0 && info.theTallBuilding) {
			// see if toPos is inside the radius of the tall building.
			Coord3D bldgPos = *info.theTallBuilding->getPosition();
			Coord2D delta;
			Real radius = info.theTallBuilding->getGeometryInfo().getBoundingCircleRadius() + 2*PATHFIND_CELL_SIZE_F;
			delta.x = toPos.x - bldgPos.x;
			delta.y = toPos.y - bldgPos.y;
			if (delta.length() <= radius*0.98) {
				if (delta.length() < 0.1) {
					delta.x = 1;
				}
				delta.normalize();
				delta.x *= radius;
				delta.y *= radius;
				toPos.x = bldgPos.x+delta.x;
				toPos.y = bldgPos.y+delta.y;
				nextNode->setPosition(&toPos);
				continue;
			}
			delta.x = fromPos.x - bldgPos.x;
			delta.y = fromPos.y - bldgPos.y;
			if (delta.length() <= radius*0.98) {
				if (delta.length() < 0.1) {
					delta.x = 1;
				}
				delta.normalize();
				delta.x *= radius;
				delta.y *= radius;
				fromPos.x = bldgPos.x+delta.x;
				fromPos.y = bldgPos.y+delta.y;
			}


			computeNormalRadialOffset(fromPos, *insertPos2, toPos, info.theTallBuilding, radius);
			computeNormalRadialOffset(fromPos, *insertPos1, *insertPos2, info.theTallBuilding, radius);
			computeNormalRadialOffset(*insertPos2, *insertPos3, toPos, info.theTallBuilding, radius);

			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------
Bool Pathfinder::circleClipsTallBuilding(	const Coord3D *from, const Coord3D *to, Real circleRadius, ObjectID ignoreBuilding, Coord3D *adjustTo)
{
	PartitionFilterAcceptByKindOf filterKindof(MAKE_KINDOF_MASK(KINDOF_AIRCRAFT_PATH_AROUND), KINDOFMASK_NONE);
	PartitionFilter *filters[] = { &filterKindof, nullptr };
	Object* tallBuilding = ThePartitionManager->getClosestObject(to, circleRadius, FROM_BOUNDINGSPHERE_2D, filters);
	if (tallBuilding) {
		Real radius = tallBuilding->getGeometryInfo().getBoundingCircleRadius() + 2*PATHFIND_CELL_SIZE_F;
		computeNormalRadialOffset(*from, *adjustTo, *to, tallBuilding, circleRadius+radius);
		Object* otherTallBuilding = ThePartitionManager->getClosestObject(adjustTo, circleRadius, FROM_BOUNDINGSPHERE_2D, filters);
		if (otherTallBuilding && otherTallBuilding!=tallBuilding) {
			radius = otherTallBuilding->getGeometryInfo().getBoundingCircleRadius() + 2*PATHFIND_CELL_SIZE_F;
			Coord3D tmpTo = *adjustTo;
			computeNormalRadialOffset(*from, *adjustTo, tmpTo, otherTallBuilding, circleRadius+radius);
		}
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------

struct LinePassableStruct
{
	const Object *obj;
	LocomotorSurfaceTypeMask acceptableSurfaces;
	Int radius;
	Bool centerInCell;
	Bool blocked;
	Bool allowPinched;
};

/*static*/ Int Pathfinder::linePassableCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	const LinePassableStruct* d = (const LinePassableStruct*)userData;

	Bool isCrusher = d->obj ? d->obj->getCrusherLevel() > 0 : false;
	TCheckMovementInfo info;
	info.cell.x = to_x;
	info.cell.y = to_y;
	info.layer = to->getLayer();
	info.centerInCell = d->centerInCell;
	info.radius = d->radius;
	info.considerTransient = d->blocked;
	info.acceptableSurfaces = d->acceptableSurfaces;
	if (!pathfinder->checkForMovement(d->obj, info))
	{
		return 1;	// bail out
	}

	if (info.allyFixedCount || info.enemyFixed)
	{
		return 1;	// bail out
	}

	if (!d->allowPinched && to->getPinched()) {
		return 1; // bail out.
	}

	if (from && to->getLayer() != LAYER_GROUND && from->getLayer() == to->getLayer()) {
		if (to->getType() == PathfindCell::CELL_CLEAR) {
			return 0;
		}
	}

	if (pathfinder->validMovementPosition( isCrusher, d->acceptableSurfaces, to, from ) == false)
	{
		return 1;	// bail out
	}

	return 0;	// keep going
}

//-----------------------------------------------------------------------------

struct GroundPathPassableStruct
{
	Int		diameter;
	Bool	crusher;
};

/*static*/ Int Pathfinder::groundPathPassableCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	const GroundPathPassableStruct* d = (const GroundPathPassableStruct*)userData;

	Int curDiameter = pathfinder->clearCellForDiameter(d->crusher, to_x, to_y, to->getLayer(), d->diameter);
	if (curDiameter==d->diameter) return 0;	//  good to go.
	if (from && to->getLayer() != LAYER_GROUND && from->getLayer() == to->getLayer()) {
		return 0;
	}

	return 1;	// failed.
}

//-----------------------------------------------------------------------------

/**
 * Given two world-space points, check the line of sight between them for any impassible cells.
 * Uses Bresenham line algorithm from www.gamedev.net.
 */
Bool Pathfinder::isLinePassable( const Object *obj, LocomotorSurfaceTypeMask acceptableSurfaces,
																PathfindLayerEnum layer, const Coord3D& startWorld,
																const Coord3D& endWorld, Bool blocked,
																Bool allowPinched)
{
	LinePassableStruct info;
	//CRCDEBUG_LOG(("Pathfinder::isLinePassable(): %d %d %d ", m_ignoreObstacleID, m_isMapReady, m_isTunneling));

	info.obj = obj;
	info.acceptableSurfaces = acceptableSurfaces;
	getRadiusAndCenter(obj, info.radius, info.centerInCell);
	info.blocked = blocked;
	info.allowPinched = allowPinched;

	Int ret = iterateCellsAlongLine(startWorld, endWorld, layer, linePassableCallback, (void*)&info);
	return ret == 0;
}

//-----------------------------------------------------------------------------

/**
 * Given two world-space points, check the line of sight between them for any impassible cells.
 * Uses Bresenham line algorithm from www.gamedev.net.
 */
Bool Pathfinder::isGroundPathPassable( Bool isCrusher, const Coord3D& startWorld, PathfindLayerEnum startLayer,
		const Coord3D& endWorld, Int pathDiameter)
{
	GroundPathPassableStruct info;

	info.diameter = pathDiameter;
	info.crusher = isCrusher;

	Int ret = iterateCellsAlongLine(startWorld, endWorld, startLayer, groundPathPassableCallback, (void*)&info);
	return ret == 0;
}

/**
 * Classify the cells under the bridge
 * If 'repaired' is true, bridge is repaired
 * If 'repaired' is false, bridge has been damaged to be impassable
 */
void Pathfinder::changeBridgeState( PathfindLayerEnum layer, Bool repaired)
{
	if (m_layers[layer].isUnused()) return;
	if (m_layers[layer].setDestroyed(!repaired)) {
		m_zoneManager.markZonesDirty();
	}
}

void Pathfinder::getRadiusAndCenter(const Object *obj, Int &iRadius, Bool &center)
{
	enum {MAX_RADIUS = 2};
	if (!obj)
	{
		center = true;
		iRadius = 0;
		return;
	}
	Real diameter = 2*obj->getGeometryInfo().getBoundingCircleRadius();
	if (diameter>PATHFIND_CELL_SIZE_F && diameter<2.0f*PATHFIND_CELL_SIZE_F) {
		diameter = 2.0f*PATHFIND_CELL_SIZE_F;
	}
	iRadius = REAL_TO_INT_FLOOR(diameter/PATHFIND_CELL_SIZE_F+0.3f);
	center = false;
	if (iRadius==0) iRadius++;
	if (iRadius&1)
	{
		center = true;
	}
	iRadius /= 2;
	if (iRadius > MAX_RADIUS)
	{
		iRadius = MAX_RADIUS;
		center = true;
	}
}

/**
 * Updates the goal cell for an ai unit.
 */
void Pathfinder::updateGoal( Object *obj, const Coord3D *newGoalPos, PathfindLayerEnum layer)
{
	if (obj->isKindOf(KINDOF_IMMOBILE)) {
		// Only consider mobile.
		return;
	}

	AIUpdateInterface *ai = obj->getAIUpdateInterface();
	if (!ai) return; // only consider ai objects.
	if (!ai->isDoingGroundMovement()) {
#if RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING
		Bool isUnmannedHelicopter = false;
#else
		// exception:sniped choppers are on ground
		Bool isUnmannedHelicopter = ( obj->isKindOf( KINDOF_PRODUCED_AT_HELIPAD ) && obj->isDisabledByType( DISABLED_UNMANNED  ) ) ;
#endif
		if (!isUnmannedHelicopter) {
			updateAircraftGoal(obj, newGoalPos);
			return;
		}
	}

	PathfindLayerEnum originalLayer = obj->getDestinationLayer();

	Bool layerChanged = originalLayer != layer;

	Bool doGround=false;
	Bool doLayer=false;
	if (layer==LAYER_GROUND) {
		doGround = true;
	} else {
		doLayer = true;
		if (TheTerrainLogic->objectInteractsWithBridgeEnd(obj, layer)) {
			doGround = true;
		}
	}

	ICoord2D goalCell = *ai->getPathfindGoalCell();

	Bool centerInCell;
	Int radius;
	ICoord2D newCell;
	getRadiusAndCenter(obj, radius, centerInCell);
	Int numCellsAbove = radius;
	if (centerInCell) numCellsAbove++;
	if (centerInCell) {
		newCell.x = REAL_TO_INT_FLOOR(newGoalPos->x/PATHFIND_CELL_SIZE_F);
		newCell.y = REAL_TO_INT_FLOOR(newGoalPos->y/PATHFIND_CELL_SIZE_F);
	} else {
		newCell.x = REAL_TO_INT_FLOOR(0.5f+newGoalPos->x/PATHFIND_CELL_SIZE_F);
		newCell.y = REAL_TO_INT_FLOOR(0.5f+newGoalPos->y/PATHFIND_CELL_SIZE_F);
	}
	if (!layerChanged && newCell.x==goalCell.x && newCell.y == goalCell.y) {
		return;
	}
	removeGoal(obj);

	obj->setDestinationLayer(layer);
	ai->setPathfindGoalCell(newCell);
	Int i,j;
	ICoord2D cellNdx;

	Bool warn = true;
	for (i=newCell.x-radius; i<newCell.x+numCellsAbove; i++) {
		for (j=newCell.y-radius; j<newCell.y+numCellsAbove; j++) {
			PathfindCell	*cell;
			if (doLayer) {
				cell = getCell(layer, i, j);
				if (cell) {
					if (warn && cell->getGoalUnit()!=INVALID_ID && cell->getGoalUnit() != obj->getID()) {
						warn = false;
						//Units got stuck close to each other.  jba
					}
					cellNdx.x = i;
					cellNdx.y = j;
					cell->setGoalUnit(obj->getID(), cellNdx);
				}
			}
			if (doGround) {
				cell = getCell(LAYER_GROUND, i, j);
				if (cell) {
					if (warn && cell->getGoalUnit()!=INVALID_ID && cell->getGoalUnit() != obj->getID()) {
						warn = false;
						//Units got stuck close to each other.  jba
					}
					cellNdx.x = i;
					cellNdx.y = j;
					cell->setGoalUnit(obj->getID(), cellNdx);
				}
			}
		}
	}

}

/**
 * Updates the goal cell for an ai unit.
 */
void Pathfinder::updateAircraftGoal( Object *obj, const Coord3D *newGoalPos)
{
	if (obj->isKindOf(KINDOF_IMMOBILE)) {
		// Only consider mobile.
		return;
	}
	removeGoal(obj);
	AIUpdateInterface *ai = obj->getAIUpdateInterface();
	if (!ai) return; // only consider ai objects.
	if (ai->isDoingGroundMovement()) {
		return;  // shouldn't really happen, but just in case.
	}

	// For now, we are only doing HOVER, and WINGS.
	if (!ai->isAircraftThatAdjustsDestination()) return;

	ICoord2D goalCell = *ai->getPathfindGoalCell();

	Bool centerInCell;
	Int radius;
	ICoord2D newCell;
	getRadiusAndCenter(obj, radius, centerInCell);
	Int numCellsAbove = radius;
	if (centerInCell) numCellsAbove++;
	if (centerInCell) {
		newCell.x = REAL_TO_INT_FLOOR(newGoalPos->x/PATHFIND_CELL_SIZE_F);
		newCell.y = REAL_TO_INT_FLOOR(newGoalPos->y/PATHFIND_CELL_SIZE_F);
	} else {
		newCell.x = REAL_TO_INT_FLOOR(0.5f+newGoalPos->x/PATHFIND_CELL_SIZE_F);
		newCell.y = REAL_TO_INT_FLOOR(0.5f+newGoalPos->y/PATHFIND_CELL_SIZE_F);
	}
	if (newCell.x==goalCell.x && newCell.y == goalCell.y) {
		return;
	}

	ai->setPathfindGoalCell(newCell);
	Int i,j;
	ICoord2D cellNdx;

	for (i=newCell.x-radius; i<newCell.x+numCellsAbove; i++) {
		for (j=newCell.y-radius; j<newCell.y+numCellsAbove; j++) {
			PathfindCell	*cell;
			cell = getCell(LAYER_GROUND, i, j);
			if (cell) {
				cellNdx.x = i;
				cellNdx.y = j;
				cell->setGoalAircraft(obj->getID(), cellNdx);
			}
		}
	}

}

/**
 * Removes the goal cell for an ai unit.
 * Used for a unit that is going to be moving several times, like following a waypoint path,
 * or intentionally collides with other units (like a car bomb). jba
 */
void Pathfinder::removeGoal( Object *obj)
{
	if (obj->isKindOf(KINDOF_IMMOBILE)) {
		// Only consider mobile.
		return;
	}
	AIUpdateInterface *ai = obj->getAIUpdateInterface();
	if (!ai) return; // only consider ai objects.
	ICoord2D goalCell = *ai->getPathfindGoalCell();

	Bool centerInCell;
	Int radius;
	ICoord2D newCell;
	getRadiusAndCenter(obj, radius, centerInCell);
	if (radius==0) {
		radius++;
	}
	Int numCellsAbove = radius;
	if (centerInCell) numCellsAbove++;
	newCell.x = newCell.y = -1;
	if (newCell.x==goalCell.x && newCell.y == goalCell.y) {
		return;
	}
	ICoord2D cellNdx;
	ai->setPathfindGoalCell(newCell);
	Int i,j;
	if (goalCell.x>=0 && goalCell.y>=0) {
		for (i=goalCell.x-radius; i<goalCell.x+numCellsAbove; i++) {
			for (j=goalCell.y-radius; j<goalCell.y+numCellsAbove; j++) {
				PathfindCell	*cell = getCell(LAYER_GROUND, i, j);
				if (cell) {
					if (cell->getGoalUnit()==obj->getID()) {
						cellNdx.x = i;
						cellNdx.y = j;
						cell->setGoalUnit(INVALID_ID, cellNdx);
					}
					if (cell->getGoalAircraft()==obj->getID()) {
						cellNdx.x = i;
						cellNdx.y = j;
						cell->setGoalAircraft(INVALID_ID, cellNdx);
					}
				}
				if (obj->getDestinationLayer()!=LAYER_GROUND) {
					cell = getCell( obj->getDestinationLayer(), i, j);
					if (cell) {
						if (cell->getGoalUnit()==obj->getID()) {
							cellNdx.x = i;
							cellNdx.y = j;
							cell->setGoalUnit(INVALID_ID, cellNdx);
						}
					}
				}
			}
		}
	}
}

/**
 * Updates the position cell for an ai unit.
 */
void Pathfinder::updatePos( Object *obj, const Coord3D *newPos)
{
	if (obj->isKindOf(KINDOF_IMMOBILE))
	{
		// Only consider mobile.
		return;
	}
	if (!m_isMapReady)
		return;

	AIUpdateInterface *ai = obj->getAIUpdateInterface();
	if (!ai)
		return; // only consider ai objects.

	ICoord2D curCell = *ai->getCurPathfindCell();
	if (!ai->isDoingGroundMovement())
	{
		if (curCell.x>=0 && curCell.y>=0)
		{
			removePos(obj);
		}
		return;
	}

	Bool centerInCell;
	Int radius;
	ICoord2D newCell;
	getRadiusAndCenter(obj, radius, centerInCell);
	Int numCellsAbove = radius;
	if (centerInCell)
		numCellsAbove++;
	if (centerInCell)
	{
		newCell.x = REAL_TO_INT_FLOOR(newPos->x/PATHFIND_CELL_SIZE_F);
		newCell.y = REAL_TO_INT_FLOOR(newPos->y/PATHFIND_CELL_SIZE_F);
	}
	else
	{
		newCell.x = REAL_TO_INT_FLOOR(0.5f+newPos->x/PATHFIND_CELL_SIZE_F);
		newCell.y = REAL_TO_INT_FLOOR(0.5f+newPos->y/PATHFIND_CELL_SIZE_F);
	}
	if (newCell.x==curCell.x && newCell.y == curCell.y)
	{
		return;
	}

	PathfindLayerEnum layer = obj->getLayer();
	Bool doGround=false;
	Bool doLayer=false;
	if (layer==LAYER_GROUND) {
		doGround = true;	// just have to do ground
	} else {
		doLayer = true; // have to do the layer
		if (TheTerrainLogic->objectInteractsWithBridgeEnd(obj, layer)) {
			doGround = true; // In this case, have to both layer & ground, as they overlap here.
		}
	}

	ai->setCurPathfindCell(newCell);
	Int i,j;
	ICoord2D cellNdx;
	//DEBUG_LOG(("Updating unit pos at cell %d, %d", newCell.x, newCell.y));
	if (curCell.x>=0 && curCell.y>=0) {
		for (i=curCell.x-radius; i<curCell.x+numCellsAbove; i++) {
			for (j=curCell.y-radius; j<curCell.y+numCellsAbove; j++) {
				cellNdx.x = i;
				cellNdx.y = j;
				PathfindCell	*cell = getCell(layer, i, j);
				if (cell) {
					if (cell->getPosUnit()==obj->getID()) {
						cell->setPosUnit(INVALID_ID, cellNdx);
					}
				}
				if (layer!=LAYER_GROUND) {
					// Remove from the ground, if present.
					cell = getCell(LAYER_GROUND, i, j);
					if (cell) {
						if (cell->getPosUnit()==obj->getID()) {
							cell->setPosUnit(INVALID_ID, cellNdx);
						}
					}
				}
			}
		}
	}
	for (i=newCell.x-radius; i<newCell.x+numCellsAbove; i++) {
		for (j=newCell.y-radius; j<newCell.y+numCellsAbove; j++) {
			PathfindCell	*cell;
			cellNdx.x = i;
			cellNdx.y = j;
			if (doLayer) {
				cell = getCell(layer, i, j);
				if (cell) {
					cell->setPosUnit(obj->getID(), cellNdx);
				}
			}
			if (doGround) {
				cell = getCell(LAYER_GROUND, i, j);
				if (cell) {
					cell->setPosUnit(obj->getID(), cellNdx);
				}
			}
		}
	}
}

/**
 * Removes the position cell flags for an ai unit.
 */
void Pathfinder::removePos( Object *obj)
{
	if (obj->isKindOf(KINDOF_IMMOBILE)) {
		// Only consider mobile.
		return;
	}
	if (!m_isMapReady) return;
	AIUpdateInterface *ai = obj->getAIUpdateInterface();
	if (!ai) return; // only consider ai objects.
	ICoord2D curCell = *ai->getCurPathfindCell();
	Bool centerInCell;
	Int radius;
	getRadiusAndCenter(obj, radius, centerInCell);
	Int numCellsAbove = radius;
	if (centerInCell) numCellsAbove++;
	PathfindLayerEnum layer = obj->getLayer();

	ICoord2D newCell;
	newCell.x = newCell.y = -1;
	ai->setCurPathfindCell(newCell);

	Int i,j;
	ICoord2D cellNdx;
	//DEBUG_LOG(("Updating unit pos at cell %d, %d", newCell.x, newCell.y));
	if (curCell.x>=0 && curCell.y>=0) {
		for (i=curCell.x-radius; i<curCell.x+numCellsAbove; i++) {
			for (j=curCell.y-radius; j<curCell.y+numCellsAbove; j++) {
				cellNdx.x = i;
				cellNdx.y = j;
				PathfindCell	*cell = getCell(layer, i, j);
				if (cell) {
					if (cell->getPosUnit()==obj->getID()) {
						cell->setPosUnit(INVALID_ID, cellNdx);
					}
				}
				if (layer!=LAYER_GROUND) {
					// Remove from the ground, if present.
					cell = getCell(LAYER_GROUND, i, j);
					if (cell) {
						if (cell->getPosUnit()==obj->getID()) {
							cell->setPosUnit(INVALID_ID, cellNdx);
						}
					}
				}
			}
		}
	}
}

/**
 * Removes a mobile unit from the pathfind grid.
 */
void Pathfinder::removeUnitFromPathfindMap(  Object *obj )
{
	removePos(obj);
	removeGoal(obj);
}

Bool Pathfinder::moveAllies(Object *obj, Path *path)
{
	PerfTrace::ScopedPathTimer timer(getPerfTraceFrame(), PerfTrace::PATH_TIMER_MOVE_ALLIES);

#ifdef DO_UNIT_TIMINGS
#pragma MESSAGE("*** WARNING *** DOING DO_UNIT_TIMINGS!!!!")
extern Bool g_UT_startTiming;
if (g_UT_startTiming) return false;
#endif
	if (!obj->isKindOf(KINDOF_DOZER) && !obj->isKindOf(KINDOF_HARVESTER)) {
		// Harvesters & dozers want a clear path.
		if (!path->getBlockedByAlly()) {
			return FALSE; // Only move units if it is required.
		}
	}
	LatchRestore<Int> recursiveDepth(m_moveAlliesDepth, m_moveAlliesDepth+1);
	if (m_moveAlliesDepth > 2) {
		return false;
	}

	Bool centerInCell;
	Int radius;
	getRadiusAndCenter(obj, radius, centerInCell);
	Int numCellsAbove = radius;
	if (centerInCell) numCellsAbove++;
	PathNode *node;
	ObjectID ignoreId = INVALID_ID;
	if (obj->getAIUpdateInterface()) {
		ignoreId = obj->getAIUpdateInterface()->getIgnoredObstacleID();
	}
	for( node = path->getLastNode(); node && node != path->getFirstNode(); node = node->getPrevious() )	{
		ICoord2D curCell;
		worldToCell(node->getPosition(), &curCell);
		Int i, j;
		for (i=curCell.x-radius; i<curCell.x+numCellsAbove; i++) {
			for (j=curCell.y-radius; j<curCell.y+numCellsAbove; j++) {
				PathfindCell	*cell = getCell(node->getLayer(), i, j);
				if (!cell) {
					continue; // Cell is not on the pathfinding grid
				}

				ObjectID unitId = cell->getPosUnit();
				if (unitId==INVALID_ID) {
					continue;
				}

				if (unitId==obj->getID()) {
					continue;	// It's us.
				}

				if (unitId==ignoreId) {
					continue;	 // It's the one we are ignoring.
				}

				Object *otherObj = TheGameLogic->findObjectByID(unitId);
				if (!otherObj) {
					continue;
				}

				if (obj->getRelationship(otherObj)!=ALLIES) {
					continue;  // Only move allies.
				}

				if (obj->isKindOf(KINDOF_INFANTRY) && otherObj->isKindOf(KINDOF_INFANTRY)) {
					continue;  // infantry can walk through other infantry, so just let them.
				}
				if (obj->isKindOf(KINDOF_INFANTRY) && !otherObj->isKindOf(KINDOF_INFANTRY)) {
					// If this is a general clear operation, don't let infantry push vehicles.
					if (!path->getBlockedByAlly()) {
						continue;
					}
				}

				if (!otherObj->getAI() || otherObj->getAI()->isMoving()) {
					continue;
				}

#if !(RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING)
				if (otherObj->getAI()->isAttacking()) {
					continue; // Don't move units that are attacking. [8/14/2003]
				}

				//Kris: Patch 1.01 November 3, 2003
				//Black Lotus exploit fix -- moving while hacking.
				if( otherObj->testStatus( OBJECT_STATUS_IS_USING_ABILITY ) || otherObj->getAI()->isBusy() ) {
					continue; // Packing or unpacking objects for example
				}
#endif

				//DEBUG_LOG(("Moving ally"));
				PerfTrace::NoteRepathTriggered(getPerfTraceFrame());
				otherObj->getAI()->aiMoveAwayFromUnit(obj, CMD_FROM_AI);
			}
		}
	}
	return true;
}


/**
 * Moves an allied unit out of the path of another unit.
 * Uses A* algorithm.
 */
Path *Pathfinder::getMoveAwayFromPath(Object* obj, Object *otherObj,
											Path *pathToAvoid, Object *otherObj2, Path *pathToAvoid2)
{
	if (!m_isMapReady)
		return nullptr; // Should always be ok.

#ifdef DEBUG_LOGGING
	Int startTimeMS = ::GetTickCount();
#endif
	Bool isHuman = true;
	if (obj && obj->getControllingPlayer() && (obj->getControllingPlayer()->getPlayerType()==PLAYER_COMPUTER)) {
		isHuman = false; // computer gets to cheat.
	}
	Bool otherCenter;
	Int otherRadius;
	getRadiusAndCenter(otherObj, otherRadius, otherCenter);

	Bool isCrusher = obj ? obj->getCrusherLevel() > 0 : false;

	m_zoneManager.setAllPassable();

	Bool centerInCell;
	Int radius;
	getRadiusAndCenter(obj, radius, centerInCell);

	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));

	// determine start cell
	ICoord2D startCellNdx;
	Coord3D startPos = *obj->getPosition();
	if (!centerInCell) {
		startPos.x += PATHFIND_CELL_SIZE_F*0.5f;
		startPos.x += PATHFIND_CELL_SIZE_F*0.5f;
	}
	worldToCell(&startPos, &startCellNdx);
	PathfindCell *parentCell = getClippedCell( obj->getLayer(), obj->getPosition() );
	if (!parentCell)
		return nullptr;

	if (!obj->getAIUpdateInterface()) // shouldn't happen, but can't move it without an ai.
		return nullptr; 

	const LocomotorSet& locomotorSet = obj->getAIUpdateInterface()->getLocomotorSet();

	m_isTunneling = false;
	if (validMovementPosition( isCrusher, locomotorSet.getValidSurfaces(), parentCell ) == false) {
		m_isTunneling = true; // We can't move from our current location.  So relax the constraints.
	}

	TCheckMovementInfo info;
	info.cell = startCellNdx;
	info.layer = obj->getLayer();
	info.centerInCell = centerInCell;
	info.radius = radius;
	info.considerTransient = false;
	info.acceptableSurfaces = locomotorSet.getValidSurfaces();
	if (!checkForMovement(obj, info) || info.enemyFixed) {
		m_isTunneling = true; // We can't move from our current location.  So relax the constraints.
	}

	if (!parentCell->allocateInfo(startCellNdx)) {
		return nullptr;
	}
	parentCell->startPathfind(nullptr);

	beginOpenSearch(parentCell);

	// "closed" list is initially empty
	m_closedList.reset();

	//
	// Continue search until "open" list is empty, or
	// until goal is found.
	//

	Real boxHalfWidth = radius*PATHFIND_CELL_SIZE_F - (PATHFIND_CELL_SIZE_F/4.0f);
	if (centerInCell) boxHalfWidth+=PATHFIND_CELL_SIZE_F/2;
	boxHalfWidth += otherRadius*PATHFIND_CELL_SIZE_F;
	if (otherCenter) boxHalfWidth+=PATHFIND_CELL_SIZE_F/2;

	while (hasOpenCells())
	{
		// take head cell off of open list - it has lowest estimated total path cost
		parentCell = popNextOpenCell();
		if (!parentCell) {
			break;
		}

		Region2D bounds;
		Coord3D cellCenter;
		adjustCoordToCell(parentCell->getXIndex(), parentCell->getYIndex(), centerInCell, cellCenter, parentCell->getLayer());
		bounds.lo.x = cellCenter.x-boxHalfWidth;
		bounds.lo.y = cellCenter.y-boxHalfWidth;
		bounds.hi.x = cellCenter.x+boxHalfWidth;
		bounds.hi.y = cellCenter.y+boxHalfWidth;
		PathNode *node;
		Bool overlap = false;

		for( node = pathToAvoid->getFirstNode(); node && node->getNextOptimized(); node = node->getNextOptimized() )	{
			Coord2D start, end;
			start.x = node->getPosition()->x;
			start.y = node->getPosition()->y;
			end.x = node->getNextOptimized()->getPosition()->x;
			end.y = node->getNextOptimized()->getPosition()->y;
			if (LineInRegion(&start, &end, &bounds)) {
				overlap = true;
				break;
			}
		}

		if (!overlap && pathToAvoid2) {
			for( node = pathToAvoid2->getFirstNode(); node && node->getNextOptimized(); node = node->getNextOptimized() )	{
				Coord2D start, end;
				start.x = node->getPosition()->x;
				start.y = node->getPosition()->y;
				end.x = node->getNextOptimized()->getPosition()->x;
				end.y = node->getNextOptimized()->getPosition()->y;
				if (LineInRegion(&start, &end, &bounds)) {
					overlap = true;
					break;
				}
			}
		}
		if (!overlap) {
			if (startCellNdx.x == parentCell->getXIndex() && startCellNdx.y == parentCell->getYIndex()) {
				// we didn't move. Always move at least 1 cell. jba.
				overlap = true;
			}
		}
		///@todo - Adjust cost intersecting path - closer to front is more expensive. jba.
		if (!overlap && checkDestination(obj, parentCell->getXIndex(), parentCell->getYIndex(),
				parentCell->getLayer(), radius, centerInCell)) {
			// success - found a path to the goal
			if (false && TheGlobalData->m_debugAI)
				debugShowSearch(true);
			m_isTunneling = false;
			// construct and return path
			Path *newPath = buildActualPath( obj, locomotorSet.getValidSurfaces(), obj->getPosition(), parentCell, centerInCell, false);
#if RETAIL_COMPATIBLE_PATHFINDING
			if (!s_useFixedPathfinding) {
				parentCell->releaseInfo();
				cleanOpenAndClosedLists();
			}
			else
#endif
			{
				cleanOpenAndClosedLists();
				parentCell->releaseInfo();
			}
			return newPath;
		}
		// put parent cell onto closed list - its evaluation is finished
		parentCell->putOnClosedList( m_closedList );

		// Check to see if we can change layers in this cell.
		checkChangeLayers(parentCell);

		examineNeighboringCells(parentCell, nullptr, locomotorSet, isHuman, centerInCell, radius, startCellNdx, obj, NO_ATTACK);

	}

#if defined(RTS_DEBUG)
	debugShowSearch(true);
#endif

	DEBUG_LOG(("%d getMoveAwayFromPath pathfind failed --", TheGameLogic->getFrame()));
	DEBUG_LOG(("Unit '%s', time %f", obj->getTemplate()->getName().str(), (::GetTickCount()-startTimeMS)/1000.0f));

	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding) {
		cleanOpenAndClosedLists();
	}
	else
#endif
	{
		cleanOpenAndClosedLists();
		parentCell->releaseInfo();
	}
	return nullptr;
}


/** Patch to the exiting path from the current position, either because we became blocked,
  or because we had to move off the path to avoid other units. */
Path *Pathfinder::patchPath( const Object *obj, const LocomotorSet& locomotorSet,
		Path *originalPath, Bool blocked )
{
	PerfTrace::ScopedPathTimer timer(getPerfTraceFrame(), PerfTrace::PATH_TIMER_ASTAR_EXPAND);
	//CRCDEBUG_LOG(("Pathfinder::patchPath()"));
#ifdef DEBUG_LOGGING
	Int startTimeMS = ::GetTickCount();
#endif
	if (originalPath==nullptr) return nullptr;
	Bool centerInCell;
	Int radius;
	getRadiusAndCenter(obj, radius, centerInCell);
	Bool isHuman = true;
	if (obj && obj->getControllingPlayer() && (obj->getControllingPlayer()->getPlayerType()==PLAYER_COMPUTER)) {
		isHuman = false; // computer gets to cheat.
	}

	m_zoneManager.setAllPassable();

	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));

	enum {CELL_LIMIT = 2000}; // max cells to examine.
	Int cellCount = 0;

	Coord3D currentPosition = *obj->getPosition();

	// determine start cell
	ICoord2D startCellNdx;
	Coord3D startPos = *obj->getPosition();
	if (!centerInCell) {
		startPos.x += PATHFIND_CELL_SIZE_F*0.5f;
		startPos.x += PATHFIND_CELL_SIZE_F*0.5f;
	}
	worldToCell(&startPos, &startCellNdx);
	//worldToCell(obj->getPosition(), &startCellNdx);
	PathfindCell *parentCell = getClippedCell( obj->getLayer(), &currentPosition);
	if (parentCell == nullptr)
		return nullptr;
	if (!obj->getAIUpdateInterface()) {
		return nullptr; // shouldn't happen, but can't move it without an ai.
	}

	m_isTunneling = false;

	if (!parentCell->allocateInfo(startCellNdx)) {
		return nullptr;
	}
	parentCell->startPathfind( nullptr);

	// initialize "open" list to contain start cell
	beginOpenSearch(parentCell);

	// "closed" list is initially empty
	m_closedList.reset();

	//
	// Continue search until "open" list is empty, or
	// until goal is found.
	//

#if defined(RTS_DEBUG)
	extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
	if (TheGlobalData->m_debugAI)
	{
		RGBColor color;
		color.setFromInt(0);
		addIcon(nullptr, 0,0,color);
	}
#endif

	PathNode *startNode;
	Coord3D goalPos = *originalPath->getLastNode()->getPosition();
	Real goalDeltaSqr = sqr(goalPos.x-currentPosition.x) + sqr(goalPos.y - currentPosition.y);
	for( startNode = originalPath->getLastNode(); startNode != originalPath->getFirstNode(); startNode = startNode->getPrevious() )	{
		ICoord2D cellCoord;
		worldToCell(startNode->getPosition(), &cellCoord);
		TCheckMovementInfo info;
		info.cell = cellCoord;
		info.layer = startNode->getLayer();
		info.centerInCell = centerInCell;
		info.radius = radius;
		info.considerTransient = blocked;
		info.acceptableSurfaces = locomotorSet.getValidSurfaces();
#if defined(RTS_DEBUG)
		if (TheGlobalData->m_debugAI) {
			RGBColor color;
			color.setFromInt(0);
			color.green = 1;
			addIcon(startNode->getPosition(), PATHFIND_CELL_SIZE_F*0.5f, 100, color);
		}
#endif
		Int dx = cellCoord.x-startCellNdx.x;
		Int dy = cellCoord.y-startCellNdx.y;
		if (dx<-2 || dx>2) info.considerTransient = false;
		if (dy<-2 || dy>2) info.considerTransient = false;
		if (!checkForMovement(obj, info)) {
			break;
		}
		if (info.allyFixedCount || info.enemyFixed) {
			break;	// Don't patch through cells that are occupied.
		}
		Real curSqr = sqr(startNode->getPosition()->x-currentPosition.x) + sqr(startNode->getPosition()->y - currentPosition.y);
		if (curSqr < goalDeltaSqr) {
			goalPos = *startNode->getPosition();
			goalDeltaSqr = curSqr;
		}

	}
	if (startNode == originalPath->getLastNode()) {
#if RETAIL_COMPATIBLE_PATHFINDING
		if (!s_useFixedPathfinding) {
			cleanOpenAndClosedLists();
		}
		else
#endif
		{
			parentCell->releaseInfo();
		}
		return nullptr; // no open nodes.
	}
	PathfindCell *candidateGoal;
	candidateGoal = getCell(LAYER_GROUND, &goalPos); // just using for cost estimates.
	ICoord2D goalCellNdx;
	worldToCell(&goalPos, &goalCellNdx);
	if (!candidateGoal->allocateInfo(goalCellNdx)) {
#if RETAIL_COMPATIBLE_PATHFINDING
		if (s_useFixedPathfinding)
#endif
		{
			parentCell->releaseInfo();
		}
		return nullptr;
	}

	while (hasOpenCells())
	{
		// take head cell off of open list - it has lowest estimated total path cost
		parentCell = popNextOpenCell();
		if (!parentCell) {
			break;
		}

		Coord3D cellCenter;
		adjustCoordToCell(parentCell->getXIndex(), parentCell->getYIndex(), centerInCell, cellCenter, parentCell->getLayer());
		PathNode *matchNode;
		Bool found = false;
		for( matchNode = originalPath->getLastNode(); matchNode != startNode; matchNode = matchNode->getPrevious() )	{
			if (cellCenter.x == matchNode->getPosition()->x && cellCenter.y == matchNode->getPosition()->y)	{
				found = true;
				break;
			}
		}
		if (found ) {
			// success - found a path to the goal
			if ( TheGlobalData->m_debugAI)
				debugShowSearch(true);
			m_isTunneling = false;
			// construct and return path
			Path *path = newInstance(Path);
			PathNode *node;
			for( node = originalPath->getLastNode(); node != matchNode; node = node->getPrevious() )	{
				path->prependNode(node->getPosition(), node->getLayer());
			}
			prependCells(path, obj->getPosition(), parentCell, centerInCell);

			// cleanup the path by checking line of sight
			path->optimize(obj, locomotorSet.getValidSurfaces(), blocked);
#if RETAIL_COMPATIBLE_PATHFINDING
			if (!s_useFixedPathfinding) {
				parentCell->releaseInfo();
				cleanOpenAndClosedLists();
				candidateGoal->releaseInfo();
			}
			else
#endif
			{
				cleanOpenAndClosedLists();
				parentCell->releaseInfo();
				candidateGoal->releaseInfo();
			}

			return path;
		}
		// put parent cell onto closed list - its evaluation is finished
		parentCell->putOnClosedList( m_closedList );

		if (cellCount < CELL_LIMIT) {
			// Check to see if we can change layers in this cell.
			checkChangeLayers(parentCell);
			cellCount += examineNeighboringCells(parentCell, nullptr, locomotorSet, isHuman, centerInCell, radius, startCellNdx, obj, NO_ATTACK);
		}
	}

	DEBUG_LOG(("%d patchPath Pathfind failed --", TheGameLogic->getFrame()));
	DEBUG_LOG(("Unit '%s', time %f", obj->getTemplate()->getName().str(), (::GetTickCount()-startTimeMS)/1000.0f));

#if defined(RTS_DEBUG)
	if (TheGlobalData->m_debugAI) {
		debugShowSearch(true);
	}
#endif
	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding) {
		if (!candidateGoal->getOpen() && !candidateGoal->getClosed())
		{
			// Not on one of the lists 
			candidateGoal->releaseInfo();
		}
		cleanOpenAndClosedLists();
	}
	else
#endif
	{
		cleanOpenAndClosedLists();
		parentCell->releaseInfo();
		candidateGoal->releaseInfo();
	}
	return nullptr;
}


/** Find a short, valid path to a location that obj can attack victim from.  */
Path *Pathfinder::findAttackPath( const Object *obj, const LocomotorSet& locomotorSet, const Coord3D *from,
		const Object *victim, const Coord3D* victimPos, const Weapon *weapon )
{
	PerfTrace::ScopedPathTimer timer(getPerfTraceFrame(), PerfTrace::PATH_TIMER_ASTAR_EXPAND);
	if (!m_isMapReady)
		return nullptr; // Should always be ok.

	Bool isCrusher = obj ? obj->getCrusherLevel() > 0 : false;
	Int radius;
	Bool centerInCell;
	getRadiusAndCenter(obj, radius, centerInCell);

	// Quick check:  See if moving couple of cells towards the victim will work.
	{
		Coord3D curPos = *obj->getPosition();
		Coord3D goalPos = victim?*victim->getPosition():*victimPos;
		Coord3D delta;
		delta.set(goalPos.x-curPos.x, goalPos.y-curPos.y, 0);
		delta.normalize();
		delta.x *= PATHFIND_CELL_SIZE_F;
		delta.y *= PATHFIND_CELL_SIZE_F;
		Int i;
		for (i=1; i<10; i++) {
			Coord3D testPos = curPos;
			testPos.x += delta.x*i*0.5f;
			testPos.y += delta.y*i*0.5f;

			ICoord2D cellNdx;
			worldToCell(&testPos, &cellNdx);
			PathfindCell *aCell = getCell(obj->getLayer(), cellNdx.x, cellNdx.y);
			if (!aCell)
				break;

			if (!validMovementPosition(isCrusher, locomotorSet.getValidSurfaces(), aCell))
				break;

			if (!checkDestination(obj, cellNdx.x, cellNdx.y, obj->getLayer(), radius, centerInCell))
				break;

			if (!weapon->isGoalPosWithinAttackRange(obj, &testPos, victim, victimPos))
				continue;

			if (isAttackViewBlockedByObstacle(obj, testPos, victim, *victimPos))
				continue;

			// return path.
			Path *path = newInstance(Path);
			path->prependNode( &testPos, obj->getLayer() );
			path->prependNode( &curPos, obj->getLayer() );
			path->getFirstNode()->setNextOptimized(path->getFirstNode()->getNext());
			if (TheGlobalData->m_debugAI==AI_DEBUG_PATHS) {
				setDebugPath(path);
			}
			return path;
		}
	}

	const Int ATTACK_CELL_LIMIT = 2500; // this is a rather expensive operation, so limit the search.

	Bool isHuman = true;
	if (obj && obj->getControllingPlayer() && (obj->getControllingPlayer()->getPlayerType()==PLAYER_COMPUTER)) {
		isHuman = false; // computer gets to cheat.
	}
	m_zoneManager.clearPassableFlags();
	Path *hPat = findClosestHierarchicalPath(isHuman, locomotorSet, from, victimPos, isCrusher);
	if (hPat) {
		deleteInstance(hPat);
	}	else {
		m_zoneManager.setAllPassable();
	}

	Int cellCount = 0;

	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));

	Int attackDistance = weapon->getAttackDistance(obj, victim, victimPos);
	attackDistance += 3*PATHFIND_CELL_SIZE;

		// determine start cell
	ICoord2D startCellNdx;
	Coord3D objPos = *obj->getPosition();
	// since worldtocell truncates, add.
	if (centerInCell) {
		objPos.x += PATHFIND_CELL_SIZE_F/2.0f;
		objPos.y += PATHFIND_CELL_SIZE_F/2.0f;
	}

	if (!obj->getAIUpdateInterface()) // shouldn't happen, but can't move without an ai.
		return nullptr;

	worldToCell(&objPos, &startCellNdx);
	PathfindCell *parentCell = getClippedCell( obj->getLayer(), &objPos );
	if (!parentCell)
		return nullptr;

	if (!parentCell->allocateInfo(startCellNdx))
		return nullptr;

	const PathfindCell *startCell = parentCell;
	parentCell->startPathfind(nullptr);

	// determine start cell
	ICoord2D victimCellNdx;
	worldToCell(victim ? victim->getPosition() : victimPos, &victimCellNdx);

	// determine goal cell
	PathfindCell *goalCell = getCell( LAYER_GROUND, victimCellNdx.x, victimCellNdx.y );
	if (!goalCell)
		return nullptr;

 	if (!goalCell->allocateInfo(victimCellNdx)) {
		return nullptr;
	}

	// initialize "open" list to contain start cell
	beginOpenSearch(parentCell);

	// "closed" list is initially empty
	m_closedList.reset();

	//
	// Continue search until "open" list is empty, or
	// until goal is found.
	//

	PathfindCell *closestCell = nullptr;
	Real closestDistanceSqr = FLT_MAX;
	Bool checkLOS = false;
	if (!victim) {
		checkLOS = true;
	}
	if (victim && !victim->isSignificantlyAboveTerrain()) {
		checkLOS = true;
	}

	while (hasOpenCells())
	{
		// take head cell off of open list - it has lowest estimated total path cost
		parentCell = popNextOpenCell();
		if (!parentCell) {
			break;
		}

		Coord3D cellCenter;
		adjustCoordToCell(parentCell->getXIndex(), parentCell->getYIndex(), centerInCell, cellCenter, parentCell->getLayer());

		///@todo - Adjust cost intersecting path - closer to front is more expensive. jba.
		if (weapon->isGoalPosWithinAttackRange(obj, &cellCenter, victim, victimPos) &&
			checkDestination(obj, parentCell->getXIndex(), parentCell->getYIndex(),
				parentCell->getLayer(), radius, centerInCell)) {
			// check line of sight.
			Bool viewBlocked = false;
			if (checkLOS)
			{
				viewBlocked = isAttackViewBlockedByObstacle(obj, cellCenter, victim, *victimPos);
			}
			if (startCell == parentCell) {
				// We never want to accept our starting cell.
				// If we could attack from there, we wouldn't be calling
				// FindAttackPath.  Usually happens cause the cell is valid for attack, but
				// a point near the cell center isn't, and that happens to be where the
				// attacker is standing, and it's too close to move to.
				viewBlocked = true;
			} else {
				// If through some unfortunate rounding, we end up moving near ourselves,
				// don't want it.
				Coord3D cellPos;
				adjustCoordToCell(parentCell->getXIndex(), parentCell->getYIndex(), centerInCell, cellPos, parentCell->getLayer());
				Real dx = (cellPos.x - objPos.x);
				Real dy = (cellPos.y - objPos.y);
				if (sqr(dx) + sqr(dy) < sqr(PATHFIND_CELL_SIZE_F*0.5f)) {
					viewBlocked = true;
				}
			}
			if (!viewBlocked)
			{
				// success - found a path to the goal
				Bool show = TheGlobalData->m_debugAI;
	#ifdef INTENSE_DEBUG
				Int count = 0;
				PathfindCell *cur;
				for (cur = m_closedList.getHead(); cur; cur=cur->getNextOpen()) {
					count++;
				}
				if (count>1000) {
					show = true;
					DEBUG_LOG(("FAP cells %d obj %s %x", count, obj->getTemplate()->getName().str(), obj));
	#ifdef STATE_MACHINE_DEBUG
					if( obj->getAIUpdateInterface() )
					{
						DEBUG_LOG(("State %s",  obj->getAIUpdateInterface()->getCurrentStateName().str()));
					}
	#endif
					TheScriptEngine->AppendDebugMessage("Big Attack path", false);
				}
	#endif
				if (show)
					debugShowSearch(true);

#if !(RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING)
				// put parent cell onto closed list - its evaluation is finished
				parentCell->putOnClosedList( m_closedList );

				if (obj->isKindOf(KINDOF_VEHICLE)) {
					// Strip backwards.
					PathfindCell *lastBlocked = nullptr;
					PathfindCell *cur = parentCell;
					Bool useLargeRadius = false;
					Int cellLimit = 12; // Magic number, yes I know - jba.   It is about 4 * size of an average vehicle width (3 cells) [8/15/2003]
					while (cur) {
						cellLimit--;
						if (cellLimit<0) {
							break;
						}
						TCheckMovementInfo info;
						info.cell.x = cur->getXIndex();
						info.cell.y = cur->getYIndex();
						info.layer = cur->getLayer();
						if (useLargeRadius) {
							info.centerInCell = centerInCell;
							info.radius = radius;
						} else {
							info.centerInCell = true;
							info.radius = 0;
						}
						info.considerTransient = false;
						info.acceptableSurfaces = locomotorSet.getValidSurfaces();
						PathfindCell	*cell = getCell(info.layer,info.cell.x,info.cell.y);
						Bool unitIdle = false;
						if (cell) {
							ObjectID posUnit = cell->getPosUnit();
							Object *unit = TheGameLogic->findObjectByID(posUnit);
							if (unit && unit->getAI() && unit->getAI()->isIdle()) {
								unitIdle = true;
							}
						}
						Bool checkMovement = checkForMovement(obj, info);
						Bool blockedByEnemy = info.enemyFixed;
						Bool blockedByAllies = info.allyFixedCount || info.allyGoal;
						if (unitIdle) {
							// If the unit present is idle, it doesn't block allies. [8/18/2003]
							blockedByAllies = false;
						}


						if (!checkMovement || blockedByEnemy || blockedByAllies) {
							lastBlocked = cur;
							useLargeRadius = true;
						} else {
							useLargeRadius = false;
						}
						cur = cur->getParentCell();
					}
					if (lastBlocked) {
						parentCell = lastBlocked;
						if (lastBlocked->getParentCell()) {
							parentCell = lastBlocked->getParentCell();
						}
					}
				}
#endif
				// construct and return path
				Path *path = buildActualPath( obj, locomotorSet.getValidSurfaces(), obj->getPosition(), parentCell, centerInCell, false);
#if RETAIL_COMPATIBLE_PATHFINDING
				if (!s_useFixedPathfinding) {
					if (goalCell->hasInfo() && !goalCell->getClosed() && !goalCell->getOpen()) {
						goalCell->releaseInfo();
					}
					cleanOpenAndClosedLists();
				}
				else
#endif
				{
					cleanOpenAndClosedLists();
					parentCell->releaseInfo();
					goalCell->releaseInfo();
				}
				return path;
			}
		}
		if (checkDestination(obj, parentCell->getXIndex(), parentCell->getYIndex(), parentCell->getLayer(), radius, centerInCell)) {
			if (validMovementPosition( isCrusher, locomotorSet.getValidSurfaces(), parentCell )) {
				Real dx = IABS(victimCellNdx.x-parentCell->getXIndex());
				Real dy = IABS(victimCellNdx.y-parentCell->getYIndex());
				Real distSqr = dx*dx+dy*dy;
				if (distSqr < closestDistanceSqr) {
					closestCell = parentCell;
					closestDistanceSqr = distSqr;
				}
			}
		}

		// put parent cell onto closed list - its evaluation is finished
		parentCell->putOnClosedList( m_closedList );

		if (cellCount < ATTACK_CELL_LIMIT) {
				// Check to see if we can change layers in this cell.
			checkChangeLayers(parentCell);
			cellCount += examineNeighboringCells(parentCell, goalCell, locomotorSet, isHuman, centerInCell,
				radius, startCellNdx, obj, attackDistance);
		}

	}

#ifdef INTENSE_DEBUG
	DEBUG_LOG(("obj %s %x", obj->getTemplate()->getName().str(), obj));
#ifdef STATE_MACHINE_DEBUG
	if( obj->getAIUpdateInterface() )
	{
		DEBUG_LOG(("State %s",  obj->getAIUpdateInterface()->getCurrentStateName().str()));
	}
#endif
	debugShowSearch(true);
	TheScriptEngine->AppendDebugMessage("Overflowed attack path", false);
#endif
#if 0
	if (closestCell) {
		// construct and return path
		Path *path = buildActualPath( obj, locomotorSet, obj->getPosition(), closestCell, centerInCell, false);
		cleanOpenAndClosedLists();
		return path;
	}
#if defined(RTS_DEBUG)
	DEBUG_LOG(("%d (%d cells) Attack Pathfind failed from (%f,%f) to (%f,%f) --", TheGameLogic->getFrame(), cellCount, from->x, from->y, victim->getPosition()->x, victim->getPosition()->y));
	DEBUG_LOG(("Unit '%s', attacking '%s' time %f", obj->getTemplate()->getName().str(),  victim->getTemplate()->getName().str(), (::GetTickCount()-startTimeMS)/1000.0f));
#endif
#endif
#ifdef DUMP_PERF_STATS
	TheGameLogic->incrementOverallFailedPathfinds();
#endif
	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding) {
		if (goalCell->hasInfo() && !goalCell->getClosed() && !goalCell->getOpen()) {
			goalCell->releaseInfo();
		}
		cleanOpenAndClosedLists();
	}
	else
#endif
	{
		cleanOpenAndClosedLists();
		parentCell->releaseInfo();
		goalCell->releaseInfo();
	}
	return nullptr;
}

/** Find a short, valid path to a location that is safe from the repulsors.  */
Path *Pathfinder::findSafePath( const Object *obj, const LocomotorSet& locomotorSet,
		const Coord3D *from, const Coord3D* repulsorPos1, const Coord3D* repulsorPos2, Real repulsorRadius)
{
	PerfTrace::ScopedPathTimer timer(getPerfTraceFrame(), PerfTrace::PATH_TIMER_ASTAR_EXPAND);
	//CRCDEBUG_LOG(("Pathfinder::findSafePath()"));
	if (m_isMapReady == false) return nullptr; // Should always be ok.
#if defined(RTS_DEBUG)
//	Int startTimeMS = ::GetTickCount();
#endif

	const Int MAX_CELLS = MAX_SAFE_PATH_CELL_COUNT; // this is a rather expensive operation, so limit the search.

	Bool centerInCell;
	Int radius;
	getRadiusAndCenter(obj, radius, centerInCell);
	Real repulsorDistSqr = repulsorRadius*repulsorRadius;
	Int cellCount = 0;
	Bool isHuman = true;
	if (obj && obj->getControllingPlayer() && (obj->getControllingPlayer()->getPlayerType()==PLAYER_COMPUTER)) {
		isHuman = false; // computer gets to cheat.
	}

	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));
	// create unique "mark" values for open and closed cells for this pathfind invocation

	m_zoneManager.setAllPassable();
	// determine start cell
	ICoord2D startCellNdx;
	worldToCell(obj->getPosition(), &startCellNdx);
	PathfindCell *parentCell = getClippedCell( obj->getLayer(), obj->getPosition() );
	if (parentCell == nullptr)
		return nullptr;
	if (!obj->getAIUpdateInterface()) {
		return nullptr; // shouldn't happen, but can't move it without an ai.
	}
	if (!parentCell->allocateInfo(startCellNdx)) {
		return nullptr;
	}
	parentCell->startPathfind( nullptr);

	// initialize "open" list to contain start cell
	beginOpenSearch(parentCell);

	// "closed" list is initially empty
	m_closedList.reset();
	Real farthestDistanceSqr = 0;
	while (hasOpenCells())
	{
		// take head cell off of open list - it has lowest estimated total path cost
		parentCell = popNextOpenCell();
		if (!parentCell) {
			break;
		}

		Coord3D cellCenter;
		adjustCoordToCell(parentCell->getXIndex(), parentCell->getYIndex(), centerInCell, cellCenter, parentCell->getLayer());

		///@todo - Adjust cost intersecting path - closer to front is more expensive. jba.
		Real dx = cellCenter.x-repulsorPos1->x;
		Real dy = cellCenter.y-repulsorPos1->y;
		Bool ok = false;
		Real distSqr = dx*dx+dy*dy;
		dx = cellCenter.x-repulsorPos2->x;
		dy = cellCenter.y-repulsorPos2->y;
		Real distSqr2 = dx*dx+dy*dy;
		if (distSqr2<distSqr) {
			distSqr = distSqr2;
		}
		if (distSqr>repulsorDistSqr) {
			ok = true;
		}
		if (!hasOpenCells() && cellCount>0) {
			ok = true; // exhausted the search space, just take the last cell.
		}
		if (distSqr > farthestDistanceSqr) {
			farthestDistanceSqr = distSqr;
			if (cellCount > MAX_CELLS) {
#ifdef INTENSE_DEBUG
				DEBUG_LOG(("Took intermediate path, dist %f, goal dist %f", sqrt(farthestDistanceSqr), repulsorRadius));
#endif
				ok = true; // Already a big search, just take this one.
			}
		}
		if ( ok &&
			checkDestination(obj, parentCell->getXIndex(), parentCell->getYIndex(),
				parentCell->getLayer(), radius, centerInCell)) {
			// success - found a path to the goal
			Bool show = TheGlobalData->m_debugAI;
#ifdef INTENSE_DEBUG
			Int count = 0;
			PathfindCell *cur;
			for (cur = m_closedList.getHead(); cur; cur=cur->getNextOpen()) {
				count++;
			}
			if (count>2000) {
				show = true;
				DEBUG_LOG(("cells %d obj %s %x", count, obj->getTemplate()->getName().str(), obj));
#ifdef STATE_MACHINE_DEBUG
				if( obj->getAIUpdateInterface() )
				{
					DEBUG_LOG(("State %s",  obj->getAIUpdateInterface()->getCurrentStateName().str()));
				}
#endif
				TheScriptEngine->AppendDebugMessage("Big Safe path", false);
			}
#endif
			if (show)
				debugShowSearch(true);
#if defined(RTS_DEBUG)
			//DEBUG_LOG(("Attack path took %d cells, %f sec", cellCount, (::GetTickCount()-startTimeMS)/1000.0f));
#endif
			// construct and return path
			Path *path = buildActualPath( obj, locomotorSet.getValidSurfaces(), obj->getPosition(), parentCell, centerInCell, false);
#if RETAIL_COMPATIBLE_PATHFINDING
			if (!s_useFixedPathfinding) {
				parentCell->releaseInfo();
				cleanOpenAndClosedLists();
			}
			else
#endif
			{
				cleanOpenAndClosedLists();
				parentCell->releaseInfo();
			}
			return path;
		}

		// put parent cell onto closed list - its evaluation is finished
		parentCell->putOnClosedList( m_closedList );

		// Check to see if we can change layers in this cell.
		checkChangeLayers(parentCell);

		cellCount += examineNeighboringCells(parentCell, nullptr, locomotorSet, isHuman, centerInCell, radius, startCellNdx, obj, NO_ATTACK);

	}

#ifdef INTENSE_DEBUG
	DEBUG_LOG(("obj %s %x count %d", obj->getTemplate()->getName().str(), obj, cellCount));
#ifdef STATE_MACHINE_DEBUG
	if( obj->getAIUpdateInterface() )
	{
		DEBUG_LOG(("State %s",  obj->getAIUpdateInterface()->getCurrentStateName().str()));
	}
#endif
	TheScriptEngine->AppendDebugMessage("Overflowed Safe path", false);
#endif
#if 0
#if defined(RTS_DEBUG)
	DEBUG_LOG(("%d (%d cells) Attack Pathfind failed from (%f,%f) to (%f,%f) --", TheGameLogic->getFrame(), cellCount, from->x, from->y, victim->getPosition()->x, victim->getPosition()->y));
	DEBUG_LOG(("Unit '%s', attacking '%s' time %f", obj->getTemplate()->getName().str(),  victim->getTemplate()->getName().str(), (::GetTickCount()-startTimeMS)/1000.0f));
#endif
#endif
#ifdef DUMP_PERF_STATS
	TheGameLogic->incrementOverallFailedPathfinds();
#endif
	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding) {
		cleanOpenAndClosedLists();
	}
	else
#endif
	{
		cleanOpenAndClosedLists();
		parentCell->releaseInfo();
	}
	return nullptr;
}

//-----------------------------------------------------------------------------
