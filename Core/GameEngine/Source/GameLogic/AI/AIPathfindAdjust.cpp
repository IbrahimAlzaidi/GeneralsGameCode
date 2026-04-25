#include "PreRTS.h"

#include "GameLogic/AIPathfind.h"
#include "AIPathfindInternal.h"

#include "Common/CRCDebug.h"
#include "Common/GlobalData.h"
#include "Common/Player.h"
#include "Common/PerfTrace.h"
#include "Common/ThingTemplate.h"

#include "GameLogic/AI.h"
#include "GameLogic/GameLogic.h"
#include "GameLogic/Locomotor.h"
#include "GameLogic/Object.h"
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

constexpr const UnsignedInt MAX_ADJUSTMENT_CELL_COUNT = 400;
static Int frameToShowObstacles;

static inline Bool IS_IMPASSABLE(PathfindCell::CellType type) {
	if (type==PathfindCell::CELL_IMPASSABLE) {
		return true;
	}
	if (type==PathfindCell::CELL_OBSTACLE) {
		return true;
	}
	if (type==PathfindCell::CELL_BRIDGE_IMPASSABLE) {
		return true;
	}
	return false;
}


/**
 * Show all cells touched in the last search
 */
void Pathfinder::debugShowSearch(  Bool pathFound  )
{
	if (!TheGlobalData->m_debugAI) {
		return;
	}
#if defined(RTS_DEBUG)
	extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);

	// show all explored cells for debugging
	PathfindCell *s;
	RGBColor color;
	color.red = color.blue = color.green = 1;
	if (!pathFound) {
		addIcon(nullptr, 0, 0, color);	 // erase.
	}

	for( s = m_openList.getHead(); s; s=s->getNextOpen() )
	{
		// create objects to show path - they decay
		RGBColor color;
		color.red = color.green = 0;
		color.blue = 1;

		Coord3D pos;
		pos.x = ((Real)s->getXIndex() + 0.5f) * PATHFIND_CELL_SIZE_F;
		pos.y = ((Real)s->getYIndex() + 0.5f) * PATHFIND_CELL_SIZE_F;
		pos.z = TheTerrainLogic->getLayerHeight( pos.x, pos.y, s->getLayer() ) + 0.5f;
		addIcon(&pos, PATHFIND_CELL_SIZE_F*.6f, 200, color);
	}

	for( s = m_closedList.getHead(); s; s=s->getNextOpen() )
	{
		// create objects to show path - they decay
		RGBColor color;
		color.red = color.blue = 1;
		color.green = 0;
		if (!pathFound)	color.blue = 0;

		Int length=200;
		if (!pathFound)
			length *= 2;

		Coord3D pos;
		pos.x = ((Real)s->getXIndex() + 0.5f) * PATHFIND_CELL_SIZE_F;
		pos.y = ((Real)s->getYIndex() + 0.5f) * PATHFIND_CELL_SIZE_F;
		pos.z = TheTerrainLogic->getLayerHeight( pos.x, pos.y, s->getLayer()) + 0.5f;
		addIcon(&pos, PATHFIND_CELL_SIZE_F*.6f, length, color);
	}
#endif
}

Locomotor* Pathfinder::chooseBestLocomotorForPosition(PathfindLayerEnum layer, LocomotorSet* locomotorSet, const Coord3D* pos )
{
	Int x = REAL_TO_INT_FLOOR(pos->x/PATHFIND_CELL_SIZE);
	Int y = REAL_TO_INT_FLOOR(pos->y/PATHFIND_CELL_SIZE);
	PathfindCell* cell = getCell(layer, x, y );
	// off the map? call it CELL_CLEAR...
	PathfindCell::CellType celltype = cell ? cell->getType() : PathfindCell::CELL_CLEAR;

	LocomotorSurfaceTypeMask acceptableSurfaces = validLocomotorSurfacesForCellType(celltype);
	return locomotorSet->findLocomotor(acceptableSurfaces);
}

/*static*/ LocomotorSurfaceTypeMask Pathfinder::validLocomotorSurfacesForCellType(PathfindCell::CellType t)
{
	switch (t)
	{
		case PathfindCell::CELL_CLEAR:
			return LOCOMOTORSURFACE_GROUND | LOCOMOTORSURFACE_AIR;

		case PathfindCell::CELL_WATER:
			return LOCOMOTORSURFACE_WATER | LOCOMOTORSURFACE_AIR;

		case PathfindCell::CELL_CLIFF:
			return LOCOMOTORSURFACE_CLIFF | LOCOMOTORSURFACE_AIR;

		case PathfindCell::CELL_RUBBLE:
			return LOCOMOTORSURFACE_RUBBLE | LOCOMOTORSURFACE_AIR;

		case PathfindCell::CELL_OBSTACLE:
		case PathfindCell::CELL_BRIDGE_IMPASSABLE:
		case PathfindCell::CELL_IMPASSABLE:
			return LOCOMOTORSURFACE_AIR;

		default:
			return NO_SURFACES;
	}
}

//
// Return true if we can move onto this position
//
Bool Pathfinder::validMovementTerrain( PathfindLayerEnum layer, const Locomotor* locomotor, const Coord3D *pos)
{
	Int x = REAL_TO_INT_FLOOR(pos->x/PATHFIND_CELL_SIZE);
	Int y = REAL_TO_INT_FLOOR(pos->y/PATHFIND_CELL_SIZE);

	PathfindCell *toCell = nullptr;
	toCell = getCell( layer, x, y );

	if (toCell == nullptr)
		return false;
	// Only do terrain, not obstacle cells.  jba.
	if (toCell->getType()==PathfindCell::CELL_OBSTACLE) return true;
	if (toCell->getType()==PathfindCell::CELL_IMPASSABLE) return true;
	if (toCell->getLayer()!=LAYER_GROUND && toCell->getLayer() == PathfindCell::CELL_CLEAR) {
		return true;
	}
	// check validity of destination cell
	LocomotorSurfaceTypeMask acceptableSurfaces = validLocomotorSurfacesForCellType(toCell->getType());
	if ((locomotor->getLegalSurfaces() & acceptableSurfaces) == 0)
		return false;
	return true;
}

//
// Releases the cells on the open & closed lists.
//
void Pathfinder::cleanOpenAndClosedLists() {
	Int count = 0;
	if (m_useHeapOpenList) {
		while (!m_openHeap.empty()) {
			PathfindCell *cell = m_openHeap.pop();
			if (cell && cell->hasInfo()) {
				PathfindCellInfo *info = cell->m_info;
#if RETAIL_COMPATIBLE_PATHFINDING
				if (!info && !s_useFixedPathfinding) {
					s_useFixedPathfinding = true;
					s_forceCleanCells = true;
					break;
				}
#endif
				if (info) {
					info->m_nextOpen = nullptr;
					info->m_prevOpen = nullptr;
					info->m_open = FALSE;
					info->m_openInsertOrder = 0;
					cell->releaseInfo();
					count++;
				}
			}
		}
		m_openHeap.reset();
		m_openList.reset();
	} else if (!m_openList.empty()) {
		count += PathfindCell::releaseOpenList(m_openList);
		m_openList.reset();
	}

#if RETAIL_COMPATIBLE_PATHFINDING
	if (s_forceCleanCells) {
		forceCleanCells();
		s_forceCleanCells = false;
	}
#endif

	if (!m_closedList.empty()) {
		count += PathfindCell::releaseClosedList(m_closedList);
		m_closedList.reset();
	}

#if RETAIL_COMPATIBLE_PATHFINDING
	if (s_forceCleanCells) {
		forceCleanCells();
		s_forceCleanCells = false;
	}
#endif

	m_cumulativeCellsAllocated += count;
}


//
// Return true if we can move onto this position
//
Bool Pathfinder::validMovementPosition( Bool isCrusher, LocomotorSurfaceTypeMask acceptableSurfaces,
																			 PathfindCell *toCell, PathfindCell *fromCell )
{
	if (toCell == nullptr)
		return false;

	// check if the destination cell is classified as an obstacle,
	// and we happen to be ignoring it
	if (toCell->isObstaclePresent( m_ignoreObstacleID ))
		return true;

	if (isCrusher && toCell->isObstacleFence()) {
		return true;
	}

	// check validity of destination cell
	LocomotorSurfaceTypeMask cellSurfaces = validLocomotorSurfacesForCellType(toCell->getType());
	if ((cellSurfaces & acceptableSurfaces) == 0)
		return false;

	return true;
}

/**
 * Checks to see if obj can occupy the pathfind cell at x,y.
 * Returns false if there is another unit's goal already there.
 * Assumes your locomotor already said you can go there.
 */
Bool Pathfinder::checkDestination(const Object *obj, Int cellX, Int cellY, PathfindLayerEnum layer, Int iRadius, Bool centerInCell)
{
	const UnsignedInt frame = getPerfTraceFrame();
	PerfTrace::ScopedPathTimer timer(frame, PerfTrace::PATH_TIMER_CHECK_DESTINATION);

	// If obj==nullptr, means we are checking for any ground units present.  jba.
	Int numCellsAbove = iRadius;
	if (centerInCell) numCellsAbove++;
	Bool checkForAircraft = false;
	Int i, j;
	ObjectID ignoreId = INVALID_ID;
	ObjectID objID = INVALID_ID;
	if (obj && obj->getAIUpdateInterface()) {
		ignoreId =  obj->getAIUpdateInterface()->getIgnoredObstacleID();
		checkForAircraft = obj->getAI()->isAircraftThatAdjustsDestination();
		objID = obj->getID();
	}
	for (i=cellX-iRadius; i<cellX+numCellsAbove; i++) {
		for (j=cellY-iRadius; j<cellY+numCellsAbove; j++) {
			PathfindCell	*cell = getCell(layer, i, j);
			if (!cell) {
				PerfTrace::NoteBlockedByTerrainChecks(frame);
				return false; // off the map, so can't place here.
			}

			if (checkForAircraft) {
				if (!cell->isAircraftGoal()) {
					continue;
				}
				if (cell->getGoalAircraft() == objID) {
					continue;
				}
				PerfTrace::NoteBlockedByUnitChecks(frame);
				return false;
			}

			if (cell->getType()==PathfindCell::CELL_OBSTACLE) {
				if (cell->isObstaclePresent( ignoreId ))
					continue;
				PerfTrace::NoteBlockedByTerrainChecks(frame);
				return false;
			}

#if !(RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING)
			if (IS_IMPASSABLE(cell->getType())) {
				if (cell->getType() == PathfindCell::CELL_BRIDGE_IMPASSABLE)
				{
					PerfTrace::NoteBridgeSpecialTerrainChecks(frame);
				}
				PerfTrace::NoteBlockedByTerrainChecks(frame);
				return false;
			}
#endif

			if (cell->getFlags() == PathfindCell::NO_UNITS) {
				continue;  // Nobody is here, so it's ok.
			}

			ObjectID goalUnitID = cell->getGoalUnit();
			if (goalUnitID == objID) {
				continue; // we got it.
			}

			if (goalUnitID == ignoreId) {
				continue; // we are ignoring it.
			}

			if (goalUnitID == INVALID_ID) {
				continue;
			}

			if (!obj) {
				PerfTrace::NoteBlockedByUnitChecks(frame);
				return false;
			}
			Object *unit = TheGameLogic->findObjectByID(goalUnitID);
			if (!unit) {
				continue;
			}

			// order matters: we want to know if I consider it to be an ally, not vice versa
			if (obj->getRelationship(unit) == ALLIES) {
				PerfTrace::NoteBlockedByUnitChecks(frame);
				return false; 	// Don't usurp your allies goals.  jba.
			}
			if (cell->getFlags()==PathfindCell::UNIT_PRESENT_FIXED) {
				Bool canCrush = obj->canCrushOrSquish(unit, TEST_CRUSH_OR_SQUISH);
				if (!canCrush) {
					PerfTrace::NoteBlockedByUnitChecks(frame);
					return false; // Don't move to an occupied cell.
				}
			}
		}
	}
	return true;
}

/**
 * Checks to see if obj can move through the pathfind cell at x,y.
 * Returns false if there are other units already there.
 * Assumes your locomotor already said you can go there.
 */
Bool Pathfinder::checkForMovement(const Object *obj, TCheckMovementInfo &info)
{
	const UnsignedInt frame = getPerfTraceFrame();
	PerfTrace::ScopedPathTimer timer(frame, PerfTrace::PATH_TIMER_CHECK_FOR_MOVEMENT);

	info.allyFixedCount = 0;
	info.allyMoving = false;
	info.allyGoal = false;
	info.enemyFixed = false;

	const Int		maxAlly = 5;
	ObjectID		allies[maxAlly];
	Int					numAlly = 0;

	if (!obj) {
		return true; // not object can move there.
	}

	ObjectID ignoreId = INVALID_ID;
	if (obj->getAIUpdateInterface()) {
		ignoreId =  obj->getAIUpdateInterface()->getIgnoredObstacleID();
	}

	Int numCellsAbove = info.radius;
	if (info.centerInCell) numCellsAbove++;
	Int i, j;
//	Bool isInfantry = obj->isKindOf(KINDOF_INFANTRY);
	for (i=info.cell.x-info.radius; i<info.cell.x+numCellsAbove; i++) {
		for (j=info.cell.y-info.radius; j<info.cell.y+numCellsAbove; j++) {
			PathfindCell	*cell = getCell(info.layer,i, j);
			if (!cell) {
				PerfTrace::NoteBlockedByTerrainChecks(frame);
				return false; // off the map, so can't move here.
			}

			PathfindCell::CellFlags flags = cell->getFlags();
			if ((flags == PathfindCell::UNIT_GOAL) || (flags == PathfindCell::UNIT_GOAL_OTHER_MOVING)) {
				info.allyGoal = true;
			} else if (flags == PathfindCell::NO_UNITS) {
				continue;  // Nobody is here, so it's ok.
			}

			ObjectID posUnit = cell->getPosUnit();
			if (posUnit == obj->getID()) {
				continue; // we got it.
			}

			if (posUnit == ignoreId) {
				continue; // we are ignoring this one.
			}

			Bool check = false;
			Object *unit = nullptr;
			if (flags == PathfindCell::UNIT_PRESENT_MOVING || flags == PathfindCell::UNIT_GOAL_OTHER_MOVING) {
				unit = TheGameLogic->findObjectByID(posUnit);
				// order matters: we want to know if I consider it to be an ally, not vice versa
				if (unit && obj->getRelationship(unit) == ALLIES) {
					info.allyMoving = true;
				}
				if (info.considerTransient) {
					check = true;
				}
			}
			if (flags == PathfindCell::UNIT_PRESENT_FIXED) {
				check = true;
				unit = TheGameLogic->findObjectByID(posUnit);
			}
			if (check && unit!=nullptr) {
				if (obj->getAIUpdateInterface() && obj->getAIUpdateInterface()->getIgnoredObstacleID()==unit->getID()) {
					// Don't check if it's the ignored obstacle.
					check = false;
				}
			}
			if (!check || !unit) {
				continue;
			}

#ifdef INFANTRY_MOVES_THROUGH_INFANTRY
			if (obj->isKindOf(KINDOF_INFANTRY) && unit->isKindOf(KINDOF_INFANTRY)) {
				// Infantry can run through infantry.
				continue; //
			}
#endif
			// See if it is an ally.
			// order matters: we want to know if I consider it to be an ally, not vice versa
			if (obj->getRelationship(unit) == ALLIES) {
				if (!unit->getAIUpdateInterface()) {
					PerfTrace::NoteBlockedByUnitChecks(frame);
					return false; // can't path through not-idle units.
				}
#if RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING
				if (!unit->getAIUpdateInterface()->isIdle()) {
					PerfTrace::NoteBlockedByUnitChecks(frame);
					return false; // can't path through not-idle units.
				}
#endif
				Bool found = false;
				Int k;
				for (k=0; k<numAlly; k++) {
					if (allies[k] == unit->getID()) {
						found = true;
					}
				}
				if (!found) {
					info.allyFixedCount++;
					if (numAlly < maxAlly) {
						allies[numAlly] = unit->getID();
						numAlly++;
					}
				}
			} else {
				Bool canCrush = obj->canCrushOrSquish( unit, TEST_CRUSH_OR_SQUISH );
				if (!canCrush) {
					info.enemyFixed = true;
					PerfTrace::NoteBlockedByUnitChecks(frame);
				}
			}
		}
	}
	return true;
}

/**
 * Adjusts a coordinate to the center of it's cell.
 */
// Snaps the current position to it's grid location.
void Pathfinder::snapPosition(Object *obj, Coord3D *pos)
{
	Int iRadius;
	Bool center;
	getRadiusAndCenter(obj, iRadius, center);
	ICoord2D cell;
	Coord3D adjustDest = *pos;
	if (!center) {
		adjustDest.x += PATHFIND_CELL_SIZE_F/2;
		adjustDest.y += PATHFIND_CELL_SIZE_F/2;
	}
	worldToCell( &adjustDest, &cell );
	adjustCoordToCell(cell.x, cell.y,  center, *pos, LAYER_GROUND);
}

/**
 * Adjusts a goal position to the center of it's cell.
 */
// Snaps the current position to it's grid location.
void Pathfinder::snapClosestGoalPosition(Object *obj, Coord3D *pos)
{
	Int iRadius;
	Bool center;
	getRadiusAndCenter(obj, iRadius, center);
	ICoord2D cell;
	Coord3D adjustDest = *pos;
	if (!center) {
		adjustDest.x += PATHFIND_CELL_SIZE_F/2;
		adjustDest.y += PATHFIND_CELL_SIZE_F/2;
	}
	PathfindLayerEnum layer = TheTerrainLogic->getLayerForDestination(pos);
	worldToCell( &adjustDest, &cell );
	adjustCoordToCell(cell.x, cell.y,  center, *pos, LAYER_GROUND);
	if (checkDestination(obj, cell.x, cell.y , layer, iRadius, center)) {
		return;
	}

	// Try adjusting by 1.
	Int i,j;
	for (i = cell.x - 1; i < cell.x + 2; i++) {
		for (j = cell.y - 1; j < cell.y + 2; j++) {
			if (checkDestination(obj, i, j, layer, iRadius, center)) {
				adjustCoordToCell(i, j, center, *pos, layer);
				return;
			}
		}
	}

	if (iRadius > 0)
		return;

	// Try to find an unoccupied cell.
	for (i = cell.x - 1; i < cell.x + 2; i++) {
		for (j = cell.y - 1; j < cell.y + 2; j++) {
			PathfindCell* newCell = getCell(layer, i, j);
			if (!newCell)
				continue;

			if (newCell->getGoalUnit() == INVALID_ID || newCell->getGoalUnit() == obj->getID()) {
				adjustCoordToCell(i, j, center, *pos, layer);
				return;
			}
		}
	}

	for (i = cell.x - 1; i < cell.x + 2; i++) {
		for (j = cell.y - 1; j < cell.y + 2; j++) {
			PathfindCell* newCell = getCell(layer, i, j);
			if (!newCell)
				continue;

			if (newCell->getFlags()!=PathfindCell::UNIT_PRESENT_FIXED) {
				adjustCoordToCell(i, j, center, *pos, layer);
				return;
			}
		}
	}
}

/**
 * Returns coordinates of goal.
 *
 */
Bool Pathfinder::goalPosition(Object *obj, Coord3D *pos)
{
	Int iRadius;
	Bool center;
	AIUpdateInterface *ai = obj->getAIUpdateInterface();
	if (!ai) return false; // only consider ai objects.
	getRadiusAndCenter(obj, iRadius, center);
	ICoord2D cell = *ai->getPathfindGoalCell();
	pos->zero();
	if (cell.x<0 || cell.y<0) return false;
	adjustCoordToCell(cell.x, cell.y,  center, *pos, LAYER_GROUND);
	return true;
}


Bool Pathfinder::checkForAdjust(Object *obj, const LocomotorSet& locomotorSet, Bool isHuman,
																Int cellX, Int cellY, PathfindLayerEnum layer,
																Int iRadius, Bool center, Coord3D *dest, const Coord3D *groupDest)
{
	Coord3D adjustDest;
	PathfindCell *cellP = getCell(layer, cellX, cellY);
	if (cellP==nullptr) return false;
	if (cellP && cellP->getType() == PathfindCell::CELL_CLIFF) {
		return false;  // no final destinations on cliffs.
	}
	if (isHuman) {
		// check if new cell is in logical map.	(computer can move off logical map)
		if (cellX < m_logicalExtent.lo.x ||
				cellY < m_logicalExtent.lo.y ||
				cellX > m_logicalExtent.hi.x ||
				cellY > m_logicalExtent.hi.y) return false;
	}
	if (checkDestination(obj, cellX, cellY, layer, iRadius, center)) {
		adjustCoordToCell(cellX, cellY,  center, adjustDest, cellP->getLayer());
		Bool pathExists;
		Bool adjustedPathExists;
		if (obj->isKindOf(KINDOF_AIRCRAFT)) {
			pathExists = true;
			adjustedPathExists = true;
		}	else {
			pathExists = clientSafeQuickDoesPathExist( locomotorSet, obj->getPosition(), dest);
			adjustedPathExists = clientSafeQuickDoesPathExist( locomotorSet, obj->getPosition(), &adjustDest);
			if (!pathExists) {
				if (clientSafeQuickDoesPathExist( locomotorSet, dest, &adjustDest))	{
 					adjustedPathExists = true;
				}
			}
		}
		if ( adjustedPathExists	) {
			if (groupDest) {
				tightenPath(obj, locomotorSet, &adjustDest, groupDest);
				// Check to see if it is a long way to get to the adjusted destination.
				Int cost = checkPathCost(obj, locomotorSet, groupDest, &adjustDest);
				Int dx = IABS(groupDest->x-adjustDest.x);
				Int dy = IABS(groupDest->y-adjustDest.y);
				if (1.4f*(dx+dy)<cost) {
					return false;
				}
			}
			*dest = adjustDest;
			return true;
		}
	}
	return false;
}

Bool Pathfinder::checkForLanding(Int cellX, Int cellY, PathfindLayerEnum layer,
																Int iRadius, Bool center, Coord3D *dest)
{
	Coord3D adjustDest;
	PathfindCell *cellP = getCell(layer, cellX, cellY);
	if (cellP==nullptr) return false;
	switch (cellP->getType())
	{
		case PathfindCell::CELL_CLIFF:
		case PathfindCell::CELL_WATER:
		case PathfindCell::CELL_IMPASSABLE:
			return false;  // no final destinations on cliffs, water, etc.
	}
	if (checkDestination(nullptr, cellX, cellY, layer, iRadius, center)) {
		adjustCoordToCell(cellX, cellY,  center, adjustDest, cellP->getLayer());
		*dest = adjustDest;
		return true;
	}
	return false;
}

/**
 * Find an unoccupied spot for a unit to land at.
 * Returns false if there are no spots available within a reasonable radius.
 */
Bool Pathfinder::adjustToLandingDestination(Object *obj, Coord3D *dest)
{
	Int iRadius;
	Bool center;
	getRadiusAndCenter(obj, iRadius, center);
	ICoord2D cell;
	Coord3D adjustDest = *dest;

	Region3D extent;
	TheTerrainLogic->getMaximumPathfindExtent(&extent);
	// If the object is off the map & the goal is off the map, it is a scripted setup, so just
	// go to the dest.
	if (!extent.isInRegionNoZ(dest)) {
		if (!extent.isInRegionNoZ(obj->getPosition())) {
			return true;
		}
	}

	if (!center) {
		adjustDest.x += PATHFIND_CELL_SIZE_F/2;
		adjustDest.y += PATHFIND_CELL_SIZE_F/2;
	}
	worldToCell( &adjustDest, &cell );

	Int limit = MAX_ADJUSTMENT_CELL_COUNT;
	Int i, j;
	i = cell.x;
	j = cell.y;
	PathfindLayerEnum layer = TheTerrainLogic->getLayerForDestination(dest);
	if (checkForLanding(i,j, layer, iRadius, center, dest)) {
		return true;
	}

	Int delta=1;
	Int count;
	while (limit>0) {
		for (count = delta; count>0; count--) {
			i++;
			limit--;
			if (checkForLanding(i,j, layer, iRadius, center, dest)) {
				return true;
			}
		}
		for (count = delta; count>0; count--) {
			j++;
			limit--;
			if (checkForLanding(i,j, layer, iRadius, center, dest)) {
				return true;
			}
		}
		delta++;
		for (count = delta; count>0; count--) {
			i--;
			limit--;
			if (checkForLanding(i,j, layer, iRadius, center, dest)) {
				return true;
			}
		}
		for (count = delta; count>0; count--) {
			j--;
			limit--;
			if (checkForLanding(i,j, layer, iRadius, center, dest)) {
				return true;
			}
		}
		delta++;
	}
	return false;
}


/**
 * Find an unoccupied spot for a unit to move to.
 * Returns false if there are no spots available within a reasonable radius.
 */
Bool Pathfinder::adjustDestination(Object *obj, const LocomotorSet& locomotorSet, Coord3D *dest, const Coord3D *groupDest)
{
	if( obj->isKindOf(KINDOF_PROJECTILE) )
	{
		return true; // missiles can go wherever they want to. jba.
	}

	Bool isHuman = true;
	if (obj && obj->getControllingPlayer() && (obj->getControllingPlayer()->getPlayerType()==PLAYER_COMPUTER)) {
		isHuman = false; // computer gets to cheat.
	}
	Int iRadius;
	Bool center;
	getRadiusAndCenter(obj, iRadius, center);
	ICoord2D cell;
	Coord3D adjustDest = *dest;
	if (!center) {
		adjustDest.x += PATHFIND_CELL_SIZE_F/2;
		adjustDest.y += PATHFIND_CELL_SIZE_F/2;
	}
	worldToCell( &adjustDest, &cell );
	PathfindLayerEnum layer = TheTerrainLogic->getLayerForDestination(dest);
	if (groupDest) {
		layer = TheTerrainLogic->getLayerForDestination(groupDest);
	}

	Int limit = MAX_ADJUSTMENT_CELL_COUNT;
	Int i, j;
	i = cell.x;
	j = cell.y;
	if (checkForAdjust(obj, locomotorSet, isHuman, i,j, layer, iRadius, center, dest, groupDest)) {
		return true;
	}

	Int delta=1;
	Int count;
	while (limit>0) {
		for (count = delta; count>0; count--) {
			i++;
			limit--;
			if (checkForAdjust(obj, locomotorSet, isHuman, i,j, layer, iRadius, center, dest, groupDest)) {
				return true;
			}
		}
		for (count = delta; count>0; count--) {
			j++;
			limit--;
			if (checkForAdjust(obj, locomotorSet, isHuman, i,j, layer, iRadius, center, dest, groupDest)) {
				return true;
			}
		}
		delta++;
		for (count = delta; count>0; count--) {
			i--;
			limit--;
			if (checkForAdjust(obj, locomotorSet, isHuman, i,j, layer, iRadius, center, dest, groupDest)) {
				return true;
			}
		}
		for (count = delta; count>0; count--) {
			j--;
			limit--;
			if (checkForAdjust(obj, locomotorSet, isHuman, i,j, layer, iRadius, center, dest, groupDest)) {
				return true;
			}
		}
		delta++;
	}
	if (groupDest) {
		// Didn't work, so just do simple adjust.
		return(adjustDestination(obj, locomotorSet, dest, nullptr));
	}
	return false;
}

Bool Pathfinder::checkForTarget(const Object *obj, 	Int cellX, Int cellY, const Weapon *weapon,
																const Object *victim, const Coord3D *victimPos,
																Int iRadius, Bool center,Coord3D *dest)
{
	Coord3D adjustDest;
	if (checkDestination(obj, cellX, cellY, LAYER_GROUND, iRadius, center)) {
		adjustCoordToCell(cellX, cellY,  center, adjustDest, LAYER_GROUND);
		if (weapon->isGoalPosWithinAttackRange( obj, &adjustDest, victim, victimPos ))	{
			*dest = adjustDest;
			return true;
		}
	}
	return false;
}

/**
 * Find an unoccupied spot for a unit to move to that can fire at victim.
 * Returns false if there are no spots available within a reasonable radius.
 */
Bool Pathfinder::adjustTargetDestination(const Object *obj, const Object *target, const Coord3D *targetPos,
																				 const Weapon *weapon, Coord3D *dest)
{
	Int iRadius;
	Bool center;
	getRadiusAndCenter(obj, iRadius, center);
	ICoord2D cell;
	Coord3D adjustDest = *dest;
	if (!center) {
		adjustDest.x += PATHFIND_CELL_SIZE_F/2;
		adjustDest.y += PATHFIND_CELL_SIZE_F/2;
	}
	if (worldToCell( &adjustDest, &cell )) {
		return false; // outside of bounds.
	}

	Int limit = MAX_ADJUSTMENT_CELL_COUNT;
	Int i, j;
	i = cell.x;
	j = cell.y;
	if (checkForTarget(obj, i,j, weapon, target, targetPos, iRadius, center, dest)) {
		return true;
	}

	Int delta=1;
	Int count;
	while (limit>0) {
		for (count = delta; count>0; count--) {
			i++;
			limit--;
			if (checkForTarget(obj, i,j, weapon, target, targetPos, iRadius, center, dest)) {
				return true;
			}
		}
		for (count = delta; count>0; count--) {
			j++;
			limit--;
			if (checkForTarget(obj, i,j, weapon, target, targetPos, iRadius, center, dest)) {
				return true;
			}
		}
		delta++;
		for (count = delta; count>0; count--) {
			i--;
			limit--;
			if (checkForTarget(obj, i,j, weapon, target, targetPos, iRadius, center, dest)) {
				return true;
			}
		}
		for (count = delta; count>0; count--) {
			j--;
			limit--;
			if (checkForTarget(obj, i,j, weapon, target, targetPos, iRadius, center, dest)) {
				return true;
			}
		}
		delta++;
	}
	return false;
}

Bool Pathfinder::checkForPossible(Bool isCrusher, Int fromZone,  Bool center, const LocomotorSet& locomotorSet,
																	Int cellX, Int cellY, PathfindLayerEnum layer, Coord3D *dest, Bool startingInObstacle)
{
	PathfindCell *goalCell = getCell(layer, cellX, cellY);
	if (!goalCell) return false;
#if RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING
	if (goalCell->getType() == PathfindCell::CELL_OBSTACLE) return false;
#else
	if (IS_IMPASSABLE(goalCell->getType())) return false;
#endif
	Int zone2 =  m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), isCrusher, goalCell->getZone());
	if (startingInObstacle) {
		zone2 = m_zoneManager.getEffectiveTerrainZone(zone2);
	}
	if (fromZone==zone2) {
		adjustCoordToCell(cellX, cellY,  center, *dest, layer);
		return true;
	}
	return false;
}

/**
 * Find a pathable spot near the destination.
 * Returns false if there are no spots available within a reasonable radius.
 */
Bool Pathfinder::adjustToPossibleDestination(Object *obj, const LocomotorSet& locomotorSet,
																						 Coord3D *dest)
{
	Int radius;
	Bool center;
	getRadiusAndCenter(obj, radius, center);
	ICoord2D goalCellNdx;
	Coord3D adjustDest = *dest;
	if (!center) {
		adjustDest.x += PATHFIND_CELL_SIZE_F/2;
		adjustDest.y += PATHFIND_CELL_SIZE_F/2;
	}
	if (worldToCell( &adjustDest, &goalCellNdx )) {
		return false; // outside of bounds.
	}

	// determine goal cell
	PathfindCell *goalCell;
	PathfindLayerEnum destinationLayer = TheTerrainLogic->getLayerForDestination(dest);

	goalCell = getCell(destinationLayer, goalCellNdx.x, goalCellNdx.y);


	Coord3D from = *obj->getPosition();

	// determine start cell
	ICoord2D startCellNdx;
	worldToCell(&from, &startCellNdx);
	PathfindLayerEnum layer = LAYER_GROUND;
	if (obj) {
		layer = obj->getLayer();
	}
	PathfindCell *parentCell = getClippedCell( layer, &from );
	if (parentCell == nullptr) {
		return false;
	}

	Int zone1, zone2;
	Bool isCrusher = obj ? obj->getCrusherLevel() > 0 : false;
	zone1 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), isCrusher, parentCell->getZone());
	Bool isObstacle = false;
	if (parentCell->getType() == PathfindCell::CELL_OBSTACLE)	{
		isObstacle = true;
	}
	if (isObstacle) {
		zone1 = m_zoneManager.getEffectiveTerrainZone(zone1);
		zone1 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), isCrusher, zone1);
	}

	zone2 =  m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), isCrusher, goalCell->getZone());

	if (zone1 == zone2) {
		if (checkDestination(obj, goalCellNdx.x, goalCellNdx.y, destinationLayer, radius, center)) {
			return true;
		}
	}

	Int limit = MAX_ADJUSTMENT_CELL_COUNT;
	Int i, j;
	i = goalCellNdx.x;
	j = goalCellNdx.y;

	Int delta=1;
	Int count;
	while (limit>0) {
		for (count = delta; count>0; count--) {
			i++;
			limit--;
			if (checkForPossible(isCrusher, zone1, center, locomotorSet, i,j, destinationLayer, dest, isObstacle)) {
				if (checkDestination(obj, i, j, destinationLayer, radius, center)) {
					return true;
				}
			}
		}
		for (count = delta; count>0; count--) {
			j++;
			limit--;
			if (checkForPossible(isCrusher, zone1, center, locomotorSet, i,j, destinationLayer, dest, isObstacle)) {
				if (checkDestination(obj, i, j, destinationLayer, radius, center)) {
					return true;
				}
			}
		}
		delta++;
		for (count = delta; count>0; count--) {
			i--;
			limit--;
			if (checkForPossible(isCrusher, zone1, center, locomotorSet, i,j, destinationLayer, dest, isObstacle)) {
				if (checkDestination(obj, i, j, destinationLayer, radius, center)) {
					return true;
				}
			}
		}
		for (count = delta; count>0; count--) {
			j--;
			limit--;
			if (checkForPossible(isCrusher, zone1, center, locomotorSet, i,j, destinationLayer, dest, isObstacle)) {
				if (checkDestination(obj, i, j, destinationLayer, radius, center)) {
					return true;
				}
			}
		}
		delta++;
	}
	return false;
}


/**
 * Queues an object to do a pathfind.
 * It will call the object's ai update->doPathfind() during processPathfindQueue().
 */

#if defined(RTS_DEBUG)
void Pathfinder::doDebugIcons() {
	const Int FRAMES_TO_SHOW_OBSTACLES = 100;
	extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
	// render AI debug information
	if (TheGlobalData->m_debugAI!=AI_DEBUG_CELLS && TheGlobalData->m_debugAI!=AI_DEBUG_TERRAIN) {
		return;
	}

		RGBColor color;
		color.red = color.green = color.blue = 0;
		addIcon(nullptr, 0, 0, color);	 // clear.
		Coord3D topLeftCorner;
		Bool showCells = TheGlobalData->m_debugAI==AI_DEBUG_CELLS;
		Int i;
		for (i=0; i<=LAYER_LAST; i++) {
			m_layers[i].doDebugIcons();
		}
		if (!showCells)	{
			frameToShowObstacles = TheGameLogic->getFrame()+FRAMES_TO_SHOW_OBSTACLES;
			//return;
		}
		// show the pathfind grid
		for( int j=0; j<getExtent()->y; j++ )
		{
			topLeftCorner.y = (Real)j * PATHFIND_CELL_SIZE_F;

			for( int i=0; i<getExtent()->x; i++ )
			{
				topLeftCorner.x = (Real)i * PATHFIND_CELL_SIZE_F;

				color.red = color.green = color.blue = 0;
				Bool empty = true;

				const PathfindCell *cell = TheAI->pathfinder()->getCell( LAYER_GROUND, i, j );
				if (cell)
				{
					switch (cell->getType())
					{
						case PathfindCell::CELL_CLIFF:
							color.red = 1;
							empty = false;
							break;
						case PathfindCell::CELL_BRIDGE_IMPASSABLE:
							color.blue = color.red = 1;
							empty = false;
							break;
						case PathfindCell::CELL_IMPASSABLE:
							color.green = 1;
							empty = false;
							break;

						case PathfindCell::CELL_WATER:
							color.blue = 1;
							empty = false;
							break;

						case PathfindCell::CELL_RUBBLE:
							color.red = 1;
							color.green = 0.5;
							empty = false;
							break;

						case PathfindCell::CELL_OBSTACLE:
							color.red = color.green = 1;
							empty = false;
							break;
						default:
							if (cell->getPinched()) {
								color.blue = color.green = 0.7f;
								empty = false;
							}
							break;
					}
				}
				if (showCells) {
					empty = true;
					color.red = color.green = color.blue = 0;
					if (empty && cell) {
						if (cell->getFlags()!=PathfindCell::NO_UNITS) {
							empty = false;
							if (cell->getFlags() == PathfindCell::UNIT_GOAL) {
								color.red = 1;
							}	else if (cell->getFlags() == PathfindCell::UNIT_PRESENT_FIXED) {
								color.green = color.blue = color.red = 1;
							}	else if (cell->getFlags() == PathfindCell::UNIT_PRESENT_MOVING) {
								color.green = 1;
							}	else {
								color.green = color.red = 1;
							}
						}
						if (cell->isAircraftGoal()) {
							empty = false;
							color.red = 0;
							color.green = color.blue = 1;
						}
					}
				}
				if (!empty) {
					Coord3D loc;
					loc.x = topLeftCorner.x + PATHFIND_CELL_SIZE_F/2.0f;
					loc.y = topLeftCorner.y + PATHFIND_CELL_SIZE_F/2.0f;
					loc.z = TheTerrainLogic->getGroundHeight(loc.x , loc.y);
					addIcon(&loc, PATHFIND_CELL_SIZE_F*0.8f, FRAMES_TO_SHOW_OBSTACLES-1, color);
				}
			}

	}
}
#endif


//-------------------------------------------------------------------------------------------------
/**
 * Create an aircraft path.  Just jogs around tall buildings marked with KINDOF_AIRCRAFT_PATH_AROUND.
 */
Path *Pathfinder::getAircraftPath( const Object *obj, const Coord3D *to )
{
	// for now, quick path objects don't pathfind, generally airborne units
	// build a trivial one-node path containing destination, then avoid buildings.
	Path *thePath = newInstance(Path);
	const AIUpdateInterface *ai = obj->getAI();
	ObjectID avoidObject = INVALID_ID;
	if (ai) {
		avoidObject = ai->getBuildingToNotPathAround();
	}

	// If it is an aircraft that circles (like raptors & migs) we need to adjust the destination
	// to one that doesn't clip buildings.
	Bool checkClips = false;
	if (ai && ai->getCurLocomotor()) {
		if (ai->getCurLocomotor()->getAppearance() == LOCO_WINGS) {
			checkClips = true;
		}
	}

	Real radius = 100;
	Coord3D adjDest = *to;
	if (checkClips) {
		circleClipsTallBuilding(obj->getPosition(), to, radius, avoidObject, &adjDest);
	}
	thePath->prependNode(&adjDest, LAYER_GROUND);
	Coord3D pos = *obj->getPosition();
	pos.z = to->z;
	thePath->prependNode( &pos, LAYER_GROUND );
	Int limit = 20;
	PathNode *curNode = thePath->getFirstNode();
	while (curNode && curNode->getNext()) {
		Coord3D newPos1, newPos2, newPos3;
		if (segmentIntersectsTallBuilding(curNode, curNode->getNext(), avoidObject, &newPos1, &newPos2, &newPos3)) {
			PathNode *newNode3 = newInstance(PathNode);
			newNode3->setPosition( &newPos3 );
			newNode3->setLayer(LAYER_GROUND);
			curNode->append(newNode3);
			PathNode *newNode2 = newInstance(PathNode);
			newNode2->setPosition( &newPos2 );
			newNode2->setLayer(LAYER_GROUND);
			curNode->append(newNode2);
			PathNode *newNode1 = newInstance(PathNode);
			newNode1->setPosition( &newPos1 );
			newNode1->setLayer(LAYER_GROUND);
			curNode->append(newNode1);
			curNode = newNode2;
		}
		curNode = curNode->getNext();
		limit--;
		if (limit<0) break;
	}

	curNode = thePath->getFirstNode();
	while (curNode && curNode->getNext()) {
		curNode->setNextOptimized(curNode->getNext());
		curNode = curNode->getNext();
	}
	thePath->markOptimized();
	if (TheGlobalData->m_debugAI==AI_DEBUG_PATHS) {
		TheAI->pathfinder()->setDebugPath(thePath);
	}

	return thePath;
}
