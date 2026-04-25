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

static UnsignedInt getRepathCooldownFrames(const Int queueDepth, const PerfTrace::PathRequestClass requestClass)
{
	if (queueDepth < static_cast<Int>(PATHFIND_REPATH_CONGESTION_THRESHOLD))
	{
		return 0;
	}

	switch (requestClass)
	{
		case PerfTrace::PATH_REQUEST_PATCH_OR_REPATH:
			return PATHFIND_REPATH_COOLDOWN_PATCH_FRAMES;
		case PerfTrace::PATH_REQUEST_SAFE_PATH_OR_AUTONOMOUS:
			return PATHFIND_REPATH_COOLDOWN_SAFE_FRAMES;
		default:
			return 0;
	}
}

static Bool isPhase3GroupSpreadActive(const Object *obj, Coord2D *offsetOut = nullptr)
{
	const Int PHASE3_MIN_GROUP_MEMBERS = 6;

	if (obj == nullptr)
	{
		return false;
	}

	Coord2D offset;
	obj->getFormationOffset(&offset);
	const Real offsetLenSqr = offset.x * offset.x + offset.y * offset.y;
	if (offsetLenSqr < sqr(PATHFIND_CELL_SIZE_F * 0.75f))
	{
		return false;
	}

	Object *mutableObj = const_cast<Object *>(obj);
	AIGroup *group = mutableObj->getGroup();
	if (obj->getFormationID() == NO_FORMATION_ID)
	{
		if (group == nullptr || group->getCount() < PHASE3_MIN_GROUP_MEMBERS)
		{
			return false;
		}
	}

	if (offsetOut != nullptr)
	{
		*offsetOut = offset;
	}
	return true;
}

static Int getPhase3LaneBiasCost(
	const Object *obj,
	const ICoord2D &startCellNdx,
	const PathfindCell *goalCell,
	const ICoord2D &candidateCell,
	const TCheckMovementInfo &info)
{
	Coord2D offset;
	if (!isPhase3GroupSpreadActive(obj, &offset) || goalCell == nullptr)
	{
		return 0;
	}

	const Int PHASE3_COST_ORTHOGONAL = 10;

	Real axisX = static_cast<Real>(goalCell->getXIndex() - startCellNdx.x);
	Real axisY = static_cast<Real>(goalCell->getYIndex() - startCellNdx.y);
	const Real axisLenSqr = axisX * axisX + axisY * axisY;
	if (axisLenSqr < 1.0f)
	{
		return 0;
	}

	const Real axisLen = sqrt(axisLenSqr);
	axisX /= axisLen;
	axisY /= axisLen;

	Coord2D right;
	right.x = -axisY;
	right.y = axisX;
	const Real desiredLateral = offset.x * right.x + offset.y * right.y;
	if (fabs(desiredLateral) < PATHFIND_CELL_SIZE_F * 0.5f)
	{
		return 0;
	}

	const Real relX = static_cast<Real>(candidateCell.x - goalCell->getXIndex()) * PATHFIND_CELL_SIZE_F;
	const Real relY = static_cast<Real>(candidateCell.y - goalCell->getYIndex()) * PATHFIND_CELL_SIZE_F;
	const Real candidateLateral = relX * right.x + relY * right.y;

	Int laneBiasCost = 0;
	if ((desiredLateral > 0.0f && candidateLateral < -PATHFIND_CELL_SIZE_F * 0.5f)
		|| (desiredLateral < 0.0f && candidateLateral > PATHFIND_CELL_SIZE_F * 0.5f))
	{
		laneBiasCost += 2 * PHASE3_COST_ORTHOGONAL;
	}

	const Real lateralDelta = fabs(candidateLateral - desiredLateral);
	if (lateralDelta > PATHFIND_CELL_SIZE_F)
	{
		const Real scaled = MIN(2.5f, lateralDelta / (PATHFIND_CELL_SIZE_F * 2.0f));
		laneBiasCost += static_cast<Int>(scaled * PHASE3_COST_ORTHOGONAL);
	}

	if (info.allyMoving)
	{
		laneBiasCost += PHASE3_COST_ORTHOGONAL;
	}
	if (info.allyGoal)
	{
		laneBiasCost += PHASE3_COST_ORTHOGONAL;
	}
	if (info.allyFixedCount > 1)
	{
		laneBiasCost += (info.allyFixedCount - 1) * PHASE3_COST_ORTHOGONAL;
	}

	return laneBiasCost;
}


void Pathfinder::beginOpenSearch(PathfindCell *startCell)
{
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding)
	{
		m_useHeapOpenList = false;
		m_openHeap.reset();
		m_openList.reset(startCell);
		if (startCell)
		{
			PerfTrace::NoteOpenListPeak(getPerfTraceFrame(), m_openList.size());
		}
		return;
	}
#endif

	m_openList.reset();
	m_openHeap.reset();
	m_useHeapOpenList = true;
	m_nextOpenInsertOrder = 1;
	if (startCell)
	{
		startCell->putOnSortedOpenList(m_openList);
	}
}

Bool Pathfinder::hasOpenCells() const
{
	return !m_openList.empty() || (m_useHeapOpenList && !m_openHeap.empty());
}

PathfindCell *Pathfinder::popNextOpenCell()
{
	PathfindCell *parentCell = nullptr;
	if (m_useHeapOpenList)
	{
		while (!m_openHeap.empty())
		{
			parentCell = m_openHeap.pop();
			if (!parentCell || !parentCell->hasInfo() || !parentCell->getOpen())
			{
				continue;
			}

			parentCell->removeFromOpenList(m_openList);
			return parentCell;
		}
	}
	if (!m_openList.empty())
	{
		parentCell = m_openList.getHead();
		parentCell->removeFromOpenList(m_openList);
		return parentCell;
	}
	return nullptr;
}

// ======================================================================

/**
 * Process some path requests in the pathfind queue.
 */
//DECLARE_PERF_TIMER(processPathfindQueue)
void Pathfinder::processPathfindQueue()
{
	const UnsignedInt frame = getPerfTraceFrame();
	PerfTrace::ScopedPathTimer timer(frame, PerfTrace::PATH_TIMER_PATHFIND_UPDATE);

	//USE_PERF_TIMER(processPathfindQueue)
	if (!m_isMapReady) {
		return;
	}
#ifdef DEBUG_QPF
	Int pathsFound = 0;
#ifdef DEBUG_LOGGING
	Int startTimeMS = ::GetTickCount();
	__int64 startTime64;
	double timeToUpdate=0.0f;
	__int64 endTime64,freq64;
	QueryPerformanceFrequency((LARGE_INTEGER *)&freq64);
	QueryPerformanceCounter((LARGE_INTEGER *)&startTime64);
#endif
#endif

	if (m_zoneManager.needToCalculateZones()) {
		PerfTrace::ScopedPathTimer zoneTimer(frame, PerfTrace::PATH_TIMER_ZONE_UPDATE);
		PerfTrace::NoteZoneRecomputes(frame);
		m_zoneManager.calculateZones(m_map, m_layers, m_extent);
		return;
	}

	// Get the current logical extent.
	Region3D terrainExtent;
	TheTerrainLogic->getExtent( &terrainExtent );
	IRegion2D bounds;
	bounds.lo.x = REAL_TO_INT_FLOOR(terrainExtent.lo.x / PATHFIND_CELL_SIZE_F);
	bounds.hi.x = REAL_TO_INT_FLOOR(terrainExtent.hi.x / PATHFIND_CELL_SIZE_F);
	bounds.lo.y = REAL_TO_INT_FLOOR(terrainExtent.lo.y / PATHFIND_CELL_SIZE_F);
	bounds.hi.y = REAL_TO_INT_FLOOR(terrainExtent.hi.y / PATHFIND_CELL_SIZE_F);
	bounds.hi.x--;
	bounds.hi.y--;
	m_logicalExtent = bounds;

	m_cumulativeCellsAllocated = 0;
	PerfTrace::NotePathQueueDepth(frame, getQueuedPathRequestCount());
	Int oldestQueueAge = 0;
	Int oldestExplicitQueueAge = 0;
	getQueueAgeMetrics(frame, oldestQueueAge, oldestExplicitQueueAge);
	PerfTrace::NoteQueueOldestAge(frame, oldestQueueAge);
	PerfTrace::NoteQueueExplicitOldestAge(frame, oldestExplicitQueueAge);
	PerfTrace::NoteCellInfoPeakLive(frame, PathfindCellInfo::getCellInfoPeakLive());
	PerfTrace::NoteCellInfoAllocFailures(frame, PathfindCellInfo::getCellInfoAllocFailures());
	PerfTrace::NoteCellInfoPoolCapacity(frame, PathfindCellInfo::getCellInfoPoolCapacity());

	for (Int pass = 1; pass <= 2; pass++)
	{
		while (m_cumulativeCellsAllocated < PATHFIND_CELLS_PER_FRAME &&
			m_queuePRTail != m_queuePRHead)
		{
			Int slot = m_queuePRHead;
			Bool found = false;
			Int scanSlot = slot;
			Int queueLen = getQueuedPathRequestCount();

			for (Int scanned = 0; scanned < queueLen; scanned++)
			{
				if (m_queuedPathfindRequests[scanSlot] != INVALID_ID)
				{
					const UnsignedByte slotClass = m_queuedRequestClasses[scanSlot];
					const Bool isHigh = isHighPriorityRequestClass(slotClass);
					if ((pass == 1 && isHigh) || (pass == 2 && !isHigh))
					{
						slot = scanSlot;
						found = true;
						break;
					}
				}
				scanSlot++;
				if (scanSlot >= PATHFIND_QUEUE_LEN) scanSlot = 0;
			}

			if (!found) break;

			PerfTrace::ScopedPathTimer dispatchTimer(frame, PerfTrace::PATH_TIMER_PATH_REQUEST_DISPATCH);
			PerfTrace::NoteQueueServicePass(frame, pass);

			Object *obj = TheGameLogic->findObjectByID(m_queuedPathfindRequests[slot]);
			const PerfTrace::PathRequestClass requestClass =
				static_cast<PerfTrace::PathRequestClass>(m_queuedRequestClasses[slot]);
			const Int waitFrames = static_cast<Int>(frame - m_queuedRequestFrames[slot]);
			const UnsignedInt goalFingerprint = m_queuedRequestGoalFingerprints[slot];
			removeQueuedRequestAt(slot);

			PerfTrace::NotePathRequestProcessed(frame, requestClass);
			PerfTrace::NoteMaxQueueWaitFrames(frame, waitFrames);

			if (obj)
			{
				AIUpdateInterface *ai = obj->getAIUpdateInterface();
				if (ai)
				{
					const Bool isHigh = isHighPriorityRequestClass(static_cast<UnsignedByte>(requestClass));
					m_requestCellBudget = isHigh ? PATHFIND_CELLS_PER_REQUEST_EXPLICIT : PATHFIND_CELLS_PER_REQUEST_LOW;
					PerfTrace::NoteRequestCellBudget(frame, m_requestCellBudget);
					const UnsignedInt cooldownFrames = getRepathCooldownFrames(getQueuedPathRequestCount(), requestClass);
					const Int recentSlot = m_recentlyServicedCursor;
					m_recentlyServicedRequestIDs[recentSlot] = obj->getID();
					m_recentlyServicedRequestFrames[recentSlot] = frame;
					m_recentlyServicedRequestClasses[recentSlot] = static_cast<UnsignedByte>(requestClass);
					m_recentlyServicedGoalFingerprints[recentSlot] = goalFingerprint;
					m_recentlyServicedCooldownUntilFrames[recentSlot] = frame + cooldownFrames;
					m_recentlyServicedCursor++;
					if (m_recentlyServicedCursor >= PATHFIND_QUEUE_LEN)
					{
						m_recentlyServicedCursor = 0;
					}

					const Int cellsBefore = m_cumulativeCellsAllocated;
					ai->doPathfind(this);
					PerfTrace::NoteLongestSingleRequestCells(frame, m_cumulativeCellsAllocated - cellsBefore);
#ifdef DEBUG_QPF
					pathsFound++;
#endif
				}
			}
			PerfTrace::NotePathQueueDepth(frame, getQueuedPathRequestCount());
		}
	}
	PerfTrace::NotePathCells(frame, m_cumulativeCellsAllocated);
#ifdef DEBUG_QPF
	if (pathsFound>0) {
#ifdef DEBUG_LOGGING
		QueryPerformanceCounter((LARGE_INTEGER *)&endTime64);
		timeToUpdate = ((double)(endTime64-startTime64) / (double)(freq64));
		if (timeToUpdate>0.01f)
		{
			DEBUG_LOG(("%d Pathfind queue: %d paths, %d cells --", TheGameLogic->getFrame(), pathsFound, m_cumulativeCellsAllocated));
			DEBUG_LOG(("time %f (%f)", timeToUpdate, (::GetTickCount()-startTimeMS)/1000.0f));
		}
#endif
	}	
#endif
#if defined(RTS_DEBUG)
	doDebugIcons();
#endif

}


void Pathfinder::checkChangeLayers(PathfindCell *parentCell)
{
	if (parentCell->getConnectLayer() == LAYER_INVALID)
		return;

	ICoord2D newCellCoord = { parentCell->getXIndex(), parentCell->getYIndex() };
	PathfindCell *newCell = getCell(parentCell->getConnectLayer(), newCellCoord.x, newCellCoord.y );

	if (!newCell) {
		DEBUG_CRASH(("Couldn't find cell."));
		return;
	}

	// already on one of the lists
	if (newCell->hasInfo() && (newCell->getOpen() || newCell->getClosed())) {
		return;
	}

	if (!newCell->allocateInfo(newCellCoord)) {
		// Out of cells for pathing...
		return;
	}
	// compute cost of path thus far
	// keep track of path we're building - point back to cell we moved here from
	newCell->setParentCell(parentCell) ;
	// store cost of this path
	newCell->setCostSoFar(parentCell->getCostSoFar()); // same as parent cost
	newCell->setTotalCost(parentCell->getTotalCost());
	// insert newCell in open list such that open list is sorted, smallest total path cost first
	newCell->putOnSortedOpenList( m_openList );
}

bool Pathfinder::checkCellOutsideExtents(ICoord2D& cell) {
	return 	cell.x < m_logicalExtent.lo.x ||
					cell.x > m_logicalExtent.hi.x ||
					cell.y < m_logicalExtent.lo.y ||
					cell.y > m_logicalExtent.hi.y;
}


struct ExamineCellsStruct
{
	Pathfinder					*thePathfinder;
	const LocomotorSet	*theLoco;
	Bool								centerInCell;
	Bool								isHuman;
	Int									radius;
	const Object				*obj;
	PathfindCell				*goalCell;
};

/*static*/ Int Pathfinder::examineCellsCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	ExamineCellsStruct* d = (ExamineCellsStruct*)userData;
	Bool isCrusher = d->obj ? d->obj->getCrusherLevel() > 0 : false;
	if (d->thePathfinder->m_isTunneling) return 1; // abort.
	if (from && to) {
			if (!d->thePathfinder->validMovementPosition( isCrusher, d->theLoco->getValidSurfaces(), to, from )) {
				return 1;
			}
			if ( (to->getLayer() == LAYER_GROUND) && !d->thePathfinder->m_zoneManager.isPassable(to_x, to_y) ) {
				return 1;
			}

			if (to->getPinched()) {
				return 1; // abort.
			}
			if (d->isHuman) {
				// check if new cell is in logical map.	(computer can move off logical map)
				if (to_x < d->thePathfinder->m_logicalExtent.lo.x) return 1; // abort
				if (to_y < d->thePathfinder->m_logicalExtent.lo.y) return 1; // abort
				if (to_x > d->thePathfinder->m_logicalExtent.hi.x) return 1; // abort
				if (to_y > d->thePathfinder->m_logicalExtent.hi.y) return 1; // abort
			}
			TCheckMovementInfo info;
			info.cell.x = to_x;
			info.cell.y = to_y;
			info.layer = from->getLayer();
			info.centerInCell = d->centerInCell;
			info.radius = d->radius;
			info.considerTransient = false;
			info.acceptableSurfaces = d->theLoco->getValidSurfaces();
			if (!d->thePathfinder->checkForMovement(d->obj, info)) {
				return 1; //abort.
			}

			if (info.enemyFixed) {
				return 1; //abort.
			}

			if (info.allyFixedCount) {
				return 1; //abort.
			}

			UnsignedInt newCostSoFar = from->getCostSoFar() + 0.5f*COST_ORTHOGONAL;
			if (to->getType() == PathfindCell::CELL_CLIFF ) {
				return 1;
			}

			ICoord2D newCellCoord;
			newCellCoord.x = to_x;
			newCellCoord.y = to_y;

			if (!to->allocateInfo(newCellCoord)) {
				// Out of cells for pathing...
 				return 1;
			}
			to->setBlockedByAlly(false);
			Int costRemaining = 0;
			costRemaining = to->costToGoal( d->goalCell );

			// check if this neighbor cell is already on the open (waiting to be tried)
			// or closed (already tried) lists
			if ( to->hasInfo() && (to->getOpen() || to->getClosed()) )
			{
				// already on one of the lists - if existing costSoFar is less,
				// the new cell is on a longer path, so skip it
				if (to->getCostSoFar() <= newCostSoFar)
					return 0; // keep going.
			}

			to->setCostSoFar(newCostSoFar);
			// keep track of path we're building - point back to cell we moved here from
			to->setParentCell(from) ;
			to->setTotalCost(to->getCostSoFar() + costRemaining) ;

			// if to was on closed list, remove it from the list
			if (to->getClosed())
				to->removeFromClosedList( d->thePathfinder->m_closedList );

			// if the to was already on the open list, remove it so it can be re-inserted in order
			if (to->getOpen())
				to->removeFromOpenList( d->thePathfinder->m_openList );

			// insert to in open list such that open list is sorted, smallest total path cost first
			to->putOnSortedOpenList( d->thePathfinder->m_openList );
	}

	return 0;	// keep going
}


Int Pathfinder::examineNeighboringCells(PathfindCell *parentCell, PathfindCell *goalCell, const LocomotorSet& locomotorSet,
																				 Bool isHuman, Bool centerInCell, Int radius, const ICoord2D &startCellNdx,
																				 const Object *obj, Int attackDistance)
{
		Bool canPathThroughUnits = false;
		if (obj && obj->getAIUpdateInterface()) {
			canPathThroughUnits = obj->getAIUpdateInterface()->canPathThroughUnits();
		}
		Bool isCrusher = obj ? obj->getCrusherLevel() > 0 : false;
		if (attackDistance==NO_ATTACK && !m_isTunneling && !locomotorSet.isDownhillOnly() && goalCell) {
			ExamineCellsStruct info;
			info.thePathfinder = this;
			info.theLoco = &locomotorSet;
			info.centerInCell = centerInCell;
			info.radius = radius;
			info.obj = obj;
			info.isHuman = isHuman;
			info.goalCell = goalCell;
			ICoord2D start, end;
			start.x = parentCell->getXIndex();
			start.y = parentCell->getYIndex();
			end.x = goalCell->getXIndex();
			end.y = goalCell->getYIndex();
			iterateCellsAlongLine(start, end, parentCell->getLayer(), examineCellsCallback, &info);
		}

		Int cellCount = 0;
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
			if (!newCell)
				continue;

			Bool notZonePassable = false;
			if ((newCell->getLayer()==LAYER_GROUND) && !m_zoneManager.isPassable(newCellCoord.x, newCellCoord.y)) {
				notZonePassable = true;
			}

			// check if new cell is in logical map.	(computer can move off logical map)
			if (isHuman && checkCellOutsideExtents(newCellCoord))
				continue;

			// check if this neighbor cell is already on the open (waiting to be tried)
			// or closed (already tried) lists
			if ( newCell->hasInfo() && (newCell->getOpen() || newCell->getClosed()) )
				continue;

			if (i>=firstDiagonal) {
				// make sure one of the adjacent sides is open.
				if (!neighborFlags[adjacent[i-4]] && !neighborFlags[adjacent[i-3]]) {
					continue;
				}
			}

			// do the gravity check here
			if ( locomotorSet.isDownhillOnly() )
			{
				Coord3D fromPos;
				fromPos.x = parentCell->getXIndex() * PATHFIND_CELL_SIZE_F ;
				fromPos.y = parentCell->getYIndex() * PATHFIND_CELL_SIZE_F ;
				fromPos.z = TheTerrainLogic->getGroundHeight(fromPos.x , fromPos.y);

				Coord3D toPos;
				toPos.x = newCellCoord.x * PATHFIND_CELL_SIZE_F ;
				toPos.y = newCellCoord.y * PATHFIND_CELL_SIZE_F ;
				toPos.z = TheTerrainLogic->getGroundHeight(toPos.x , toPos.y);

				if ( fromPos.z < toPos.z )
					continue;
			}

			Bool movementValid = validMovementPosition(isCrusher, locomotorSet.getValidSurfaces(), newCell, parentCell);
			Bool dozerHack = false;
			if (!movementValid && obj->isKindOf(KINDOF_DOZER) && newCell->getType() == PathfindCell::CELL_OBSTACLE) {
				Object* obstacle = TheGameLogic->findObjectByID(newCell->getObstacleID());
				if (obstacle && !(obj->getRelationship(obstacle) == ENEMIES)) {
					movementValid = true;
					dozerHack = true;
				}
			}

			if (!movementValid && !m_isTunneling) {
				continue;
			}

			if (!dozerHack)
				neighborFlags[i] = true;

			TCheckMovementInfo info;
			info.cell = newCellCoord;
			info.layer = parentCell->getLayer();
			info.centerInCell = centerInCell;
			info.radius = radius;
			info.considerTransient = false;
			info.acceptableSurfaces = locomotorSet.getValidSurfaces();
			Int dx = newCellCoord.x-startCellNdx.x;
			Int dy = newCellCoord.y-startCellNdx.y;
			if (dx<0) dx = -dx;
			if (dy<0) dy = -dy;
			if (dx>1+radius) info.considerTransient = false;
			if (dy>1+radius) info.considerTransient = false;
			if (!checkForMovement(obj, info) || info.enemyFixed) {
				if (!m_isTunneling) {
					continue;
				}
				movementValid = false;
			}

			if (movementValid && !newCell->getPinched()) {
				//Note to self - only turn off tunneling after check for movement.jba.
				m_isTunneling = false;
			}

			if (!newCell->hasInfo()) {
				if (!newCell->allocateInfo(newCellCoord)) {
					// Out of cells for pathing...
 					return cellCount;
				}
				cellCount++;
			}

			newCostSoFar = newCell->costSoFar( parentCell );
			if (info.allyMoving && dx<10 && dy<10) {
				newCostSoFar += 3*COST_DIAGONAL;
			}
			newCostSoFar += getPhase3LaneBiasCost(obj, startCellNdx, goalCell, newCellCoord, info);

			if (newCell->getType() == PathfindCell::CELL_CLIFF && !newCell->getPinched() ) {
				Coord3D fromPos;
				fromPos.x = parentCell->getXIndex() * PATHFIND_CELL_SIZE_F ;
				fromPos.y = parentCell->getYIndex() * PATHFIND_CELL_SIZE_F ;
				fromPos.z = TheTerrainLogic->getGroundHeight(fromPos.x , fromPos.y);

				Coord3D toPos;
				toPos.x = newCellCoord.x * PATHFIND_CELL_SIZE_F ;
				toPos.y = newCellCoord.y * PATHFIND_CELL_SIZE_F ;
				toPos.z = TheTerrainLogic->getGroundHeight(toPos.x , toPos.y);

				if ( fabs(fromPos.z - toPos.z)<PATHFIND_CELL_SIZE_F) {
					newCostSoFar += 7*COST_DIAGONAL;
				}
			} else if (newCell->getPinched()) {
				newCostSoFar += COST_ORTHOGONAL;
			}

			newCell->setBlockedByAlly(false);
			if (info.allyFixedCount>0) {
#if RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING
				newCostSoFar += 3*COST_DIAGONAL*info.allyFixedCount;
#else
				newCostSoFar += 3*COST_DIAGONAL;
#endif
				if (!canPathThroughUnits)
					newCell->setBlockedByAlly(true);
			}

			Int costRemaining = 0;
			if (goalCell) {
				if (attackDistance == NO_ATTACK)  {
					costRemaining = newCell->costToGoal( goalCell );
				}	else {
					dx = newCellCoord.x - goalCell->getXIndex();
					dy = newCellCoord.y - goalCell->getYIndex();
					costRemaining = COST_ORTHOGONAL*sqrt(dx*dx + dy*dy);
					costRemaining -= attackDistance/2;
					if (costRemaining<0)
						costRemaining=0;
					if (info.allyGoal) {
						if (obj->isKindOf(KINDOF_VEHICLE)) {
							newCostSoFar += 3*COST_ORTHOGONAL;
						}	else {
							// Infantry can pass through infantry.
							newCostSoFar += COST_ORTHOGONAL;
						}
					}
				}
			}

			if (notZonePassable) {
				newCostSoFar += 100*COST_ORTHOGONAL;
			}

			if (newCell->getType()==PathfindCell::CELL_OBSTACLE) {
				newCostSoFar += 100*COST_ORTHOGONAL;
			}

			if (m_isTunneling) {
				if (!validMovementPosition( isCrusher, locomotorSet.getValidSurfaces(), newCell, parentCell )) {
					newCostSoFar += 10*COST_ORTHOGONAL;
				}
			}

			newCell->setCostSoFar(newCostSoFar);
			// keep track of path we're building - point back to cell we moved here from
			newCell->setParentCell(parentCell) ;
			if (m_isTunneling) {
				costRemaining = 0; // find the closest valid cell.
			}
			newCell->setTotalCost(newCell->getCostSoFar() + costRemaining) ;

			// if newCell was on closed list, remove it from the list
			if (newCell->getClosed())
				newCell->removeFromClosedList( m_closedList );

			// if the newCell was already on the open list, remove it so it can be re-inserted in order
			if (newCell->getOpen())
				newCell->removeFromOpenList( m_openList );

			// insert newCell in open list such that open list is sorted, smallest total path cost first
			newCell->putOnSortedOpenList( m_openList );
		}
	return cellCount;
}


/**
 * Find a short, valid path between given locations.
 * Uses A* algorithm.
 */
Path *Pathfinder::findPath( Object *obj, const LocomotorSet& locomotorSet, const Coord3D *from,
													 const Coord3D *rawTo)
{
	if (!clientSafeQuickDoesPathExist(locomotorSet, from, rawTo)) {
		return nullptr;
	}
	Bool isHuman = true;
	if (obj && obj->getControllingPlayer() && (obj->getControllingPlayer()->getPlayerType()==PLAYER_COMPUTER)) {
		isHuman = false; // computer gets to cheat.
	}

	m_zoneManager.clearPassableFlags();
	Path *hPat = findHierarchicalPath(isHuman, locomotorSet, from, rawTo, false);
	if (hPat) {
		deleteInstance(hPat);
	}	else {
		m_zoneManager.setAllPassable();
	}

	Path *pat = internalFindPath(obj, locomotorSet, from, rawTo);
	if (pat!=nullptr) {
		return pat;
	}

	return nullptr;
}
/**
 * Find a short, valid path between given locations.
 * Uses A* algorithm.
 */
Path *Pathfinder::internalFindPath( Object *obj, const LocomotorSet& locomotorSet, const Coord3D *from,
													 const Coord3D *rawTo)
{
	PerfTrace::ScopedPathTimer timer(getPerfTraceFrame(), PerfTrace::PATH_TIMER_ASTAR_EXPAND);
	//CRCDEBUG_LOG(("Pathfinder::findPath()"));
#ifdef INTENSE_DEBUG
	DEBUG_LOG(("internal find path..."));
#endif

#ifdef DEBUG_LOGGING
	Int startTimeMS = ::GetTickCount();
#endif
	Bool centerInCell = true;
	Int radius = 0;
	if (obj) {
		getRadiusAndCenter(obj, radius, centerInCell);
	}

	Bool isHuman = true;
	if (obj && obj->getControllingPlayer() && (obj->getControllingPlayer()->getPlayerType()==PLAYER_COMPUTER)) {
		isHuman = false; // computer gets to cheat.
	}

	if (rawTo->x == 0.0f && rawTo->y == 0.0f) {
		DEBUG_LOG(("Attempting pathfind to 0,0, generally a bug."));
		return nullptr;
	}
	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));
	if (m_isMapReady == false) {
		return nullptr;
	}

	Coord3D adjustTo = *rawTo;
	Coord3D *to = &adjustTo;
	Coord3D clipFrom = *from;
	clip(&clipFrom, &adjustTo);

	if (!centerInCell) {
		adjustTo.x += PATHFIND_CELL_SIZE_F/2;
		adjustTo.y += PATHFIND_CELL_SIZE_F/2;
	}

	m_isTunneling = false;

	PathfindLayerEnum destinationLayer = TheTerrainLogic->getLayerForDestination(to);
	// determine goal cell
	PathfindCell *goalCell = getCell( destinationLayer, to );
	if (goalCell == nullptr) {
		return nullptr;
	}

	ICoord2D cell;
	worldToCell( to, &cell );

	if (!checkDestination(obj, cell.x, cell.y, destinationLayer, radius, centerInCell)) {
		return nullptr;
	}
	// determine start cell
	ICoord2D startCellNdx;
	worldToCell(&clipFrom, &startCellNdx);
	PathfindLayerEnum layer = LAYER_GROUND;
	if (obj) {
		layer = obj->getLayer();
	}
	PathfindCell *parentCell = getClippedCell( layer,&clipFrom );
	if (parentCell == nullptr) {
		return nullptr;
	}

	ICoord2D pos2d;
	worldToCell(to, &pos2d);
	if (!goalCell->allocateInfo(pos2d)) {
		return nullptr;
	}
	if (parentCell!=goalCell) {
		worldToCell(&clipFrom, &pos2d);
		if (!parentCell->allocateInfo(pos2d)) {
			goalCell->releaseInfo();
			return nullptr;
		}
	}
	//
	// Determine if this pathfind is "tunneling" or not.
	// A tunneling pathfind starts from within an obstacle, and is allowed
	// to ignore obstacle cells until it reaches a cell that is no longer
	// classified as an obstacle.  At that point, the pathfind behaves normally.
	//
	m_isTunneling = parentCell->getType() == PathfindCell::CELL_OBSTACLE;

	Int zone1, zone2;
	Bool isCrusher = obj ? obj->getCrusherLevel() > 0 : false;
	zone1 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), isCrusher, parentCell->getZone());
	zone2 =  m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), isCrusher, goalCell->getZone());

	if ( (layer==LAYER_WALL && zone1 == 0) || (destinationLayer==LAYER_WALL && zone2 == 0) ) {
#if RETAIL_COMPATIBLE_PATHFINDING
		if (s_useFixedPathfinding)
#endif
		{
			goalCell->releaseInfo();
			parentCell->releaseInfo();
		}
		return nullptr;
	}

	if (goalCell->isObstaclePresent(m_ignoreObstacleID) || m_isTunneling) {
		// Use terrain zones instead of building zones, since we are moving into or out of a building.
		zone2 = m_zoneManager.getEffectiveTerrainZone(zone2);
		zone2 =  m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), isCrusher, zone2);
		zone1 = m_zoneManager.getEffectiveTerrainZone(zone1);
		zone1 = m_zoneManager.getEffectiveZone(locomotorSet.getValidSurfaces(), isCrusher, zone1);
	}

	//DEBUG_LOG(("Zones %d to %d", zone1, zone2));

	if ( zone1 != zone2) {
		//DEBUG_LOG(("Intense Debug Info - Pathfind Zone screen failed-cannot reach desired location."));
		goalCell->releaseInfo();
		parentCell->releaseInfo();
		return nullptr;
	}

	// sanity check - if destination is invalid, can't path there
	if (!validMovementPosition( isCrusher, destinationLayer, locomotorSet, to ))	{
		m_isTunneling = false;
		goalCell->releaseInfo();
		parentCell->releaseInfo();
		return nullptr;
	}

	// sanity check - if source is invalid, we have to cheat
	if (!validMovementPosition( isCrusher, layer, locomotorSet, from ))	{
		// somehow we got to an impassable location.
		m_isTunneling = true;
	}

	parentCell->startPathfind(goalCell);

	beginOpenSearch(parentCell);

	m_closedList.reset();

	Int cellCount = 0;

	Int requestBudget = m_requestCellBudget > 0 ? m_requestCellBudget : PATHFIND_CELLS_PER_REQUEST_EXPLICIT;

	while (hasOpenCells())
	{
		if (m_cumulativeCellsAllocated + cellCount >= PATHFIND_CELLS_PER_FRAME)
		{
			break;
		}
		if (cellCount >= requestBudget)
		{
			break;
		}

		parentCell = popNextOpenCell();
		if (!parentCell) break;

		if (parentCell == goalCell)
		{
			// success - found a path to the goal
			Bool show = TheGlobalData->m_debugAI==AI_DEBUG_PATHS;
#ifdef INTENSE_DEBUG
			DEBUG_LOG(("internal find path SUCCESS..."));
			Int count = 0;
			if (cellCount>1000 && obj) {
				show = true;
				DEBUG_LOG(("cells %d obj %s %x from (%f,%f) to(%f, %f)", count, obj->getTemplate()->getName().str(), obj, from->x, from->y, to->x, to->y));
#ifdef STATE_MACHINE_DEBUG
				if( obj->getAIUpdateInterface() )
				{
					DEBUG_LOG(("State %s",  obj->getAIUpdateInterface()->getCurrentStateName().str()));
				}
#endif
				TheScriptEngine->AppendDebugMessage("Big path", false);
			}
#endif
			if (show)
				debugShowSearch(true);

			m_isTunneling = false;
			// construct and return path
			Path *path =  buildActualPath( obj, locomotorSet.getValidSurfaces(), from, goalCell, centerInCell, false );
#if RETAIL_COMPATIBLE_PATHFINDING
			if (!s_useFixedPathfinding)
			{
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

		cellCount += examineNeighboringCells(parentCell, goalCell, locomotorSet, isHuman, centerInCell, radius, startCellNdx, obj, NO_ATTACK);

		if (m_useHeapOpenList)
		{
			PerfTrace::NoteOpenListPeak(getPerfTraceFrame(), m_openHeap.size());
		}

	}

	// failure - goal cannot be reached
#if defined(RTS_DEBUG)
#ifdef INTENSE_DEBUG
	DEBUG_LOG(("internal find path FAILURE..."));
#endif
	if (TheGlobalData->m_debugAI == AI_DEBUG_PATHS)
	{
		extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
 		RGBColor color;
		color.blue = 0;
		color.red = color.green = 1;
		addIcon(nullptr, 0, 0, color);
		debugShowSearch(false);
		Coord3D pos;
		pos = *from;
		pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
		addIcon(&pos, 3*PATHFIND_CELL_SIZE_F, 600, color);
		pos = *to;
		pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
		addIcon(&pos, 3*PATHFIND_CELL_SIZE_F, 600, color);
		Real dx, dy;
		dx = from->x - to->x;
		dy = from->y - to->y;

		Int count = sqrt(dx*dx+dy*dy)/(PATHFIND_CELL_SIZE_F/2);
		if (count<2) count = 2;
		Int i;
		color.green = 0;
		for (i=1; i<count; i++) {
			pos.x = from->x + (to->x-from->x)*i/count;
			pos.y = from->y + (to->y-from->y)*i/count;
			pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
			addIcon(&pos, PATHFIND_CELL_SIZE_F/2, 60, color);

		}
	}

	if (obj) {
#ifdef DUMP_PERF_STATS
		TheGameLogic->incrementOverallFailedPathfinds();
#endif
#ifdef STATE_MACHINE_DEBUG
		if( obj->getAIUpdateInterface() )
		{
			DEBUG_LOG(("state %s", obj->getAIUpdateInterface()->getCurrentStateName().str()));
		}
#endif
	}
#endif

#ifdef DEBUG_LOGGING
	if (obj)
	{
		Bool valid;
		valid = validMovementPosition( isCrusher, obj->getLayer(), locomotorSet, to ) ;

		DEBUG_LOG(("%d Pathfind failed from (%f,%f) to (%f,%f), OV %d --", TheGameLogic->getFrame(), from->x, from->y, to->x, to->y, valid));
		DEBUG_LOG(("Unit '%s', time %f, cells %d", obj->getTemplate()->getName().str(), (::GetTickCount()-startTimeMS)/1000.0f,cellCount));
	}
#endif

	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding)
	{
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

/**
 * Checks to see if there is enough path width at this cell for ground
 * movement.  Returns the width available.
 */
Int Pathfinder::clearCellForDiameter(Bool crusher, Int cellX, Int cellY, PathfindLayerEnum layer, Int pathDiameter)
{
	Int radius = pathDiameter/2;
	Int numCellsAbove = radius;
	if (radius==0) numCellsAbove++;
	Int i, j;
	Bool clear = true;
	Bool cutCorners = false;
	if (radius>1) {
		cutCorners = true;
		// We remove the outside corner cells from the check.
	}
	for (i=cellX-radius; i<cellX+numCellsAbove; i++) {
		Bool xMinOrMax = (i==cellX-radius) || (i==cellX+numCellsAbove-1);
		for (j=cellY-radius; j<cellY+numCellsAbove; j++) {
			Bool yMinOrMax = (j==cellY-radius) || (j==cellY+numCellsAbove-1);
			if (xMinOrMax && yMinOrMax && cutCorners) {
				continue; // this is an outside corner cell, and we are cutting corners. jba. :)
			}
			PathfindCell	*cell = getCell(layer, i, j);
			if (cell) {
				if (cell->getType() != PathfindCell::CELL_CLEAR) {
					if (cell->getType() == PathfindCell::CELL_OBSTACLE) {
						if (cell->isObstacleFence()) {
							if (!crusher) {
								clear = false;
							}
						} else {
							clear = false;
						}
					} else {
						clear = false;
					}
				}
				if (cell->getFlags() == PathfindCell::UNIT_PRESENT_FIXED && pathDiameter>=2) {
					Object *obj = TheGameLogic->findObjectByID(cell->getPosUnit());
					if (obj) {
						if (crusher) {
							if (obj->getCrushableLevel()>1) {
								clear = false;
							}
						} else {
							if (obj->getCrushableLevel()>0) {
								clear = false;
							}
						}
					}
				}
			} else {
				return false; // off the map.
			}
			if (!clear) break;
		}
	}
	if (clear) {
		if (radius==0) return 1;
		return 2*radius;
	}
	if (pathDiameter < 2) return 0;
	return clearCellForDiameter(crusher, cellX, cellY, layer, pathDiameter-2);
}

/**
 * Work backwards from goal cell to construct final path.
 */
Path *Pathfinder::buildGroundPath(Bool isCrusher, const Coord3D *fromPos, PathfindCell *goalCell, Bool center, Int pathDiameter )
{
	DEBUG_ASSERTCRASH( goalCell, ("Pathfinder::buildActualPath: goalCell == nullptr") );

	Path *path = newInstance(Path);

	prependCells(path, fromPos, goalCell, center);

	// cleanup the path by checking line of sight
	path->optimizeGroundPath( isCrusher, pathDiameter );


#if defined(RTS_DEBUG)
	if (TheGlobalData->m_debugAI==AI_DEBUG_GROUND_PATHS)
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
Path *Pathfinder::buildHierarchicalPath( const Coord3D *fromPos, PathfindCell *goalCell )
{
	DEBUG_ASSERTCRASH( goalCell, ("Pathfinder::buildHierarchicalPath: goalCell == nullptr") );

	Path *path = newInstance(Path);

	prependCells(path, fromPos, goalCell, true);

#if !(RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING)
	// Expand the hierarchical path around the starting point. jba [8/24/2003]
	// This allows the unit to get around friendly units that may be near it.
	Coord3D pos = *path->getFirstNode()->getPosition();
	Coord3D minPos = pos;
	minPos.x -= PathfindZoneManager::ZONE_BLOCK_SIZE*PATHFIND_CELL_SIZE_F;
	minPos.y -= PathfindZoneManager::ZONE_BLOCK_SIZE*PATHFIND_CELL_SIZE_F;
	Coord3D maxPos = pos;
	maxPos.x += PathfindZoneManager::ZONE_BLOCK_SIZE*PATHFIND_CELL_SIZE_F;
	maxPos.y += PathfindZoneManager::ZONE_BLOCK_SIZE*PATHFIND_CELL_SIZE_F;
	ICoord2D cellNdxMin, cellNdxMax;
	worldToCell(&minPos, &cellNdxMin);
	worldToCell(&maxPos, &cellNdxMax);
	Int i, j;
	for (i=cellNdxMin.x; i<=cellNdxMax.x; i++) {
		for (j=cellNdxMin.y; j<=cellNdxMax.y; j++) {
			m_zoneManager.setPassable(i, j, true);
		}
	}
#endif

#if defined(RTS_DEBUG)
	if (TheGlobalData->m_debugAI==AI_DEBUG_PATHS)
	{
		extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
 		RGBColor color;
		color.blue = 0;
		color.red = color.green = 1;
		Coord3D pos;
		Int i;
		for (i=0; i<3; i++)
		for( PathNode *node = path->getFirstNode(); node; node = node->getNext() )
		{

			// create objects to show path - they decay

			pos = *node->getPosition();
			color.red = 1;
			color.green = 0.4f;
			if (node->getLayer() != LAYER_GROUND) {
				color.red = 0;
			}
			addIcon(&pos, PATHFIND_CELL_SIZE_F, 200, color);
		}
		setDebugPath(path);
	}
#endif
	return path;
}


struct MADStruct
{
	Pathfinder					*thePathfinder;
	Object							*obj;
	ObjectID						ignoreID;
};

/*static*/ Int Pathfinder::moveAlliesDestinationCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	MADStruct* d = (MADStruct*)userData;
	if (to) {
		if (to->getPosUnit()==INVALID_ID) {
			return 0;
		}
		if (to->getPosUnit()==d->obj->getID()) {
			return 0;	// It's us.
		}
		if (to->getPosUnit()==d->ignoreID) {
			return 0;	 // It's the one we are ignoring.
		}
		Object *otherObj = TheGameLogic->findObjectByID(to->getPosUnit());
		if (otherObj==nullptr) return 0;
		if (d->obj->getRelationship(otherObj)!=ALLIES) {
			return 0;  // Only move allies.
		}
		if (otherObj && otherObj->getAI() && !otherObj->getAI()->isMoving()) {
#if !(RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING)
			//Kris: Patch 1.01 November 3, 2003
			//Black Lotus exploit fix -- moving while hacking.
			if( otherObj->testStatus( OBJECT_STATUS_IS_USING_ABILITY ) || otherObj->getAI()->isBusy() )
			{
				return 0; // Packing or unpacking objects for example
			}
#endif
			//DEBUG_LOG(("Moving ally"));
			otherObj->getAI()->aiMoveAwayFromUnit(d->obj, CMD_FROM_AI);
		}
	}

	return 0;	// keep going
}

void Pathfinder::moveAlliesAwayFromDestination(Object *obj,const Coord3D& destination)
{
	MADStruct info;
	info.obj = obj;
	info.ignoreID = obj->getAI()->getIgnoredObstacleID();
	info.thePathfinder = this;
	PathfindLayerEnum layer = obj->getLayer();
	if (layer==LAYER_GROUND) {
		layer = TheTerrainLogic->getLayerForDestination(&destination);
	}
	iterateCellsAlongLine(*obj->getPosition(), destination, layer, moveAlliesDestinationCallback, &info);

}


struct GroundCellsStruct
{
	Pathfinder					*thePathfinder;
	Bool								centerInCell;
	Int									pathDiameter;
	PathfindCell				*goalCell;
	Bool								crusher;
};

/*static*/ Int Pathfinder::groundCellsCallback(Pathfinder* pathfinder, PathfindCell* from, PathfindCell* to, Int to_x, Int to_y, void* userData)
{
	GroundCellsStruct* d = (GroundCellsStruct*)userData;
	if (from && to) {
			if (to->hasInfo()) {
				if (to->getOpen() || to->getClosed())
				{
					// already on one of the lists
					return 1; // abort.
				}
			}
			// See how wide the cell is.
			Int clearDiameter = d->thePathfinder->clearCellForDiameter(d->crusher, to_x, to_y, to->getLayer(), d->pathDiameter);
			if (clearDiameter != d->pathDiameter) {
				return 1;
			}
			ICoord2D newCellCoord;
			newCellCoord.x = to_x;
			newCellCoord.y = to_y;
			if (!to->allocateInfo(newCellCoord)) {
				// Out of cells for pathing...
 				return 1;
			}

			UnsignedInt newCostSoFar = from->getCostSoFar() + 0.5f*COST_ORTHOGONAL;
			to->setBlockedByAlly(false);

			Int costRemaining = 0;
			costRemaining = to->costToGoal( d->goalCell );
			to->setCostSoFar(newCostSoFar);
			// keep track of path we're building - point back to cell we moved here from
			to->setParentCell(from) ;
			to->setTotalCost(to->getCostSoFar() + costRemaining) ;

			// insert to in open list such that open list is sorted, smallest total path cost first
			to->putOnSortedOpenList( d->thePathfinder->m_openList );
	}

	return 0;	// keep going
}

/**
 * Find a short, valid path between given locations.
 * Uses A* algorithm.
 */
Path *Pathfinder::findGroundPath( const Coord3D *from,
													 const Coord3D *rawTo, Int pathDiameter, Bool crusher)
{
	//CRCDEBUG_LOG(("Pathfinder::findGroundPath()"));
#ifdef DEBUG_LOGGING
	Int startTimeMS = ::GetTickCount();
#endif
#ifdef INTENSE_DEBUG
	DEBUG_LOG(("Find ground path..."));
#endif
	Bool centerInCell = false;

	m_zoneManager.clearPassableFlags();
	Bool isHuman = true;

	Path *hPat = internal_findHierarchicalPath(isHuman, LOCOMOTORSURFACE_GROUND, from, rawTo, false, false);
	if (hPat) {
		deleteInstance(hPat);
	}	else {
		m_zoneManager.setAllPassable();
	}

	if (rawTo->x == 0.0f && rawTo->y == 0.0f) {
		DEBUG_LOG(("Attempting pathfind to 0,0, generally a bug."));
		return nullptr;
	}
	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));
	if (m_isMapReady == false) {
		return nullptr;
	}

	Coord3D adjustTo = *rawTo;
	Coord3D *to = &adjustTo;
	Coord3D clipFrom = *from;
	clip(&clipFrom, &adjustTo);

	m_isTunneling = false;

	PathfindLayerEnum destinationLayer = TheTerrainLogic->getLayerForDestination(to);

	ICoord2D cell;
	worldToCell( to, &cell );

	if (pathDiameter!=clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)) {
		Int offset=1;
		ICoord2D newCell;
		const Int MAX_OFFSET = 8;
		while (offset<MAX_OFFSET) {
			newCell = cell;
			cell.x += offset;
			if (clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)==pathDiameter) break;
			cell.y += offset;
			if (clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)==pathDiameter) break;
			cell.x -= offset;
			if (clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)==pathDiameter) break;
			cell.x -= offset;
			if (clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)==pathDiameter) break;
			cell.y -= offset;
			if (clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)==pathDiameter) break;
			cell.y -= offset;
			if (clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)==pathDiameter) break;
			cell.x += offset;
			if (clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)==pathDiameter) break;
			cell.x += offset;
			if (clearCellForDiameter(crusher, cell.x, cell.y, destinationLayer, pathDiameter)==pathDiameter) break;
			offset++;
			cell = newCell;
		}
		if (offset >= MAX_OFFSET) {
			return nullptr;
		}
	}

	// determine goal cell
	PathfindCell *goalCell = getCell( destinationLayer, cell.x, cell.y );
	if (goalCell == nullptr) {
		return nullptr;
	}
	if (!goalCell->allocateInfo(cell)) {
		return nullptr;
	}

	// determine start cell
	ICoord2D startCellNdx;
	PathfindLayerEnum layer = TheTerrainLogic->getLayerForDestination(from);
	PathfindCell *parentCell = getClippedCell( layer,&clipFrom );
	if (parentCell == nullptr) {
#if RETAIL_COMPATIBLE_PATHFINDING
		if (s_useFixedPathfinding)
#endif
		{
			goalCell->releaseInfo();
		}
		return nullptr;
	}
	if (parentCell!=goalCell) {
		worldToCell(&clipFrom, &startCellNdx);
		if (!parentCell->allocateInfo(startCellNdx)) {
			goalCell->releaseInfo();
			return nullptr;
		}
	}


	Int zone1, zone2;
	// m_isCrusher = false;
	zone1 = m_zoneManager.getEffectiveZone(LOCOMOTORSURFACE_GROUND, false, parentCell->getZone());
	zone2 =  m_zoneManager.getEffectiveZone(LOCOMOTORSURFACE_GROUND, false, goalCell->getZone());

	//DEBUG_LOG(("Zones %d to %d", zone1, zone2));

	if ( zone1 != zone2) {
		goalCell->releaseInfo();
		parentCell->releaseInfo();
		return nullptr;
	}
	parentCell->startPathfind(goalCell);

	// initialize "open" list to contain start cell
	beginOpenSearch(parentCell);

	// "closed" list is initially empty
	m_closedList.reset();

	// TheSuperHackers @fix helmutbuhler This was originally uninitialized and in the loop below.
#if RETAIL_COMPATIBLE_CRC
	UnsignedInt newCostSoFar = 0;
#endif

	//
	// Continue search until "open" list is empty, or
	// until goal is found.
	//
	Int cellCount = 0;
	while (hasOpenCells())
	{
		// take head cell off of open list - it has lowest estimated total path cost
		parentCell = popNextOpenCell();
		if (!parentCell) {
			break;
		}

		if (parentCell == goalCell)
		{
			// success - found a path to the goal
#ifdef INTENSE_DEBUG
	DEBUG_LOG((" time %d msec %d cells", (::GetTickCount()-startTimeMS), cellCount));
	DEBUG_LOG((" SUCCESS"));
#endif
#if defined(RTS_DEBUG)
			Bool show = TheGlobalData->m_debugAI==AI_DEBUG_GROUND_PATHS;
			if (show)
				debugShowSearch(true);
#endif
			m_isTunneling = false;
			// construct and return path
			Path *path =  buildGroundPath(crusher, from, goalCell, centerInCell, pathDiameter );
#if RETAIL_COMPATIBLE_PATHFINDING
			if (!s_useFixedPathfinding)
			{
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

		GroundCellsStruct info;
		info.thePathfinder = this;
		info.centerInCell = centerInCell;
		info.pathDiameter = pathDiameter;
		info.goalCell = goalCell;
		info.crusher = crusher;
		ICoord2D start, end;
		start.x = parentCell->getXIndex();
		start.y = parentCell->getYIndex();
		end.x = goalCell->getXIndex();
		end.y = goalCell->getYIndex();
		iterateCellsAlongLine(start, end, parentCell->getLayer(), groundCellsCallback, &info);

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

		// TheSuperHackers @fix Mauller 23/05/2025 Fixes uninitialized variable.
#if RETAIL_COMPATIBLE_CRC
		// newCostSoFar defined in outer block.
#else
		UnsignedInt newCostSoFar = 0;
#endif

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

			if ((newCell->getLayer()==LAYER_GROUND) && !m_zoneManager.isPassable(newCellCoord.x, newCellCoord.y)) {
				// check if we are within 3.
				Bool passable = false;
				if (m_zoneManager.clipIsPassable(newCellCoord.x+3, newCellCoord.y+3)) passable = true;
				if (m_zoneManager.clipIsPassable(newCellCoord.x-3, newCellCoord.y+3)) passable = true;
				if (m_zoneManager.clipIsPassable(newCellCoord.x+3, newCellCoord.y-3)) passable = true;
				if (m_zoneManager.clipIsPassable(newCellCoord.x-3, newCellCoord.y-3)) passable = true;
				if (!passable) continue;
			}

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
			Int clearDiameter = 0;
			if (newCell!=goalCell) {

				if (i>=firstDiagonal) {
					// make sure one of the adjacent sides is open.
					if (!neighborFlags[adjacent[i-4]] && !neighborFlags[adjacent[i-3]]) {
						continue;
					}
				}

				// See how wide the cell is.
				clearDiameter = clearCellForDiameter(crusher, newCellCoord.x, newCellCoord.y, newCell->getLayer(), pathDiameter);
				if (newCell->getType() != PathfindCell::CELL_CLEAR) {
					continue;
				}
				if (newCell->getPinched()) {
					continue;
				}
				neighborFlags[i] = true;

				if (!newCell->allocateInfo(newCellCoord)) {
					// Out of cells for pathing...
 					continue;
				}
				cellCount++;

#if RETAIL_COMPATIBLE_CRC
				// TheSuperHackers @fix helmutbuhler 11/06/2025 The indentation was wrong on retail here.
				newCostSoFar = newCell->costSoFar( parentCell );
				if (clearDiameter<pathDiameter) {
					int delta = pathDiameter-clearDiameter;
					newCostSoFar += 0.6f*(delta*COST_ORTHOGONAL);
				}
				newCell->setBlockedByAlly(false);
			}
#else
			}
			newCostSoFar = newCell->costSoFar( parentCell );
			if (clearDiameter<pathDiameter) {
				int delta = pathDiameter-clearDiameter;
				newCostSoFar += 0.6f*(delta*COST_ORTHOGONAL);
			}
			newCell->setBlockedByAlly(false);
#endif
			Int costRemaining = 0;
			costRemaining = newCell->costToGoal( goalCell );

			// check if this neighbor cell is already on the open (waiting to be tried)
			// or closed (already tried) lists
			if (onList)
			{
				// already on one of the lists - if existing costSoFar is less,
				// the new cell is on a longer path, so skip it
				if (newCell->getCostSoFar() <= newCostSoFar)
					continue;
			}
			newCell->setCostSoFar(newCostSoFar);
			// keep track of path we're building - point back to cell we moved here from
			newCell->setParentCell(parentCell) ;
			newCell->setTotalCost(newCell->getCostSoFar() + costRemaining) ;

			// if newCell was on closed list, remove it from the list
			if (newCell->getClosed())
				newCell->removeFromClosedList( m_closedList );

			// if the newCell was already on the open list, remove it so it can be re-inserted in order
			if (newCell->getOpen())
				newCell->removeFromOpenList( m_openList );

			// insert newCell in open list such that open list is sorted, smallest total path cost first
			newCell->putOnSortedOpenList( m_openList );
		}
	}
	// failure - goal cannot be reached
#ifdef INTENSE_DEBUG
	DEBUG_LOG((" FAILURE"));
#endif
#if defined(RTS_DEBUG)
	if (TheGlobalData->m_debugAI)
	{
		extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
 		RGBColor color;
		color.blue = 0;
		color.red = color.green = 1;
		addIcon(nullptr, 0, 0, color);
		debugShowSearch(false);
		Coord3D pos;
		pos = *from;
		pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
		addIcon(&pos, 3*PATHFIND_CELL_SIZE_F, 600, color);
		pos = *to;
		pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
		addIcon(&pos, 3*PATHFIND_CELL_SIZE_F, 600, color);
		Real dx, dy;
		dx = from->x - to->x;
		dy = from->y - to->y;

		Int count = sqrt(dx*dx+dy*dy)/(PATHFIND_CELL_SIZE_F/2);
		if (count<2) count = 2;
		Int i;
		color.green = 0;
		for (i=1; i<count; i++) {
			pos.x = from->x + (to->x-from->x)*i/count;
			pos.y = from->y + (to->y-from->y)*i/count;
			pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
			addIcon(&pos, PATHFIND_CELL_SIZE_F/2, 60, color);

		}
	}
#endif

	DEBUG_LOG(("%d FindGroundPath failed from (%f,%f) to (%f,%f) --", TheGameLogic->getFrame(), from->x, from->y, to->x, to->y));
	DEBUG_LOG(("time %f", (::GetTickCount()-startTimeMS)/1000.0f));

#ifdef DUMP_PERF_STATS
	TheGameLogic->incrementOverallFailedPathfinds();
#endif
	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding)
	{
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

/**
 * Find a short, valid path between given locations.
 * Uses A* algorithm.
 */
void Pathfinder::processHierarchicalCell( const ICoord2D &scanCell, const ICoord2D &delta, PathfindCell *parentCell,
																				 PathfindCell *goalCell, zoneStorageType parentZone,
																				 zoneStorageType *examinedZones, Int &numExZones,
																				 Bool crusher, Int &cellCount)
{
	if (scanCell.x<m_extent.lo.x || scanCell.x>m_extent.hi.x ||
		scanCell.y<m_extent.lo.y || scanCell.y>m_extent.hi.y) {
		return;
	}
#if !(RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING)
	if (parentZone == PathfindZoneManager::UNINITIALIZED_ZONE) {
		return;
	}
#endif
	if (parentZone == m_zoneManager.getBlockZone(LOCOMOTORSURFACE_GROUND,
		crusher, scanCell.x, scanCell.y, m_map)) {
		PathfindCell *newCell = getCell(LAYER_GROUND, scanCell.x, scanCell.y);
#if RTS_GENERALS && RETAIL_COMPATIBLE_PATHFINDING
		if (newCell->hasInfo() && (newCell->getOpen() || newCell->getClosed())) return; // already looked at this one.
#else
		if( !newCell->hasInfo() )
		{
 			return;
		}

		if( newCell->getOpen() || newCell->getClosed() )
			return; // already looked at this one.
#endif

		ICoord2D adjacentCell = scanCell;
		//DEBUG_ASSERTCRASH(parentZone==newCell->getZone(), ("Different zones?"));
		if (parentZone!=newCell->getZone()) return;
		adjacentCell.x += delta.x;
		adjacentCell.y += delta.y;
		if (adjacentCell.x<m_extent.lo.x || adjacentCell.x>m_extent.hi.x ||
			adjacentCell.y<m_extent.lo.y || adjacentCell.y>m_extent.hi.y) {
			return;
		}
		PathfindCell *adjNewCell = getCell(LAYER_GROUND, adjacentCell.x, adjacentCell.y);
		if (adjNewCell->hasInfo() && (adjNewCell->getOpen() || adjNewCell->getClosed())) return; // already looked at this one.
		zoneStorageType parentGlobalZone = m_zoneManager.getEffectiveZone(LOCOMOTORSURFACE_GROUND, crusher, parentZone);

		/// @todo - somehow out of bounds or bogus newZone.
		zoneStorageType newZone = m_zoneManager.getBlockZone(LOCOMOTORSURFACE_GROUND,
							crusher, adjacentCell.x, adjacentCell.y, m_map);
		zoneStorageType newGlobalZone = m_zoneManager.getEffectiveZone(LOCOMOTORSURFACE_GROUND, crusher, newZone);
		if (newGlobalZone != parentGlobalZone) {
			return; // can't step over. jba.
		}
		Int j;
		Bool found=false;
		for (j=0; j<numExZones; j++) {
			if (examinedZones[j] == newZone) {
				found = true;
				break;
			}
		}
		if (found) {
			return;
		}

		newCell->allocateInfo(scanCell);
#if RETAIL_COMPATIBLE_PATHFINDING
		if (!s_useFixedPathfinding)
		{
			if (!newCell->getClosed() && !newCell->getOpen()) {
				newCell->putOnClosedList(m_closedList);
			}
		}
		else
#endif
		{
			if (newCell->hasInfo() && !newCell->getClosed() && !newCell->getOpen()) {
				newCell->putOnClosedList(m_closedList);
			}
		}

		adjNewCell->allocateInfo(adjacentCell);
		if( adjNewCell->hasInfo() )
		{

			cellCount++;
			Int curCost = adjNewCell->costToHierGoal(parentCell);
			Int remCost = adjNewCell->costToHierGoal(goalCell);
			if (adjNewCell->getPinched() || newCell->getPinched()) {
				curCost += 2*COST_ORTHOGONAL;
			}	else {
				examinedZones[numExZones] = newZone;
				numExZones++;
			}

			adjNewCell->setCostSoFar(parentCell->getCostSoFar() + curCost);
			adjNewCell->setTotalCost(adjNewCell->getCostSoFar()+remCost);
			adjNewCell->setParentCellHierarchical(parentCell);
			// insert newCell in open list such that open list is sorted, smallest total path cost first
			adjNewCell->putOnSortedOpenList( m_openList );
		}

	}
}


/**
 * Find a short, valid path between given locations.
 * Uses A* algorithm.
 */
Path *Pathfinder::findHierarchicalPath( Bool isHuman, const LocomotorSet& locomotorSet, const Coord3D *from,
													 const Coord3D *to, Bool crusher)
{
	return internal_findHierarchicalPath(isHuman, locomotorSet.getValidSurfaces(), from, to, crusher, FALSE);
}


/**
 * Find a short, valid path between given locations.
 * Uses A* algorithm.
 */
Path *Pathfinder::findClosestHierarchicalPath( Bool isHuman, const LocomotorSet& locomotorSet, const Coord3D *from,
													 const Coord3D *to, Bool crusher)
{
	return internal_findHierarchicalPath(isHuman, locomotorSet.getValidSurfaces(), from, to, crusher, TRUE);
}



/**
 * Find a short, valid path between given locations.
 * Uses A* algorithm.
 */
Path *Pathfinder::internal_findHierarchicalPath( Bool isHuman, const LocomotorSurfaceTypeMask locomotorSurface, const Coord3D *from,
													 const Coord3D *rawTo, Bool crusher, Bool closestOK)
{
	//CRCDEBUG_LOG(("Pathfinder::findGroundPath()"));
#ifdef DEBUG_LOGGING
	Int startTimeMS = ::GetTickCount();
#endif

	if (rawTo->x == 0.0f && rawTo->y == 0.0f) {
		DEBUG_LOG(("Attempting pathfind to 0,0, generally a bug."));
		return nullptr;
	}
	DEBUG_ASSERTCRASH(m_openList.empty() && m_closedList.empty(), ("Dangling lists."));
	if (m_isMapReady == false) {
		return nullptr;
	}

	Coord3D adjustTo = *rawTo;
	Coord3D *to = &adjustTo;
	Coord3D clipFrom = *from;
	clip(&clipFrom, &adjustTo);

	m_isTunneling = false;

	PathfindLayerEnum destinationLayer = TheTerrainLogic->getLayerForDestination(to);

	ICoord2D cell;
	worldToCell( to, &cell );

	// determine goal cell
	PathfindCell *goalCell = getCell( destinationLayer, cell.x, cell.y );
	if (!goalCell) {
		return nullptr;
	}

	if (!goalCell->allocateInfo(cell)) {
		return nullptr;
	}

	// determine start cell
	ICoord2D startCellNdx;
	PathfindLayerEnum layer = TheTerrainLogic->getLayerForDestination(from);
	PathfindCell *parentCell = getClippedCell( layer,&clipFrom );
	if (!parentCell) {
		return nullptr;
	}

	if (parentCell!=goalCell) {
		worldToCell(&clipFrom, &startCellNdx);
		if (!parentCell->allocateInfo(startCellNdx)) {
			goalCell->releaseInfo();
			return nullptr;
		}
	}

	Int zone1, zone2;
	// m_isCrusher = false;
	zone1 = m_zoneManager.getEffectiveZone(locomotorSurface, false, parentCell->getZone());
	zone2 =  m_zoneManager.getEffectiveZone(locomotorSurface, false, goalCell->getZone());

	if ( zone1 != zone2) {
		goalCell->releaseInfo();
		parentCell->releaseInfo();
		return nullptr;
	}

	parentCell->startPathfind(goalCell);

	// "closed" list is initially empty
	m_closedList.reset();

	Int cellCount = 0;

	zoneStorageType goalBlockZone;
	ICoord2D goalBlockNdx;
	if (goalCell->getLayer()==LAYER_GROUND) {
		goalBlockZone = m_zoneManager.getBlockZone(locomotorSurface,
			crusher, goalCell->getXIndex(), goalCell->getYIndex(), m_map);

		goalBlockNdx.x = goalCell->getXIndex()/PathfindZoneManager::ZONE_BLOCK_SIZE;
		goalBlockNdx.y = goalCell->getYIndex()/PathfindZoneManager::ZONE_BLOCK_SIZE;
	}	else {
		goalBlockZone = goalCell->getZone();
		goalBlockNdx.x = -1;
		goalBlockNdx.y = -1;
	}

	beginOpenSearch(parentCell);

	if (parentCell->getLayer()!=LAYER_GROUND) {
		PathfindLayerEnum layer = parentCell->getLayer();
		// We're starting on a bridge, so link to land at the bridge end points.
		ICoord2D ndx;
		ICoord2D toNdx;
		m_layers[layer].getStartCellIndex(&ndx);
		m_layers[layer].getEndCellIndex(&toNdx);
		PathfindCell *cell = getCell(LAYER_GROUND, toNdx.x, toNdx.y);
		PathfindCell *startCell = getCell(LAYER_GROUND, ndx.x, ndx.y);
		if (cell && startCell) {
			// Close parent cell;
			parentCell = popNextOpenCell();
			if (!parentCell) {
				cleanOpenAndClosedLists();
				goalCell->releaseInfo();
				return nullptr;
			}
			parentCell->putOnClosedList(m_closedList);
			if (!startCell->allocateInfo(ndx)) {
				// TheSuperHackers @info We need to forcefully cleanup dangling pathfinding cells if this failure condition is hit in retail
				// Retail clients will crash beyond this point, but we attempt to recover by performing a full cleanup then enabling the fixed pathfinding codepath
#if RETAIL_COMPATIBLE_PATHFINDING
				if (!s_useFixedPathfinding) {
					s_useFixedPathfinding = true;
					forceCleanCells();
				}
				else
#endif
				{
					cleanOpenAndClosedLists();
					goalCell->releaseInfo();
				}
				return nullptr;
			}
			startCell->setParentCellHierarchical(parentCell);
			cellCount++;
			Int curCost = startCell->costToHierGoal(parentCell);
			Int remCost = startCell->costToHierGoal(goalCell);
			startCell->setCostSoFar(curCost);
			startCell->setTotalCost(remCost);
			startCell->setParentCellHierarchical(parentCell);
			// insert newCell in open list such that open list is sorted, smallest total path cost first
			startCell->putOnSortedOpenList( m_openList );

			cellCount++;
			if(!cell->allocateInfo(toNdx)) {
				// TheSuperHackers @info We need to forcefully cleanup dangling pathfinding cells if this failure condition is hit in retail
				// Retail clients will crash beyond this point, but we attempt to recover by performing a full cleanup then enabling the fixed pathfinding codepath
#if RETAIL_COMPATIBLE_PATHFINDING
				if (!s_useFixedPathfinding) {
					s_useFixedPathfinding = true;
					forceCleanCells();
				}
				else
#endif
				{
					cleanOpenAndClosedLists();
					goalCell->releaseInfo();
				}
				return nullptr;
			}
			curCost = cell->costToHierGoal(parentCell);
			remCost = cell->costToHierGoal(goalCell);
			cell->setCostSoFar(curCost);
			cell->setTotalCost(remCost);
			cell->setParentCellHierarchical(parentCell);
			// insert newCell in open list such that open list is sorted, smallest total path cost first
			cell->putOnSortedOpenList( m_openList );
		}
	}

	PathfindCell *closestCell = nullptr;
	Real closestDistSqr = sqr(HUGE_DIST);

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

		zoneStorageType parentZone;
		if (parentCell->getLayer()==LAYER_GROUND) {
			parentZone = m_zoneManager.getBlockZone(locomotorSurface,
				crusher, parentCell->getXIndex(), parentCell->getYIndex(), m_map);
		}	else {
			parentZone = parentCell->getZone();
		}

		Bool reachedGoal = false;

		Int blockX = parentCell->getXIndex()/PathfindZoneManager::ZONE_BLOCK_SIZE;
		Int blockY = parentCell->getYIndex()/PathfindZoneManager::ZONE_BLOCK_SIZE;
		if (parentZone == goalBlockZone) {
			if (goalBlockNdx.x == -1 || (blockX==goalBlockNdx.x && blockY == goalBlockNdx.y)) {
				reachedGoal = true;
			} else {
				DEBUG_LOG(("Hmm, got match before correct cell."));
			}
		}

		ICoord2D zoneBlockExtent;
		m_zoneManager.getExtent(zoneBlockExtent);

		if (!reachedGoal && m_zoneManager.interactsWithBridge(parentCell->getXIndex(), parentCell->getYIndex())) {
			Int i;
			for (i=0; i<=LAYER_LAST; i++) {
				if (m_layers[i].isUnused() || m_layers[i].isDestroyed()) {
					continue;
				}
				ICoord2D ndx;
				ICoord2D toNdx;
				m_layers[i].getStartCellIndex(&ndx);
				m_layers[i].getEndCellIndex(&toNdx);
				if (ndx.x/PathfindZoneManager::ZONE_BLOCK_SIZE != blockX ||
						ndx.y/PathfindZoneManager::ZONE_BLOCK_SIZE != blockY) {
					m_layers[i].getStartCellIndex(&toNdx);
					m_layers[i].getEndCellIndex(&ndx);
				}
				if (ndx.x<0 || ndx.y<0) continue;
				if (toNdx.x<0 || toNdx.y<0) continue;
				if (ndx.x/PathfindZoneManager::ZONE_BLOCK_SIZE == blockX &&
						ndx.y/PathfindZoneManager::ZONE_BLOCK_SIZE == blockY) {
					// Bridge connects to this block.
					Int bridgeZone = m_zoneManager.getBlockZone(locomotorSurface, crusher, ndx.x, ndx.y, m_map);
					if (bridgeZone != parentZone) {
						continue;
					}
					// We have a winner.
					if (m_layers[i].getZone() == goalBlockZone) {
						reachedGoal = true;
						break;
					}
 					PathfindCell *cell = getCell(LAYER_GROUND, toNdx.x, toNdx.y);
					if (!cell)
						continue;

					if (cell->hasInfo() && (cell->getClosed() || cell->getOpen()))
						continue;

					PathfindCell *startCell = getCell(LAYER_GROUND, ndx.x, ndx.y);
					if (!startCell)
						continue;

					if (startCell != parentCell) {
						if(!startCell->allocateInfo(ndx)) {
							// TheSuperHackers @info We need to forcefully cleanup dangling pathfinding cells if this failure condition is hit in retail
							// Retail clients will crash beyond this point, but we attempt to recover by performing a full cleanup then enabling the fixed pathfinding codepath
#if RETAIL_COMPATIBLE_PATHFINDING
							if (!s_useFixedPathfinding) {
								s_useFixedPathfinding = true;
								forceCleanCells();
							}
							else
#endif
							{
								cleanOpenAndClosedLists();
								goalCell->releaseInfo();
							}
							return nullptr;
						}
						startCell->setParentCellHierarchical(parentCell);
						if (!startCell->getClosed() && !startCell->getOpen()) {
							startCell->putOnClosedList(m_closedList);
						}
					}
					if(!cell->allocateInfo(toNdx)) {
						// TheSuperHackers @info We need to forcefully cleanup dangling pathfinding cells if this failure condition is hit in retail
						// Retail clients will crash beyond this point, but we attempt to recover by performing a full cleanup then enabling the fixed pathfinding codepath
#if RETAIL_COMPATIBLE_PATHFINDING
						if (!s_useFixedPathfinding) {
							s_useFixedPathfinding = true;
							forceCleanCells();
						}
						else
#endif
						{
							cleanOpenAndClosedLists();
							goalCell->releaseInfo();
						}
						return nullptr;
					}
					cell->setParentCellHierarchical(startCell);

					cellCount++;
					Int curCost = cell->costToHierGoal(startCell);
					Int remCost = cell->costToHierGoal(goalCell);

					cell->setCostSoFar(startCell->getCostSoFar() + curCost);
					cell->setTotalCost(cell->getCostSoFar()+remCost);
					cell->setParentCellHierarchical(startCell);
					// insert newCell in open list such that open list is sorted, smallest total path cost first
					cell->putOnSortedOpenList( m_openList );

				}
			}
		}

		if (reachedGoal)
		{
			if (parentCell != goalCell) {
				goalCell->setParentCellHierarchical(parentCell);
			}
			// success - found a path to the goal

			m_isTunneling = false;
			// construct and return path
			Path *path =  buildHierarchicalPath( from, goalCell );
#if defined(RTS_DEBUG)
			Bool show = TheGlobalData->m_debugAI==AI_DEBUG_PATHS;
			show |= (TheGlobalData->m_debugAI==AI_DEBUG_GROUND_PATHS);
			if (show)	{
				debugShowSearch(true);
			}
#endif
#if RETAIL_COMPATIBLE_PATHFINDING
			if (!s_useFixedPathfinding)
			{
				if (goalCell->hasInfo() && !goalCell->getClosed() && !goalCell->getOpen()) {
					goalCell->releaseInfo();
				}
				parentCell->releaseInfo();
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

#if defined(RTS_DEBUG)
#if 0
		Bool show = TheGlobalData->m_debugAI==AI_DEBUG_PATHS;
		show |= (TheGlobalData->m_debugAI==AI_DEBUG_GROUND_PATHS);
		if (show)	{
			extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
 			RGBColor color;
			color.blue = 1;
			color.red = 1;
			color.green = 0;
			Coord3D pos;
			pos.x = ((blockX+0.5f)*PathfindZoneManager::ZONE_BLOCK_SIZE)*PATHFIND_CELL_SIZE_F ;
			pos.y = ((blockY+0.5f)*PathfindZoneManager::ZONE_BLOCK_SIZE)*PATHFIND_CELL_SIZE_F ;
			pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
			addIcon(&pos, 5*PATHFIND_CELL_SIZE_F, 300, color);
		}
#endif
#endif
		Real dx = IABS(goalCell->getXIndex()-parentCell->getXIndex());
		Real dy = IABS(goalCell->getYIndex()-parentCell->getYIndex());
		Real distSqr = dx*dx+dy*dy;
		if (distSqr < closestDistSqr) {
			closestCell = parentCell;
			closestDistSqr = distSqr;
		}

		// put parent cell onto closed list - its evaluation is finished
		parentCell->putOnClosedList( m_closedList );

		Int i;
		zoneStorageType examinedZones[PathfindZoneManager::ZONE_BLOCK_SIZE];
		Int numExZones = 0;
		// Left side.
		if (blockX>0) {
			for (i=1; i<=PathfindZoneManager::ZONE_BLOCK_SIZE; i++) {
			ICoord2D scanCell;
				scanCell.x = blockX*PathfindZoneManager::ZONE_BLOCK_SIZE;
				scanCell.y = (blockY*PathfindZoneManager::ZONE_BLOCK_SIZE);
				scanCell.y += PathfindZoneManager::ZONE_BLOCK_SIZE/2;
				Int offset = i>>1;
				if (i&1) offset = -offset;
				scanCell.y += offset;
				ICoord2D delta;
				delta.x = -1; // left side moves -1.
				delta.y = 0;

				PathfindCell *cell = getCell(LAYER_GROUND, scanCell.x, scanCell.y);
				if (!cell)
					continue;

				if ( cell->hasInfo() && (cell->getClosed() || cell->getOpen()) ) {
					if (parentZone == m_zoneManager.getBlockZone(locomotorSurface, crusher, scanCell.x, scanCell.y, m_map))
						break;
				}

				if (isHuman && checkCellOutsideExtents(scanCell))
					continue;

				processHierarchicalCell(scanCell, delta, parentCell,
					goalCell, parentZone, examinedZones, numExZones, crusher, cellCount);
			}
		}
		// Right side.
		if (blockX<zoneBlockExtent.x-1) {
			numExZones = 0;
			for (i=1; i<=PathfindZoneManager::ZONE_BLOCK_SIZE; i++) {
			ICoord2D scanCell;
				scanCell.x = blockX*PathfindZoneManager::ZONE_BLOCK_SIZE;
				scanCell.x += PathfindZoneManager::ZONE_BLOCK_SIZE-1;
				scanCell.y = (blockY*PathfindZoneManager::ZONE_BLOCK_SIZE);
				scanCell.y += PathfindZoneManager::ZONE_BLOCK_SIZE/2;
				Int offset = i>>1;
				if (i&1) offset = -offset;
				scanCell.y += offset;
				ICoord2D delta;
				delta.x = 1; // right side moves +1.
				delta.y = 0;

				PathfindCell *cell = getCell(LAYER_GROUND, scanCell.x, scanCell.y);
				if (!cell)
					continue;

				if ( cell->hasInfo() && (cell->getClosed() || cell->getOpen()) ) {
					if (parentZone == m_zoneManager.getBlockZone(locomotorSurface, crusher, scanCell.x, scanCell.y, m_map))
						break;
				}

				if (isHuman && checkCellOutsideExtents(scanCell))
					continue;

				processHierarchicalCell(scanCell, delta, parentCell,
					goalCell, parentZone, examinedZones, numExZones, crusher, cellCount);
			}
		}
		// Top side.
		if (blockY>0) {
			numExZones = 0;
			for (i=1; i<=PathfindZoneManager::ZONE_BLOCK_SIZE; i++) {
			ICoord2D scanCell;
				scanCell.y = blockY*PathfindZoneManager::ZONE_BLOCK_SIZE;
				scanCell.x = (blockX*PathfindZoneManager::ZONE_BLOCK_SIZE);
				scanCell.x += PathfindZoneManager::ZONE_BLOCK_SIZE/2;
				Int offset = i>>1;
				if (i&1) offset = -offset;
				scanCell.x += offset;
				ICoord2D delta;
				delta.x = 0;
				delta.y = -1;	// Top side moves -1.

				PathfindCell *cell = getCell(LAYER_GROUND, scanCell.x, scanCell.y);
				if (!cell)
					continue;

				if ( cell->hasInfo() && (cell->getClosed() || cell->getOpen()) ) {
					if (parentZone == m_zoneManager.getBlockZone(locomotorSurface, crusher, scanCell.x, scanCell.y, m_map))
						break;
				}

				if (isHuman && checkCellOutsideExtents(scanCell))
					continue;

				processHierarchicalCell(scanCell, delta, parentCell,
					goalCell, parentZone, examinedZones, numExZones, crusher, cellCount);
			}
		}
		// Bottom side.
		if (blockY<zoneBlockExtent.y-1) {
			numExZones = 0;
			for (i=1; i<=PathfindZoneManager::ZONE_BLOCK_SIZE; i++) {
			ICoord2D scanCell;
				scanCell.y = blockY*PathfindZoneManager::ZONE_BLOCK_SIZE;
				scanCell.y += PathfindZoneManager::ZONE_BLOCK_SIZE-1;
				scanCell.x = (blockX*PathfindZoneManager::ZONE_BLOCK_SIZE);
				scanCell.x += PathfindZoneManager::ZONE_BLOCK_SIZE/2;
				Int offset = i>>1;
				if (i&1) offset = -offset;
				scanCell.x += offset;
				ICoord2D delta;
				delta.x = 0;
				delta.y = 1; // Top side moves +1.

				PathfindCell *cell = getCell(LAYER_GROUND, scanCell.x, scanCell.y);
				if (!cell)
					continue;

				if ( cell->hasInfo() && (cell->getClosed() || cell->getOpen()) ) {
					if (parentZone == m_zoneManager.getBlockZone(locomotorSurface, crusher, scanCell.x, scanCell.y, m_map))
						break;
				}

				if (isHuman && checkCellOutsideExtents(scanCell))
					continue;

				processHierarchicalCell(scanCell, delta, parentCell,
					goalCell, parentZone, examinedZones, numExZones, crusher, cellCount);
			}
		}
	}

	if (closestOK && closestCell) {
		m_isTunneling = false;
		// construct and return path
		Path *path =  buildHierarchicalPath( from, closestCell );

#if RETAIL_COMPATIBLE_PATHFINDING
		if (!s_useFixedPathfinding)
		{
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

	// failure - goal cannot be reached
#if defined(RTS_DEBUG)
	if (TheGlobalData->m_debugAI)
	{
		extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
 		RGBColor color;
		color.blue = 0;
		color.red = color.green = 1;
		addIcon(nullptr, 0, 0, color);
		debugShowSearch(false);
		Coord3D pos;
		pos = *from;
		pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
		addIcon(&pos, 3*PATHFIND_CELL_SIZE_F, 600, color);
		pos = *to;
		pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
		addIcon(&pos, 3*PATHFIND_CELL_SIZE_F, 600, color);
		Real dx, dy;
		dx = from->x - to->x;
		dy = from->y - to->y;

		Int count = sqrt(dx*dx+dy*dy)/(PATHFIND_CELL_SIZE_F/2);
		if (count<2) count = 2;
		Int i;
		color.green = 0;
		for (i=1; i<count; i++) {
			pos.x = from->x + (to->x-from->x)*i/count;
			pos.y = from->y + (to->y-from->y)*i/count;
			pos.z = TheTerrainLogic->getGroundHeight( pos.x, pos.y ) + 0.5f;
			addIcon(&pos, PATHFIND_CELL_SIZE_F/2, 60, color);

		}
	}
#endif

	DEBUG_LOG(("%d FindHierarchicalPath failed from (%f,%f) to (%f,%f) --", TheGameLogic->getFrame(), from->x, from->y, to->x, to->y));
	DEBUG_LOG(("time %f", (::GetTickCount()-startTimeMS)/1000.0f));

#ifdef DUMP_PERF_STATS
	TheGameLogic->incrementOverallFailedPathfinds();
#endif
	m_isTunneling = false;
#if RETAIL_COMPATIBLE_PATHFINDING
	if (!s_useFixedPathfinding)
	{
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
