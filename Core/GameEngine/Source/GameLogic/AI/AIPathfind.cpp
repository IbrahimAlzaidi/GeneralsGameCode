/*
**	Command & Conquer Generals Zero Hour(tm)
**	Copyright 2025 Electronic Arts Inc.
**
**	This program is free software: you can redistribute it and/or modify
**	it under the terms of the GNU General Public License as published by
**	the Free Software Foundation, either version 3 of the License, or
**	(at your option) any later version.
**
**	This program is distributed in the hope that it will be useful,
**	but WITHOUT ANY WARRANTY; without even the implied warranty of
**	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**	GNU General Public License for more details.
**
**	You should have received a copy of the GNU General Public License
**	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

////////////////////////////////////////////////////////////////////////////////
//																																						//
//  (c) 2001-2003 Electronic Arts Inc.																				//
//																																						//
////////////////////////////////////////////////////////////////////////////////

// AIPathfind.cpp
// AI pathfinding system
// Author: Michael S. Booth, October 2001
#include "PreRTS.h"	// This must go first in EVERY cpp file in the GameEngine

#include <cstring>

#include "GameLogic/AIPathfind.h"

#include "Common/PerfTimer.h"
#include "Common/PerfTrace.h"
#include "Common/Player.h"
#include "Common/CRCDebug.h"
#include "Common/GlobalData.h"
#include "Common/LatchRestore.h"
#include "Common/ThingTemplate.h"
#include "Common/ThingFactory.h"

#include "GameClient/Line2D.h"

#include "GameLogic/AI.h"
#include "GameLogic/GameLogic.h"
#include "GameLogic/Locomotor.h"
#include "GameLogic/Module/ContainModule.h"
#include "GameLogic/Module/AIUpdate.h"
#include "GameLogic/Module/PhysicsUpdate.h"
#include "GameLogic/Object.h"
#include "GameLogic/PartitionManager.h"
#include "GameLogic/TerrainLogic.h"
#include "GameLogic/Weapon.h"
#if RETAIL_COMPATIBLE_PATHFINDING
#include "GameClient/InGameUI.h"
#include "GameClient/GameText.h"
#include "Common/GameAudio.h"
#include "Common/MiscAudio.h"
#endif

#include "Common/UnitTimings.h" //Contains the DO_UNIT_TIMINGS define jba.

#define no_INTENSE_DEBUG

#define DEBUG_QPF

#ifdef INTENSE_DEBUG
#include "GameLogic/ScriptEngine.h"
#endif

#include "Common/Xfer.h"
#include "Common/XferCRC.h"

//------------------------------------------------------------------------------ Performance Timers
#include "Common/PerfMetrics.h"

//-------------------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------
extern Bool s_useFixedPathfinding;
extern Bool s_forceCleanCells;

#if RETAIL_COMPATIBLE_PATHFINDING
void Pathfinder::forceCleanCells()
{
	UnicodeString pathfinderFailoverMessage = TheGameText->FETCH_OR_SUBSTITUTE("GUI:PathfindingCrashPrevented", L"A pathfinding crash was prevented, now switching to the crash fixed pathfinding.");
	TheInGameUI->message(pathfinderFailoverMessage);

	TheAudio->addAudioEvent(&TheAudio->getMiscAudio()->m_allCheerSound);

	PathfindCellInfo::forceCleanPathFindCellInfos();
	m_openList.reset();
	m_openHeap.reset();
	m_closedList.reset();

	for (int j = 0; j <= m_extent.hi.y; ++j) {
		for (int i = 0; i <= m_extent.hi.x; ++i) {
#if RETAIL_COMPATIBLE_PATHFINDING_ALLOCATION
			if (m_map[i][j].isObstructionInvalid()) {
				m_map[i][j].clearObstruction();
			}
#endif
			if (m_map[i][j].hasInfo()) {
				m_map[i][j].releaseInfo();
			}
		}
	}
}
#endif



//----------------------- Pathfinder ---------------------------------------

Pathfinder::Pathfinder() :m_map(nullptr)
{
	debugPath = nullptr;
	PathfindCellInfo::allocateCellInfos();
	reset();
}

Pathfinder::~Pathfinder()
{
	PathfindCellInfo::releaseCellInfos();
}

void Pathfinder::reset()
{
	DEBUG_LOG(("Pathfind cell is %d bytes, PathfindCellInfo is %d bytes", sizeof(PathfindCell), sizeof(PathfindCellInfo)));

	delete [] m_blockOfMapCells;
	m_blockOfMapCells = nullptr;

	delete [] m_map;
	m_map = nullptr;

	Int i;
	for (i=0; i<=LAYER_LAST; i++) {
		m_layers[i].reset();
	}

	// reset the pathfind grid
	m_extent.lo.x=m_extent.lo.y=m_extent.hi.x=m_extent.hi.y=0;
	m_logicalExtent.lo.x=m_logicalExtent.lo.y=m_logicalExtent.hi.x=m_logicalExtent.hi.y=0;
	m_openList.reset();
	m_openHeap.reset();
	m_closedList.reset();
	m_useHeapOpenList = true;

	m_ignoreObstacleID = INVALID_ID;
	m_isTunneling = false;

	m_moveAlliesDepth = 0;

	// pathfind grid cells have not been classified yet
	m_isMapReady = false;
	m_cumulativeCellsAllocated = 0;
	m_requestCellBudget = 0;
	m_recentlyServicedCursor = 0;
	m_nextQueuedRequestSequence = 1;
	m_nextOpenInsertOrder = 1;

	debugPathPos.x = 0.0f;
	debugPathPos.y = 0.0f;
	debugPathPos.z = 0.0f;

	deleteInstance(debugPath);
	debugPath = nullptr;

	m_frameToShowObstacles = 0;

	for (m_queuePRHead=0; m_queuePRHead<PATHFIND_QUEUE_LEN; m_queuePRHead++) {
		m_queuedPathfindRequests[m_queuePRHead] = INVALID_ID;
		m_queuedRequestFrames[m_queuePRHead] = 0;
		m_queuedRequestClasses[m_queuePRHead] = static_cast<UnsignedByte>(PerfTrace::PATH_REQUEST_EXPLICIT_MOVE);
		m_queuedRequestDestinations[m_queuePRHead].zero();
		m_queuedRequestGoalFingerprints[m_queuePRHead] = 0;
		m_queuedRequestLocomotorFingerprints[m_queuePRHead] = 0;
		m_queuedRequestSequences[m_queuePRHead] = 0;
		m_recentlyServicedRequestIDs[m_queuePRHead] = INVALID_ID;
		m_recentlyServicedRequestFrames[m_queuePRHead] = 0;
		m_recentlyServicedRequestClasses[m_queuePRHead] = static_cast<UnsignedByte>(PerfTrace::PATH_REQUEST_EXPLICIT_MOVE);
		m_recentlyServicedGoalFingerprints[m_queuePRHead] = 0;
		m_recentlyServicedCooldownUntilFrames[m_queuePRHead] = 0;
	}
	m_queuePRHead = 0;
	m_queuePRTail = 0;

	m_numWallPieces = 0;
	for (i=0; i<MAX_WALL_PIECES; ++i)
	{
		m_wallPieces[i] = INVALID_ID;
	}

	if (TheAI && TheAI->getAiData()) {
		m_wallHeight = TheAI->getAiData()->m_wallHeight;
	}
	else
	{
		m_wallHeight = 0.0f;
	}
	m_zoneManager.reset();

#if RETAIL_COMPATIBLE_PATHFINDING
	s_useFixedPathfinding = false;
	s_forceCleanCells = false;
#endif

#if RTS_ZEROHOUR && RETAIL_COMPATIBLE_CRC
	m_classifyFenceZeroInit = false;
#endif
}

/**
 * Adds a piece of a wall.
 */
void Pathfinder::addWallPiece(Object *wallPiece)
{
	if (m_numWallPieces<MAX_WALL_PIECES-1) {
		m_wallPieces[m_numWallPieces] = wallPiece->getID();
		m_numWallPieces++;
	}
}

/**
 * Removes a piece of a wall
 */
void Pathfinder::removeWallPiece(Object *wallPiece)
{

	// sanity
  if( wallPiece == nullptr )
		return;

	// find entry
	for( Int i = 0; i < m_numWallPieces; ++i )
	{

		// match by id
		if( m_wallPieces[ i ] == wallPiece->getID() )
		{

			// put the last id in the wall piece array here
			m_wallPieces[ i ] = m_wallPieces[ m_numWallPieces - 1 ];

			// we now have one less entry
			m_numWallPieces--;

			// all done
			return;

		}

	}

}

/**
 * Checks if a point is on the wall.
 */
Bool Pathfinder::isPointOnWall(const Coord3D *pos)
{
	if (m_numWallPieces==0) return false;
	if (m_layers[LAYER_WALL].isUnused()) return false;
	PathfindLayerEnum layer = (PathfindLayerEnum)LAYER_WALL;
	PathfindCell *cell = getCell(layer, pos);
	// make sure the layer matches, since getCell can return ground layer cells if the pos is 'off' the bridge/wall
	if (cell && cell->getLayer() == layer) {
		if (cell->getType() == PathfindCell::CELL_CLEAR) {
			return true;
		}
	}
	return false;
}

/**
 * Adds a bridge & returns the layer.
 */
PathfindLayerEnum Pathfinder::addBridge(Bridge *theBridge)
{
	Int layer = LAYER_GROUND+1;
	while (layer<=LAYER_WALL) {
		if (m_layers[layer].isUnused()) {
			if (m_layers[layer].init(theBridge, (PathfindLayerEnum)layer) ) {
				return (PathfindLayerEnum)layer;
			}
			DEBUG_LOG(("WARNING: Bridge failed to init in pathfinder"));
			return LAYER_GROUND; // failed to init, usually cause off of the map.  jba.
		}
		layer++;
	}
	DEBUG_CRASH(("Ran out of bridge layers."));
	return LAYER_GROUND;
}

/**
 * Updates an object's layer, making sure the object is actually on the bridge first.
 */
void Pathfinder::updateLayer(Object *obj, PathfindLayerEnum layer)
{
	if (layer != LAYER_GROUND) {
		if (!TheTerrainLogic->objectInteractsWithBridgeLayer(obj, layer)) {
			layer = LAYER_GROUND;
		}
	}
	//DEBUG_LOG(("Object layer is %d", layer));
	obj->setLayer(layer);
}

