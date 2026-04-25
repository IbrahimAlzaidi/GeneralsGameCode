#include "PreRTS.h"

#include "GameLogic/AIPathfind.h"

#include "Common/CRCDebug.h"
#include "Common/GlobalData.h"
#include "Common/Xfer.h"

#include "GameLogic/AI.h"
#include "GameLogic/GameLogic.h"
#include "GameLogic/Object.h"

//-----------------------------------------------------------------------------
void Pathfinder::crc( Xfer *xfer )
{
	CRCDEBUG_LOG(("Pathfinder::crc() on frame %d", TheGameLogic->getFrame()));
	CRCDEBUG_LOG(("beginning CRC: %8.8X", ((XferCRC *)xfer)->getCRC()));

	xfer->xferUser( &m_extent, sizeof(IRegion2D) );
	CRCDEBUG_LOG(("m_extent: %8.8X", ((XferCRC *)xfer)->getCRC()));

	xfer->xferBool( &m_isMapReady );
	CRCDEBUG_LOG(("m_isMapReady: %8.8X", ((XferCRC *)xfer)->getCRC()));
	xfer->xferBool( &m_isTunneling );
	CRCDEBUG_LOG(("m_isTunneling: %8.8X", ((XferCRC *)xfer)->getCRC()));

	Int obsolete1 = 0;
	xfer->xferInt( &obsolete1 );

	xfer->xferUser(&m_ignoreObstacleID, sizeof(ObjectID));
	CRCDEBUG_LOG(("m_ignoreObstacleID: %8.8X", ((XferCRC *)xfer)->getCRC()));

	xfer->xferUser(m_queuedPathfindRequests, sizeof(ObjectID)*PATHFIND_QUEUE_LEN);
	CRCDEBUG_LOG(("m_queuedPathfindRequests: %8.8X", ((XferCRC *)xfer)->getCRC()));
	xfer->xferInt(&m_queuePRHead);
	CRCDEBUG_LOG(("m_queuePRHead: %8.8X", ((XferCRC *)xfer)->getCRC()));
	xfer->xferInt(&m_queuePRTail);
	CRCDEBUG_LOG(("m_queuePRTail: %8.8X", ((XferCRC *)xfer)->getCRC()));

	xfer->xferInt(&m_numWallPieces);
	CRCDEBUG_LOG(("m_numWallPieces: %8.8X", ((XferCRC *)xfer)->getCRC()));

#if RETAIL_COMPATIBLE_CRC
	// TheSuperHackers @fix The original code effectively accessed m_numWallPieces 128 times,
	// because it used &m_wallPieces[MAX_WALL_PIECES] which is out-of-bounds and points to m_numWallPieces.
	static_assert(sizeof(Int) == sizeof(ObjectID), "Type sizes must be equal for correct xfer");

	for (Int i = 0; i < MAX_WALL_PIECES; ++i)
	{
		xfer->xferInt(&m_numWallPieces);
	}
#else
	xfer->xferUser(m_wallPieces, sizeof(m_wallPieces));
#endif

	CRCDEBUG_LOG(("m_wallPieces: %8.8X", ((XferCRC *)xfer)->getCRC()));

	xfer->xferReal(&m_wallHeight);
	CRCDEBUG_LOG(("m_wallHeight: %8.8X", ((XferCRC *)xfer)->getCRC()));
	xfer->xferInt(&m_cumulativeCellsAllocated);
	CRCDEBUG_LOG(("m_cumulativeCellsAllocated: %8.8X", ((XferCRC *)xfer)->getCRC()));

}

//-----------------------------------------------------------------------------
void Pathfinder::xfer( Xfer *xfer )
{

	// version
	XferVersion currentVersion = 1;
	XferVersion version = currentVersion;
	xfer->xferVersion( &version, currentVersion );

}

//-----------------------------------------------------------------------------
void Pathfinder::loadPostProcess()
{

}
