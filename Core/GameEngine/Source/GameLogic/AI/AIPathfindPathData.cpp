#include "PreRTS.h"

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

#include "Common/UnitTimings.h"

#include "Common/Xfer.h"
#include "Common/XferCRC.h"

#include "Common/PerfMetrics.h"

inline Int IABS(Int x) {	if (x>=0) return x; return -x;};

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

static Real computePathSegmentLength(const PathNode *anchor, const PathNode *node)
{
	Real pathLength = 0.0f;
	for (const PathNode *cur = anchor; cur != nullptr && cur != node; cur = cur->getNext())
	{
		const PathNode *next = cur->getNext();
		if (next == nullptr)
		{
			break;
		}

		const Real dx = next->getPosition()->x - cur->getPosition()->x;
		const Real dy = next->getPosition()->y - cur->getPosition()->y;
		pathLength += sqrt(dx * dx + dy * dy);
	}
	return pathLength;
}

static Bool shouldPreserveGroupSpreadOptimization(const Object *obj, const PathNode *anchor, const PathNode *node)
{
	Coord2D offset;
	if (!isPhase3GroupSpreadActive(obj, &offset))
	{
		return false;
	}

	const Real directDx = node->getPosition()->x - anchor->getPosition()->x;
	const Real directDy = node->getPosition()->y - anchor->getPosition()->y;
	const Real directLength = sqrt(directDx * directDx + directDy * directDy);
	if (directLength < PATHFIND_CELL_SIZE_F * 4.0f)
	{
		return false;
	}

	const Real pathLength = computePathSegmentLength(anchor, node);
	if (pathLength <= directLength + PATHFIND_CELL_SIZE_F * 1.5f)
	{
		return false;
	}

	const Real invLength = 1.0f / directLength;
	Coord2D right;
	right.x = -directDy * invLength;
	right.y = directDx * invLength;
	const Real desiredLateral = offset.x * right.x + offset.y * right.y;
	return fabs(desiredLateral) >= PATHFIND_CELL_SIZE_F * 0.5f;
}

inline Bool isReallyClose(const Coord3D& a, const Coord3D& b)
{
	const Real CLOSE_ENOUGH = 0.1f;
	return
		fabs(a.x-b.x) <= CLOSE_ENOUGH &&
		fabs(a.y-b.y) <= CLOSE_ENOUGH &&
		fabs(a.z-b.z) <= CLOSE_ENOUGH;
}

PathNode::PathNode() :
	m_nextOpti(nullptr),
	m_next(nullptr),
	m_prev(nullptr),
	m_nextOptiDist2D(0),
	m_canOptimize(false),
	m_id(-1)
{
	m_nextOptiDirNorm2D.x = 0;
	m_nextOptiDirNorm2D.y = 0;
	m_pos.zero();
	m_layer = LAYER_INVALID;
}

//-----------------------------------------------------------------------------------
PathNode::~PathNode()
{
}

//-----------------------------------------------------------------------------------
void PathNode::setNextOptimized(PathNode *node)
{
	m_nextOpti = node;
	if (node)
	{
		m_nextOptiDirNorm2D.x = node->getPosition()->x - getPosition()->x;
		m_nextOptiDirNorm2D.y = node->getPosition()->y - getPosition()->y;
		m_nextOptiDist2D = m_nextOptiDirNorm2D.length();
		if (m_nextOptiDist2D == 0.0f)
		{
			m_nextOptiDist2D = 0.01f;
		}
		m_nextOptiDirNorm2D.x /= m_nextOptiDist2D;
		m_nextOptiDirNorm2D.y /= m_nextOptiDist2D;
	}
	else
	{
		m_nextOptiDist2D = 0;
	}
}

//-----------------------------------------------------------------------------------
PathNode *PathNode::prependToList( PathNode *list )
{
	m_next = list;
	if (list)
		list->m_prev = this;
	m_prev = nullptr;
	return this;
}

//-----------------------------------------------------------------------------------
PathNode *PathNode::appendToList( PathNode *list )
{
	if (list == nullptr)
	{
		m_next = nullptr;
		m_prev = nullptr;
		return this;
	}

	PathNode *tail;
	for( tail = list; tail->m_next; tail = tail->m_next )
		;

	tail->m_next = this;
	m_prev = tail;
	m_next = nullptr;

	return list;
}

//-----------------------------------------------------------------------------------
void PathNode::append( PathNode *newNode )
{
	newNode->m_next = this->m_next;
	newNode->m_prev = this;
	if (newNode->m_next) {
		newNode->m_next->m_prev = newNode;
	}
	this->m_next = newNode;

}

//-----------------------------------------------------------------------------------
const Coord3D *PathNode::computeDirectionVector()
{
	static Coord3D dir;

	if (m_next == nullptr)
	{
		if (m_prev == nullptr)
		{
			dir.x = 0.0f;
			dir.y = 0.0f;
			dir.z = 0.0f;
		}
		else
		{
			return m_prev->computeDirectionVector();
		}
	}
	else
	{
		dir.x = m_next->m_pos.x - m_pos.x;
		dir.y = m_next->m_pos.y - m_pos.y;
		dir.z = m_next->m_pos.z - m_pos.z;
	}

	return &dir;
}

//-----------------------------------------------------------------------------------
Path::Path():
m_path(nullptr),
m_pathTail(nullptr),
m_isOptimized(FALSE),
m_blockedByAlly(FALSE),
m_cpopRecentStart(nullptr),
m_cpopCountdown(MAX_CPOP),
m_cpopValid(FALSE)
{
	m_cpopIn.zero();
	m_cpopOut.distAlongPath=0;
	m_cpopOut.layer = LAYER_GROUND;
	m_cpopOut.posOnPath.zero();
}

Path::~Path()
{
	PathNode *node, *nextNode;

	for( node = m_path; node; node = nextNode )
	{
		nextNode = node->getNext();
		deleteInstance(node);
	}
}

void Path::crc( Xfer *xfer )
{
}

void Path::xfer( Xfer *xfer )
{
  XferVersion currentVersion = 1;
  XferVersion version = currentVersion;
  xfer->xferVersion( &version, currentVersion );

	PathNode *node = m_path;
	Int count = 0;
	while (node) {
		count++;
		node = node->getNext();
	}
	xfer->xferInt(&count);

	if (xfer->getXferMode() == XFER_SAVE)	{
		node = m_pathTail;
		while (node) {
			node->m_id = count;
			xfer->xferInt(&count);
			Coord3D pos = *node->getPosition();
			xfer->xferCoord3D(&pos);
			PathfindLayerEnum layer = node->getLayer();
			xfer->xferUser(&layer, sizeof(layer));
			Bool canOpt = node->getCanOptimize();
			xfer->xferBool(&canOpt);
			Int id = -1;
			if (node->getNextOptimized()) {
				id = node->getNextOptimized()->m_id;
			}
			xfer->xferInt(&id);
			count--;
			node = node->getPrevious();
		}
		DEBUG_ASSERTCRASH(count==0, ("Wrong data count"));
	} else {
		m_cpopValid = FALSE;
		while (count) {
			Int nodeId;
			xfer->xferInt(&nodeId);
			DEBUG_ASSERTCRASH(nodeId==count, ("Bad data"));
			Coord3D pos;
			xfer->xferCoord3D(&pos);
			PathfindLayerEnum layer;
			xfer->xferUser(&layer, sizeof(layer));
			Bool canOpt;
			xfer->xferBool(&canOpt);
			Int optID = -1;
			xfer->xferInt(&optID);
			PathNode *node = newInstance(PathNode);
			node->m_id = nodeId;
			node->setPosition(&pos);
			node->setLayer(layer);
			node->setCanOptimize(canOpt);
			PathNode *optNode = nullptr;
			if (optID > 0) {
				optNode = m_path;
				while (optNode && optNode->m_id != optID) {
					optNode = optNode->getNext();
				}
				DEBUG_ASSERTCRASH (optNode && optNode->m_id == optID, ("Could not find optimized link."));
			}
			m_path = node->prependToList(m_path);
			if (m_pathTail == nullptr)
				m_pathTail = node;
			if (optNode) {
				node->setNextOptimized(optNode);
			}
			count--;
		}
	}

	xfer->xferBool(&m_isOptimized);
	Int obsolete1 = 0;
	xfer->xferInt(&obsolete1);
	UnsignedInt obsolete2;
	xfer->xferUnsignedInt(&obsolete2);
	xfer->xferBool(&m_blockedByAlly);


#if defined(RTS_DEBUG)
	if (TheGlobalData->m_debugAI == AI_DEBUG_PATHS)
	{
		extern void addIcon(const Coord3D *pos, Real width, Int numFramesDuration, RGBColor color);
 		RGBColor color;
		color.blue = 0;
		color.red = color.green = 1;
		Coord3D pos;
		addIcon(nullptr, 0, 0, color);
		for( PathNode *node = getFirstNode(); node; node = node->getNext() )
		{

			pos = *node->getPosition();
			addIcon(&pos, PATHFIND_CELL_SIZE_F*.25f, 200, color);
		}

		for( node = getFirstNode(); node; node = node->getNextOptimized() )
		{
			pos = *node->getPosition();
			addIcon(&pos, PATHFIND_CELL_SIZE_F*.8f, 200, color);
		}
		TheAI->pathfinder()->setDebugPath(this);
	}
#endif
}

void Path::loadPostProcess()
{
}

void Path::prependNode( const Coord3D *pos, PathfindLayerEnum layer )
{
	PathNode *node = newInstance(PathNode);

	node->setPosition( pos );
	node->setLayer(layer);

	m_path = node->prependToList( m_path );

	if (m_pathTail == nullptr)
		m_pathTail = node;

	m_isOptimized = false;

#ifdef CPOP_STARTS_FROM_PREV_SEG
	m_cpopRecentStart = nullptr;
#endif
}

void Path::appendNode( const Coord3D *pos, PathfindLayerEnum layer )
{
	if (m_isOptimized && m_pathTail)
	{
		if (pos->x == m_pathTail->getPosition()->x && pos->y == m_pathTail->getPosition()->y) {
			DEBUG_LOG(("Warning - Path Seg length == 0, ignoring. john a."));
			return;
		}
	}
	PathNode *node = newInstance(PathNode);

	node->setPosition( pos );
	node->setLayer(layer);

	m_path = node->appendToList( m_path );

	if (m_isOptimized && m_pathTail)
	{
		m_pathTail->setNextOptimized(node);
	}

	m_pathTail = node;

#ifdef CPOP_STARTS_FROM_PREV_SEG
	m_cpopRecentStart = nullptr;
#endif
}

void Path::updateLastNode( const Coord3D *pos )
{
	PathfindLayerEnum layer = TheTerrainLogic->getLayerForDestination(pos);
	if (m_pathTail) {
		m_pathTail->setPosition(pos);
		m_pathTail->setLayer(layer);
	}
	if (m_isOptimized && m_pathTail)
	{
		PathNode *node = m_path;
		while(node && node->getNextOptimized() != m_pathTail) {
			node = node->getNextOptimized();
		}
		if (node && node->getNextOptimized() == m_pathTail) {
			node->setNextOptimized(m_pathTail);
		}
	}
}

void Path::optimize( const Object *obj, LocomotorSurfaceTypeMask acceptableSurfaces, Bool blocked )
{
	PathNode *node, *anchor;

	anchor = getFirstNode();

	Bool firstNode = true;
	PathfindLayerEnum firstLayer = anchor->getLayer();

	while( anchor != getLastNode() )
	{
		Bool optimizedSegment = false;
		PathfindLayerEnum layer = anchor->getLayer();
		PathfindLayerEnum curLayer = anchor->getLayer();
		Int count = 0;
		const Int ALLOWED_STEPS = 3;
		for (node = anchor->getNext(); node->getNext(); node=node->getNext()) {
			count++;
			if (curLayer==LAYER_GROUND) {
				if (node->getLayer() != curLayer) {
					layer = node->getLayer();
					curLayer = layer;
					if (count > ALLOWED_STEPS) break;
				}
			}	else {
				if (node->getNext()->getLayer() != curLayer) {
					if (count > ALLOWED_STEPS) break;
				}
			}
			curLayer = node->getLayer();
			if (node->getCanOptimize()==false) {
				break;
			}
		}
		if (firstNode) {
			layer = firstLayer;
			firstNode = false;
		}
		for( ; node != anchor; node = node->getPrevious() )
		{
			Bool isPassable = false;
			if (TheAI->pathfinder()->isLinePassable( obj, acceptableSurfaces, layer, *anchor->getPosition(),
				*node->getPosition(), blocked, false))
			{
				isPassable = true;
			}
			PathfindCell* cell = TheAI->pathfinder()->getCell( layer, node->getPosition());
			if (cell && cell->getType()==PathfindCell::CELL_CLIFF && !cell->getPinched()) {
				isPassable = true;
			}
			if (!isPassable) {
				Int dx = node->getPosition()->x - anchor->getPosition()->x;
				Int dy = node->getPosition()->y - anchor->getPosition()->y;
				Bool mightBePassable = false;
				if (IABS(dx)==PATHFIND_CELL_SIZE && IABS(dy)==PATHFIND_CELL_SIZE) {
					isPassable = true;
				}
				PathNode *tmpNode;
				if (dx==0) {
					mightBePassable = true;
					for (tmpNode = node->getPrevious(); tmpNode && tmpNode != anchor; tmpNode = tmpNode->getPrevious()) {
						dx = tmpNode->getNext()->getPosition()->x - tmpNode->getPosition()->x;
						if (dx!=0) mightBePassable = false;
					}
				}
				if (dy==0) {
					mightBePassable = true;
					for (tmpNode = node->getPrevious(); tmpNode && tmpNode != anchor; tmpNode = tmpNode->getPrevious()) {
						dy = tmpNode->getNext()->getPosition()->y - tmpNode->getPosition()->y;
						if (dy!=0) mightBePassable = false;
					}
				}
				if (dx == dy) {
					mightBePassable = true;
					for (tmpNode = node->getPrevious(); tmpNode &&   tmpNode != anchor; tmpNode = tmpNode->getPrevious()) {
						dx = tmpNode->getNext()->getPosition()->x - tmpNode->getPosition()->x;
						dy = tmpNode->getNext()->getPosition()->y - tmpNode->getPosition()->y;
						if (dy!=dx) mightBePassable = false;
					}
				}
				if (dx == -dy) {
					mightBePassable = true;
					for (tmpNode = node->getPrevious(); tmpNode &&   tmpNode != anchor; tmpNode = tmpNode->getPrevious()) {
						dx = tmpNode->getNext()->getPosition()->x - tmpNode->getPosition()->x;
						dy = tmpNode->getNext()->getPosition()->y - tmpNode->getPosition()->y;
						if (dy!=-dx) mightBePassable = false;
					}
				}
				if (mightBePassable) {
					isPassable = true;
				}
			}
			if (isPassable && !shouldPreserveGroupSpreadOptimization(obj, anchor, node))
			{
				anchor->setNextOptimized( node );
				anchor = node;
				optimizedSegment = true;
				break;
			}
		}

		if (optimizedSegment == false)
		{
			anchor->setNextOptimized( anchor->getNext() );
			anchor = anchor->getNext();
		}
	}

	m_isOptimized = true;
}

void Path::optimizeGroundPath( Bool crusher, Int pathDiameter )
{
	PathNode *node, *anchor;

	anchor = getFirstNode();

	while( anchor != getLastNode() )
	{
		Bool optimizedSegment = false;
		PathfindLayerEnum layer = anchor->getLayer();
		PathfindLayerEnum curLayer = anchor->getLayer();
		Int count = 0;
		const Int ALLOWED_STEPS = 3;
		for (node = anchor->getNext(); node->getNext(); node=node->getNext()) {
			count++;
			if (curLayer==LAYER_GROUND) {
				if (node->getLayer() != curLayer) {
					layer = node->getLayer();
					curLayer = layer;
					if (count > ALLOWED_STEPS) break;
				}
			}	else {
				if (node->getNext()->getLayer() != curLayer) {
					if (count > ALLOWED_STEPS) break;
				}
			}
			curLayer = node->getLayer();
		}

		for( ; node != anchor; node = node->getPrevious() )
		{
			Bool isPassable = false;
			if (TheAI->pathfinder()->isGroundPathPassable( crusher, *anchor->getPosition(), layer,
				*node->getPosition(), pathDiameter))
			{
				isPassable = true;
			}
			if (!isPassable) {
				Int dx = node->getPosition()->x - anchor->getPosition()->x;
				Int dy = node->getPosition()->y - anchor->getPosition()->y;
				Bool mightBePassable = false;
				PathNode *tmpNode;
				if (dx==0) {
					mightBePassable = true;
					for (tmpNode = node->getPrevious(); tmpNode && tmpNode != anchor; tmpNode = tmpNode->getPrevious()) {
						dx = tmpNode->getNext()->getPosition()->x - tmpNode->getPosition()->x;
						if (dx!=0) mightBePassable = false;
					}
				}
				if (dy==0) {
					mightBePassable = true;
					for (tmpNode = node->getPrevious(); tmpNode && tmpNode != anchor; tmpNode = tmpNode->getPrevious()) {
						dy = tmpNode->getNext()->getPosition()->y - tmpNode->getPosition()->y;
						if (dy!=0) mightBePassable = false;
					}
				}
				if (dx == dy) {
					mightBePassable = true;
					for (tmpNode = node->getPrevious(); tmpNode &&   tmpNode != anchor; tmpNode = tmpNode->getPrevious()) {
						dx = tmpNode->getNext()->getPosition()->x - tmpNode->getPosition()->x;
						dy = tmpNode->getNext()->getPosition()->y - tmpNode->getPosition()->y;
						if (dy!=dx) mightBePassable = false;
					}
				}
				if (dx == -dy) {
					mightBePassable = true;
					for (tmpNode = node->getPrevious(); tmpNode &&   tmpNode != anchor; tmpNode = tmpNode->getPrevious()) {
						dx = tmpNode->getNext()->getPosition()->x - tmpNode->getPosition()->x;
						dy = tmpNode->getNext()->getPosition()->y - tmpNode->getPosition()->y;
						if (dy!=-dx) mightBePassable = false;
					}
				}
				if (mightBePassable) {
					isPassable = true;
				}
			}
			if (isPassable)
			{
				anchor->setNextOptimized( node );
				anchor = node;
				optimizedSegment = true;
				break;
			}
		}

		if (optimizedSegment == false)
		{
			anchor->setNextOptimized( anchor->getNext() );
			anchor = anchor->getNext();
		}
	}

	for (anchor=getFirstNode(); anchor!=nullptr; anchor=anchor->getNextOptimized()) {
		node = anchor->getNextOptimized();
		if (node && node->getNextOptimized()) {
			Real dx = node->getPosition()->x - anchor->getPosition()->x;
			Real dy = node->getPosition()->y - anchor->getPosition()->y;
			if (dx*dx+dy*dy < sqr(PATHFIND_CELL_SIZE_F)*3.9f) {
				anchor->setNextOptimized(node->getNextOptimized());
			}
		}
	}

	m_isOptimized = true;
}

void Path::computePointOnPath(
	const Object* obj,
	const LocomotorSet& locomotorSet,
	const Coord3D& pos,
	ClosestPointOnPathInfo& out
)
{
	CRCDEBUG_LOG(("Path::computePointOnPath() for %s", DebugDescribeObject(obj).str()));

	out.layer = LAYER_GROUND;
	out.posOnPath.zero();
	out.distAlongPath = 0;

	if (m_path == nullptr)
	{
		m_cpopValid = false;
		return;
	}
	out.layer = m_path->getLayer();

	if (m_cpopValid && m_cpopCountdown>0 && isReallyClose(pos, m_cpopIn))
	{
		out = m_cpopOut;
		m_cpopCountdown--;
		CRCDEBUG_LOG(("Path::computePointOnPath() end because we're really close"));
		return;
	}
	m_cpopCountdown = MAX_CPOP;

	out.posOnPath = *getLastNode()->getPosition();

	const PathNode* closeNode = nullptr;
	Coord2D toPos;
	Real closeDistSqr = 99999999.9f;
	Real totalPathLength = 0.0f;
	Real lengthAlongPathToPos = 0.0f;

#ifdef CPOP_STARTS_FROM_PREV_SEG
	const PathNode* prevNode = m_cpopRecentStart;
	if (prevNode == nullptr)
		prevNode = m_path;
#else
	const PathNode* prevNode = m_path;
#endif
	Coord2D segmentDirNorm;
	Real segmentLength;

	for ( const PathNode* node = prevNode->getNextOptimized(&segmentDirNorm, &segmentLength);
				node != nullptr;
				node = node->getNextOptimized(&segmentDirNorm, &segmentLength) )
	{
		const Coord3D* prevNodePos = prevNode->getPosition();
		const Coord3D* nodePos = node->getPosition();

		toPos.x = pos.x - prevNodePos->x;
		toPos.y = pos.y - prevNodePos->y;

		Real alongPathDist = segmentDirNorm.x * toPos.x + segmentDirNorm.y * toPos.y;

		Coord3D pointOnPath;
		if (alongPathDist < 0.0f)
		{
			alongPathDist = 0.0f;
			pointOnPath = *prevNodePos;
		}
		else if (alongPathDist > segmentLength)
		{
			if (node->getNextOptimized() == nullptr)
			{
				alongPathDist = segmentLength;
				pointOnPath = *nodePos;
			}
			else
			{
				totalPathLength += segmentLength;
				prevNode = node;
				continue;
			}
		}
		else
		{
			pointOnPath.x = prevNodePos->x + alongPathDist * segmentDirNorm.x;
			pointOnPath.y = prevNodePos->y + alongPathDist * segmentDirNorm.y;
			pointOnPath.z = 0;
		}

		Coord2D offset;
		offset.x = pos.x - pointOnPath.x;
		offset.y = pos.y - pointOnPath.y;

		Real offsetDistSqr = offset.x*offset.x + offset.y*offset.y;
		if (offsetDistSqr < closeDistSqr)
		{
			closeDistSqr = offsetDistSqr;
			closeNode = prevNode;
			out.posOnPath = pointOnPath;

			lengthAlongPathToPos = totalPathLength + alongPathDist;
		}

		totalPathLength += segmentLength;
		prevNode = node;
		DUMPCOORD3D(&pointOnPath);
	}

#ifdef CPOP_STARTS_FROM_PREV_SEG
	m_cpopRecentStart = closeNode;
#endif

	if (closeNode && closeNode->getNextOptimized())
	{
		const PathNode* closeNext = closeNode->getNextOptimized(&segmentDirNorm, &segmentLength);
		const Coord3D* nextNodePos = closeNext->getPosition();
		const Coord3D* closeNodePos = closeNode->getPosition();

		const PathNode* closePrev = closeNode->getPrevious();
		if (closePrev && closePrev->getLayer() > LAYER_GROUND)
		{
			out.layer = closeNode->getLayer();
		}
		if (closeNode->getLayer() > LAYER_GROUND)
		{
			out.layer = closeNode->getLayer();
		}

		if (closeNext->getLayer() > LAYER_GROUND)
		{
			out.layer = closeNext->getLayer();
		}

		toPos.x = pos.x - closeNodePos->x;
		toPos.y = pos.y - closeNodePos->y;

		Real alongPathDist = segmentDirNorm.x * toPos.x + segmentDirNorm.y * toPos.y;

		if (alongPathDist < 0.0f)
			alongPathDist = 0.0f;

		Real toDistSqr = sqr(toPos.x) + sqr(toPos.y);
		Real offsetDistSq = toDistSqr - sqr(alongPathDist);
		Real offsetDist = (offsetDistSq <= 0.0) ? 0.0 : sqrt(offsetDistSq);

		const Real maxPathError = 3.0f * PATHFIND_CELL_SIZE_F;
		const Real maxPathErrorInv = 1.0 / maxPathError;
		Real k = offsetDist * maxPathErrorInv;
		if (k > 1.0f)
			k = 1.0f;

		Bool gotPos = false;
		CRCDEBUG_LOG(("Path::computePointOnPath() calling isLinePassable() 1"));
		if (TheAI->pathfinder()->isLinePassable( obj, locomotorSet.getValidSurfaces(), out.layer, pos, *nextNodePos,
			false, true ))
		{
			out.posOnPath = *nextNodePos;
			gotPos = true;

			Bool tryAhead = alongPathDist > segmentLength * 0.5;
			if (closeNext->getCanOptimize() == false)
			{
				tryAhead = false;
			}
			if (closeNode->getLayer() != closeNext->getLayer())
			{
				tryAhead = false;
			}
			if (obj->getLayer()!=LAYER_GROUND) {
				tryAhead = false;
			}
			Bool veryClose = false;
			if (segmentLength-alongPathDist<1.0f) {
				tryAhead = true;
				veryClose = true;
			}
			if (tryAhead)
			{
				const PathNode *next = closeNext->getNextOptimized();
				if (next)
				{
					Coord3D tryPos;
					tryPos.x = (nextNodePos->x + next->getPosition()->x) * 0.5;
					tryPos.y = (nextNodePos->y + next->getPosition()->y) * 0.5;
					tryPos.z = nextNodePos->z;
					CRCDEBUG_LOG(("Path::computePointOnPath() calling isLinePassable() 2"));
					if (veryClose || TheAI->pathfinder()->isLinePassable( obj, locomotorSet.getValidSurfaces(), closeNext->getLayer(), pos, tryPos, false, true ))
					{
						gotPos = true;
						out.posOnPath = tryPos;
					}
				}
			}
		}
		else if (k > 0.5f)
		{
			Real tryDist = alongPathDist + (0.5) * (segmentLength - alongPathDist);

			out.posOnPath.x = closeNodePos->x + tryDist * segmentDirNorm.x;
			out.posOnPath.y = closeNodePos->y + tryDist * segmentDirNorm.y;
			out.posOnPath.z = closeNodePos->z;

			CRCDEBUG_LOG(("Path::computePointOnPath() calling isLinePassable() 3"));
			if (TheAI->pathfinder()->isLinePassable( obj, locomotorSet.getValidSurfaces(), out.layer, pos, out.posOnPath, false, true ))
			{
				k = 0.5f;
				gotPos = true;
			}
		}

		alongPathDist += (1.0f - k) * (segmentLength - alongPathDist);

		if (!gotPos)
		{
			if (alongPathDist > segmentLength)
			{
				alongPathDist = segmentLength;
				out.posOnPath = *nextNodePos;
			}
			else
			{
				out.posOnPath.x = closeNodePos->x + alongPathDist * segmentDirNorm.x;
				out.posOnPath.y = closeNodePos->y + alongPathDist * segmentDirNorm.y;
				out.posOnPath.z = closeNodePos->z;
				Real dx = fabs(pos.x - out.posOnPath.x);
				Real dy = fabs(pos.y - out.posOnPath.y);
				if (dx<1 && dy<1 && closeNode->getNextOptimized() && closeNode->getNextOptimized()->getNextOptimized()) {
					out.posOnPath = *closeNode->getNextOptimized()->getNextOptimized()->getPosition();
				}
			}
		}
	}

	TheAI->pathfinder()->setDebugPathPosition( &out.posOnPath );

	out.distAlongPath = totalPathLength - lengthAlongPathToPos;

	Coord3D delta;
	delta.x = out.posOnPath.x - pos.x;
	delta.y = out.posOnPath.y - pos.y;
	delta.z = 0;
	Real lenDelta = delta.length();
	if (lenDelta > out.distAlongPath && out.distAlongPath > PATHFIND_CLOSE_ENOUGH)
	{
		out.distAlongPath = lenDelta;
	}

	m_cpopIn = pos;
	m_cpopOut = out;
	m_cpopValid = true;
	CRCDEBUG_LOG(("Path::computePointOnPath() end"));

}

Real Path::computeFlightDistToGoal( const Coord3D *pos, Coord3D& goalPos )
{
	if (m_path == nullptr)
	{
		goalPos.x = 0.0f;
		goalPos.y = 0.0f;
		goalPos.z = 0.0f;
		return 0.0f;
	}
	const PathNode *curNode = getFirstNode();
	if (m_cpopRecentStart) {
		curNode = m_cpopRecentStart;
	} else {
		m_cpopRecentStart = curNode;
	}
	const PathNode *nextNode = curNode->getNextOptimized();
	goalPos = *curNode->getPosition();
	Real distance = 0;
	Bool useNext = true;
	while (nextNode) {

		if (useNext) {
			goalPos = *nextNode->getPosition();
		}

		Coord3D startPos = *curNode->getPosition();
		Coord3D endPos = *nextNode->getPosition();

		Coord2D posToGoalVector;
		posToGoalVector.x = endPos.x - pos->x;
		posToGoalVector.y = endPos.y - pos->y;

		Coord2D pathVector;
		pathVector.x = endPos.x - startPos.x;
		pathVector.y = endPos.y - startPos.y;

		pathVector.normalize();

		Real dotProduct = posToGoalVector.x*pathVector.x	+ posToGoalVector.y*pathVector.y;
		if (dotProduct>=0) {
			distance += dotProduct;
			useNext = false;
		}	else if (useNext) {
			m_cpopRecentStart = nextNode;
		}
		curNode = nextNode;
		nextNode = curNode->getNextOptimized();
	}
	return distance;

}
