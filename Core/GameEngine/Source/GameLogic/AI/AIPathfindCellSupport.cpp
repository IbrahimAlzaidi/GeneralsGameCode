#include "PreRTS.h"

#include <cstring>

#include "GameLogic/AIPathfind.h"

#include "Common/PerfTrace.h"
#include "Common/CRCDebug.h"
#include "Common/GlobalData.h"

#include "GameLogic/AI.h"
#include "GameLogic/GameLogic.h"

#if RETAIL_COMPATIBLE_PATHFINDING
#include "GameClient/InGameUI.h"
#include "GameClient/GameText.h"
#include "Common/GameAudio.h"
#include "Common/MiscAudio.h"
#endif

#include "Common/PerfMetrics.h"

PathfindCellInfo *PathfindCellInfo::s_infoArray = nullptr;
PathfindCellInfo *PathfindCellInfo::s_firstFree = nullptr;
static PathfindCellInfoArena s_cellInfoArena;

#if RETAIL_COMPATIBLE_PATHFINDING
Bool s_useFixedPathfinding = false;
Bool s_forceCleanCells = false;

void PathfindCellInfo::forceCleanPathFindCellInfos()
{
	s_cellInfoArena.reset();
	s_cellInfoArena.init();
}
#endif

PathfindCellHeap::PathfindCellHeap()
	: m_capacity(0), m_size(0), m_data(nullptr)
{
	grow();
}

PathfindCellHeap::~PathfindCellHeap()
{
	delete[] m_data;
	m_data = nullptr;
	m_capacity = 0;
	m_size = 0;
}

void PathfindCellHeap::reset()
{
	m_size = 0;
}

void PathfindCellHeap::clear()
{
	m_size = 0;
}

void PathfindCellHeap::grow()
{
	Int newCapacity = m_capacity == 0 ? INITIAL_CAPACITY : m_capacity * 2;
	PathfindCell **newData = MSGNEW("PathfindHeap") PathfindCell*[newCapacity];
	if (m_data && m_size > 0)
	{
		std::memcpy(newData, m_data, m_size * sizeof(PathfindCell *));
	}
	delete[] m_data;
	m_data = newData;
	m_capacity = newCapacity;
}

static Bool pathfindHeapLess(const PathfindCell *lhs, const PathfindCell *rhs)
{
	if (lhs->getTotalCost() != rhs->getTotalCost())
	{
		return lhs->getTotalCost() < rhs->getTotalCost();
	}
	return lhs->getOpenInsertOrder() < rhs->getOpenInsertOrder();
}

void PathfindCellHeap::push(PathfindCell *cell)
{
	if (m_size >= m_capacity)
	{
		grow();
	}
	m_data[m_size] = cell;
	Int idx = m_size;
	m_size++;
	while (idx > 0)
	{
		Int parent = (idx - 1) / 2;
		if (!pathfindHeapLess(m_data[idx], m_data[parent]))
		{
			break;
		}
		PathfindCell *tmp = m_data[parent];
		m_data[parent] = m_data[idx];
		m_data[idx] = tmp;
		idx = parent;
	}
}

PathfindCell *PathfindCellHeap::pop()
{
	if (m_size == 0)
	{
		return nullptr;
	}
	PathfindCell *result = m_data[0];
	m_size--;
	if (m_size > 0)
	{
		m_data[0] = m_data[m_size];
		Int idx = 0;
		for (;;)
		{
			Int left = 2 * idx + 1;
			Int right = 2 * idx + 2;
			Int smallest = idx;
			if (left < m_size && pathfindHeapLess(m_data[left], m_data[smallest]))
			{
				smallest = left;
			}
			if (right < m_size && pathfindHeapLess(m_data[right], m_data[smallest]))
			{
				smallest = right;
			}
			if (smallest == idx)
			{
				break;
			}
			PathfindCell *tmp = m_data[idx];
			m_data[idx] = m_data[smallest];
			m_data[smallest] = tmp;
			idx = smallest;
		}
	}
	return result;
}

PathfindCellInfoArena::PathfindCellInfoArena()
	: m_blocks(nullptr), m_blockCount(0), m_blockCapacity(0),
	  m_firstFree(nullptr), m_liveCount(0), m_peakLive(0),
	  m_allocFailures(0), m_totalCapacity(0)
{
}

PathfindCellInfoArena::~PathfindCellInfoArena()
{
	reset();
}

void PathfindCellInfoArena::init()
{
	reset();
	m_blockCapacity = 4;
	m_blocks = MSGNEW("PathfindArena") Block[m_blockCapacity];
	m_blockCount = 0;
	m_firstFree = nullptr;

	Int initialCount = CELL_INFOS_INITIAL;
	PathfindCellInfo *block = MSGNEW("PathfindCellInfo") PathfindCellInfo[initialCount];
	if (m_blockCount >= m_blockCapacity)
	{
		Int newCap = m_blockCapacity * 2;
		Block *newBlocks = MSGNEW("PathfindArena") Block[newCap];
		for (Int i = 0; i < m_blockCount; i++)
		{
			newBlocks[i] = m_blocks[i];
		}
		delete[] m_blocks;
		m_blocks = newBlocks;
		m_blockCapacity = newCap;
	}
	m_blocks[m_blockCount].data = block;
	m_blocks[m_blockCount].count = initialCount;
	m_blockCount++;
	m_totalCapacity += initialCount;

	for (Int i = initialCount - 1; i >= 0; i--)
	{
		block[i].m_isFree = true;
		block[i].m_pathParent = m_firstFree;
		m_firstFree = &block[i];
	}
}

void PathfindCellInfoArena::reset()
{
	for (Int i = 0; i < m_blockCount; i++)
	{
		delete[] m_blocks[i].data;
	}
	delete[] m_blocks;
	m_blocks = nullptr;
	m_blockCount = 0;
	m_blockCapacity = 0;
	m_firstFree = nullptr;
	m_liveCount = 0;
	m_peakLive = 0;
	m_allocFailures = 0;
	m_totalCapacity = 0;
}

PathfindCellInfo *PathfindCellInfoArena::alloc(PathfindCell *cell, const ICoord2D &pos)
{
	if (!m_firstFree)
	{
		Int growCount = CELL_INFOS_ARENA_BLOCK;
		PathfindCellInfo *block = MSGNEW("PathfindCellInfo") PathfindCellInfo[growCount];
		if (!block)
		{
			m_allocFailures++;
			return nullptr;
		}
		if (m_blockCount >= m_blockCapacity)
		{
			Int newCap = m_blockCapacity * 2;
			Block *newBlocks = MSGNEW("PathfindArena") Block[newCap];
			for (Int i = 0; i < m_blockCount; i++)
			{
				newBlocks[i] = m_blocks[i];
			}
			delete[] m_blocks;
			m_blocks = newBlocks;
			m_blockCapacity = newCap;
		}
		m_blocks[m_blockCount].data = block;
		m_blocks[m_blockCount].count = growCount;
		m_blockCount++;
		m_totalCapacity += growCount;

		for (Int i = growCount - 1; i >= 0; i--)
		{
			block[i].m_isFree = true;
			block[i].m_pathParent = m_firstFree;
			m_firstFree = &block[i];
		}
	}

	PathfindCellInfo *info = m_firstFree;
	m_firstFree = info->m_pathParent;
	info->m_isFree = false;
	info->m_cell = cell;
	info->m_pos = pos;
	info->m_nextOpen = nullptr;
	info->m_prevOpen = nullptr;
	info->m_pathParent = nullptr;
	info->m_costSoFar = 0;
	info->m_totalCost = 0;
	info->m_openInsertOrder = 0;
	info->m_open = 0;
	info->m_closed = 0;
	info->m_obstacleID = INVALID_ID;
	info->m_goalUnitID = INVALID_ID;
	info->m_posUnitID = INVALID_ID;
	info->m_goalAircraftID = INVALID_ID;
	info->m_obstacleIsFence = false;
	info->m_obstacleIsTransparent = false;
	info->m_blockedByAlly = false;

	m_liveCount++;
	if (m_liveCount > m_peakLive)
	{
		m_peakLive = m_liveCount;
	}
	return info;
}

void PathfindCellInfoArena::free(PathfindCellInfo *info)
{
	DEBUG_ASSERTCRASH(!info->m_isFree, ("Double free of PathfindCellInfo."));
	info->m_pathParent = m_firstFree;
	m_firstFree = info;
	info->m_isFree = true;
	m_liveCount--;
}

void PathfindCellInfo::allocateCellInfos()
{
	releaseCellInfos();
	s_cellInfoArena.init();
	s_infoArray = nullptr;
	s_firstFree = nullptr;
}

void PathfindCellInfo::releaseCellInfos()
{
	s_cellInfoArena.reset();
	s_infoArray = nullptr;
	s_firstFree = nullptr;
}

PathfindCellInfo *PathfindCellInfo::getACellInfo(PathfindCell *cell, const ICoord2D &pos)
{
	PathfindCellInfo *info = s_cellInfoArena.alloc(cell, pos);
	if (!info)
	{
		DEBUG_CRASH(("Ran out of pathfind cell infos in arena."));
	}
	return info;
}

void PathfindCellInfo::releaseACellInfo(PathfindCellInfo *theInfo)
{
	DEBUG_ASSERTCRASH(!theInfo->m_isFree, ("Shouldn't be free."));
	s_cellInfoArena.free(theInfo);
}

Int PathfindCellInfo::getCellInfoPeakLive()
{
	return s_cellInfoArena.getPeakLive();
}

Int PathfindCellInfo::getCellInfoLiveCount()
{
	return s_cellInfoArena.getLiveCount();
}

Int PathfindCellInfo::getCellInfoAllocFailures()
{
	return s_cellInfoArena.getAllocFailures();
}

Int PathfindCellInfo::getCellInfoPoolCapacity()
{
	return s_cellInfoArena.getPoolCapacity();
}

Bool PathfindCellList::canReverseSort(PathfindCell& currentCell) const
{
	if (m_head && m_tail)
		return m_head->getTotalCostDifference(currentCell) > m_tail->getTotalCostDifference(currentCell);

	return false;
}
