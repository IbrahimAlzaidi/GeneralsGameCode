#include "PreRTS.h"

#include "GameLogic/AIPathfind.h"

#include "Common/GlobalData.h"
#include "Common/PerfTrace.h"

#include "GameLogic/AI.h"
#include "GameLogic/GameLogic.h"
#include "GameLogic/Module/AIUpdate.h"
#include "GameLogic/Object.h"

static UnsignedInt getPerfTraceFrame()
{
	return TheGameLogic ? TheGameLogic->getFrame() : 0;
}

static PerfTrace::PathRequestClass getPathRequestClass(const AIUpdateInterface *ai)
{
	if (ai == nullptr)
	{
		return PerfTrace::PATH_REQUEST_EXPLICIT_MOVE;
	}
	if (ai->isSafePath())
	{
		return PerfTrace::PATH_REQUEST_SAFE_PATH_OR_AUTONOMOUS;
	}
	if (ai->isAttackPath())
	{
		return PerfTrace::PATH_REQUEST_ATTACK_MOVE_OR_ATTACK_PATH;
	}
	if (ai->isApproachPath() || ai->isBlockedAndStuck())
	{
		return PerfTrace::PATH_REQUEST_PATCH_OR_REPATH;
	}
	return PerfTrace::PATH_REQUEST_EXPLICIT_MOVE;
}

static Int getPathRequestPriority(const PerfTrace::PathRequestClass requestClass)
{
	switch (requestClass)
	{
		case PerfTrace::PATH_REQUEST_EXPLICIT_MOVE: return 3;
		case PerfTrace::PATH_REQUEST_ATTACK_MOVE_OR_ATTACK_PATH: return 2;
		case PerfTrace::PATH_REQUEST_PATCH_OR_REPATH: return 1;
		case PerfTrace::PATH_REQUEST_SAFE_PATH_OR_AUTONOMOUS: return 0;
		default: return 0;
	}
}

Bool Pathfinder::isHighPriorityRequestClass(UnsignedByte requestClassByte)
{
	const PerfTrace::PathRequestClass rc = static_cast<PerfTrace::PathRequestClass>(requestClassByte);
	return rc == PerfTrace::PATH_REQUEST_EXPLICIT_MOVE || rc == PerfTrace::PATH_REQUEST_ATTACK_MOVE_OR_ATTACK_PATH;
}

static Coord3D getPathRequestDestination(const AIUpdateInterface *ai)
{
	Coord3D dest;
	dest.zero();
	if (ai != nullptr)
	{
		dest = *ai->getRequestedDestination();
	}
	return dest;
}

static UnsignedInt hashPathRequestValue(UnsignedInt seed, UnsignedInt value)
{
	return (seed ^ value) * 16777619u;
}

static UnsignedInt getPathRequestGoalFingerprint(const AIUpdateInterface *ai)
{
	UnsignedInt hash = 2166136261u;
	if (ai == nullptr)
	{
		return hash;
	}

	const Coord3D *primary = ai->getRequestedDestination();
	const Coord3D *secondary = ai->getRequestedDestination2();
	const Int primaryX = REAL_TO_INT_FLOOR(primary->x / PATHFIND_CELL_SIZE_F);
	const Int primaryY = REAL_TO_INT_FLOOR(primary->y / PATHFIND_CELL_SIZE_F);
	const Int secondaryX = REAL_TO_INT_FLOOR(secondary->x / PATHFIND_CELL_SIZE_F);
	const Int secondaryY = REAL_TO_INT_FLOOR(secondary->y / PATHFIND_CELL_SIZE_F);

	hash = hashPathRequestValue(hash, static_cast<UnsignedInt>(primaryX));
	hash = hashPathRequestValue(hash, static_cast<UnsignedInt>(primaryY));
	hash = hashPathRequestValue(hash, static_cast<UnsignedInt>(secondaryX));
	hash = hashPathRequestValue(hash, static_cast<UnsignedInt>(secondaryY));
	hash = hashPathRequestValue(hash, static_cast<UnsignedInt>(ai->getRequestedVictimID()));
	return hash;
}

static UnsignedInt getPathRequestLocomotorFingerprint(const Object *obj, const AIUpdateInterface *ai)
{
	UnsignedInt hash = 2166136261u;
	hash = hashPathRequestValue(hash, static_cast<UnsignedInt>(obj ? obj->getLayer() : LAYER_GROUND));
	if (ai != nullptr)
	{
		hash = hashPathRequestValue(hash, static_cast<UnsignedInt>(ai->getCurLocomotorSetType()));
		hash = hashPathRequestValue(hash, static_cast<UnsignedInt>(ai->getLocomotorSet().getValidSurfaces()));
	}
	return hash;
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

//----- Queue Metrics & Helpers -----

Int Pathfinder::getQueuedPathRequestCount() const
{
	Int count = m_queuePRTail - m_queuePRHead;
	if (count < 0)
	{
		count += PATHFIND_QUEUE_LEN;
	}
	return count;
}

void Pathfinder::clearQueuedRequestSlot(Int slot)
{
	m_queuedPathfindRequests[slot] = INVALID_ID;
	m_queuedRequestFrames[slot] = 0;
	m_queuedRequestClasses[slot] = static_cast<UnsignedByte>(PerfTrace::PATH_REQUEST_EXPLICIT_MOVE);
	m_queuedRequestDestinations[slot].zero();
	m_queuedRequestGoalFingerprints[slot] = 0;
	m_queuedRequestLocomotorFingerprints[slot] = 0;
	m_queuedRequestSequences[slot] = 0;
}

void Pathfinder::removeQueuedRequestAt(Int slot)
{
	if (m_queuePRHead == m_queuePRTail)
	{
		return;
	}

	Int current = slot;
	Int next = current + 1;
	if (next >= PATHFIND_QUEUE_LEN)
	{
		next = 0;
	}

	while (next != m_queuePRTail)
	{
		m_queuedPathfindRequests[current] = m_queuedPathfindRequests[next];
		m_queuedRequestFrames[current] = m_queuedRequestFrames[next];
		m_queuedRequestClasses[current] = m_queuedRequestClasses[next];
		m_queuedRequestDestinations[current] = m_queuedRequestDestinations[next];
		m_queuedRequestGoalFingerprints[current] = m_queuedRequestGoalFingerprints[next];
		m_queuedRequestLocomotorFingerprints[current] = m_queuedRequestLocomotorFingerprints[next];
		m_queuedRequestSequences[current] = m_queuedRequestSequences[next];

		current = next;
		next++;
		if (next >= PATHFIND_QUEUE_LEN)
		{
			next = 0;
		}
	}

	clearQueuedRequestSlot(current);
	m_queuePRTail = current;
}

void Pathfinder::getQueueAgeMetrics(const UnsignedInt currentFrame, Int &oldestAll, Int &oldestExplicit) const
{
	oldestAll = 0;
	oldestExplicit = 0;

	Int slot = m_queuePRHead;
	while (slot != m_queuePRTail)
	{
		if (m_queuedPathfindRequests[slot] != INVALID_ID)
		{
			const Int age = static_cast<Int>(currentFrame - m_queuedRequestFrames[slot]);
			if (age > oldestAll)
			{
				oldestAll = age;
			}
			if (m_queuedRequestClasses[slot] == static_cast<UnsignedByte>(PerfTrace::PATH_REQUEST_EXPLICIT_MOVE)
				&& age > oldestExplicit)
			{
				oldestExplicit = age;
			}
		}

		++slot;
		if (slot >= PATHFIND_QUEUE_LEN)
		{
			slot = 0;
		}
	}
}

void Pathfinder::getGridMetrics(Int &mapCellsX, Int &mapCellsY, Int &mapCellsTotal,
	Int &logicalCellsX, Int &logicalCellsY, Int &logicalCellsTotal,
	Int &zoneBlockCountX, Int &zoneBlockCountY, Int &zoneBlockCountTotal) const
{
	mapCellsX = m_extent.hi.x >= m_extent.lo.x ? (m_extent.hi.x - m_extent.lo.x + 1) : 0;
	mapCellsY = m_extent.hi.y >= m_extent.lo.y ? (m_extent.hi.y - m_extent.lo.y + 1) : 0;
	mapCellsTotal = mapCellsX * mapCellsY;

	logicalCellsX = m_logicalExtent.hi.x >= m_logicalExtent.lo.x ? (m_logicalExtent.hi.x - m_logicalExtent.lo.x + 1) : 0;
	logicalCellsY = m_logicalExtent.hi.y >= m_logicalExtent.lo.y ? (m_logicalExtent.hi.y - m_logicalExtent.lo.y + 1) : 0;
	logicalCellsTotal = logicalCellsX * logicalCellsY;

	zoneBlockCountX = mapCellsX > 0 ? ((mapCellsX + PathfindZoneManager::ZONE_BLOCK_SIZE - 1) / PathfindZoneManager::ZONE_BLOCK_SIZE) : 0;
	zoneBlockCountY = mapCellsY > 0 ? ((mapCellsY + PathfindZoneManager::ZONE_BLOCK_SIZE - 1) / PathfindZoneManager::ZONE_BLOCK_SIZE) : 0;
	zoneBlockCountTotal = zoneBlockCountX * zoneBlockCountY;
}

void Pathfinder::computeFootprintMetrics(const Object *obj, Int &sideCells, Int &areaCells, Int &radius, Bool &center)
{
	sideCells = 0;
	areaCells = 0;
	radius = 0;
	center = false;

	if (obj == nullptr)
	{
		return;
	}

	getRadiusAndCenter(obj, radius, center);
	sideCells = radius * 2 + (center ? 1 : 0);
	if (sideCells < 1)
	{
		sideCells = 1;
	}
	areaCells = sideCells * sideCells;
}

//----- Queue For Path -----

Bool Pathfinder::queueForPath(ObjectID id)
{
	const UnsignedInt frame = getPerfTraceFrame();
	Object *queuedObj = TheGameLogic ? TheGameLogic->findObjectByID(id) : nullptr;
	AIUpdateInterface *queuedAI = queuedObj ? queuedObj->getAIUpdateInterface() : nullptr;
	const PerfTrace::PathRequestClass requestClass = getPathRequestClass(queuedAI);
	const Coord3D requestedDest = getPathRequestDestination(queuedAI);
	const UnsignedInt goalFingerprint = getPathRequestGoalFingerprint(queuedAI);
	const UnsignedInt locomotorFingerprint = getPathRequestLocomotorFingerprint(queuedObj, queuedAI);

	PerfTrace::NotePathRequestAttempt(frame, requestClass);

#ifdef DEBUG_LOGGING
	{
		Object *tmpObj = queuedObj;
		if (tmpObj) {
			AIUpdateInterface *tmpAI = tmpObj->getAIUpdateInterface();
			if (tmpAI) {
				const Coord3D* pos = tmpAI->friend_getRequestedDestination();
				DEBUG_ASSERTLOG(pos->x != 0.0 && pos->y != 0.0, ("Queueing pathfind to (0, 0), usually a bug. (Unit Name: '%s', Type: '%s')", tmpObj->getName().str(), tmpObj->getTemplate()->getName().str()));
			}
		}
	}
#endif

	Int recentMatch = -1;
	for (Int recentSlot = 0; recentSlot < PATHFIND_QUEUE_LEN; recentSlot++)
	{
		if (m_recentlyServicedRequestIDs[recentSlot] != id)
		{
			continue;
		}
		if (recentMatch < 0 || m_recentlyServicedRequestFrames[recentSlot] >= m_recentlyServicedRequestFrames[recentMatch])
		{
			recentMatch = recentSlot;
		}
	}

	if (recentMatch >= 0)
	{
		const Int recentSlot = recentMatch;
		const PerfTrace::PathRequestClass recentClass =
			static_cast<PerfTrace::PathRequestClass>(m_recentlyServicedRequestClasses[recentSlot]);
		const Bool sameGoal = m_recentlyServicedGoalFingerprints[recentSlot] == goalFingerprint;
		const Bool strongerThanRecent =
			getPathRequestPriority(requestClass) > getPathRequestPriority(recentClass);
		const Bool sameFrame = m_recentlyServicedRequestFrames[recentSlot] == frame;
		const Bool coolingDown = m_recentlyServicedCooldownUntilFrames[recentSlot] > frame;

		if ((sameFrame || coolingDown) && !strongerThanRecent)
		{
			PerfTrace::NotePathRequestDuplicate(frame, requestClass);
			if (sameGoal)
			{
				PerfTrace::NoteQueueSuppressedSameGoal(frame);
			}
			else
			{
				PerfTrace::NoteQueueSuppressedWeakerThanExisting(frame);
			}
			PerfTrace::NotePathQueueDepth(frame, getQueuedPathRequestCount());
			return true;
		}
	}

	/* Check & see if we are already queued. */
	Int slot = m_queuePRHead;
	while (slot != m_queuePRTail) {
		if (m_queuedPathfindRequests[slot] == id) {
			PerfTrace::NotePathRequestDuplicate(frame, requestClass);
			const PerfTrace::PathRequestClass existingClass = static_cast<PerfTrace::PathRequestClass>(m_queuedRequestClasses[slot]);
			const Bool sameGoal = existingClass == requestClass
				&& m_queuedRequestGoalFingerprints[slot] == goalFingerprint
				&& m_queuedRequestLocomotorFingerprints[slot] == locomotorFingerprint;
			if (sameGoal) {
				PerfTrace::NoteQueueSuppressedSameGoal(frame);
			}
			else if (getPathRequestPriority(requestClass) > getPathRequestPriority(existingClass)) {
				PerfTrace::NoteQueueReplaced(frame);
				m_queuedRequestClasses[slot] = static_cast<UnsignedByte>(requestClass);
				m_queuedRequestFrames[slot] = frame;
				m_queuedRequestDestinations[slot] = requestedDest;
				m_queuedRequestGoalFingerprints[slot] = goalFingerprint;
				m_queuedRequestLocomotorFingerprints[slot] = locomotorFingerprint;
				m_queuedRequestSequences[slot] = m_nextQueuedRequestSequence++;
			}
			else {
				PerfTrace::NoteQueueSuppressedWeakerThanExisting(frame);
			}
			PerfTrace::NotePathQueueDepth(frame, getQueuedPathRequestCount());
			return true;
		}
		slot++;
		if (slot >= PATHFIND_QUEUE_LEN) {
			slot = 0;
		}
	}

	// Tail is the first available slot.
	Int nextSlot = m_queuePRTail+1;
	if (nextSlot >= PATHFIND_QUEUE_LEN) {
		nextSlot = 0;
	}
	if (nextSlot==m_queuePRHead) {
		if (getPathRequestPriority(requestClass) > 0)
		{
			Int replacementSlot = -1;
			UnsignedInt replacementSequence = 0;
			Int scanSlot = m_queuePRHead;
			while (scanSlot != m_queuePRTail)
			{
				if (m_queuedPathfindRequests[scanSlot] != INVALID_ID)
				{
					const PerfTrace::PathRequestClass queuedClass =
						static_cast<PerfTrace::PathRequestClass>(m_queuedRequestClasses[scanSlot]);
					if (getPathRequestPriority(requestClass) > getPathRequestPriority(queuedClass))
					{
						if (replacementSlot < 0 || m_queuedRequestSequences[scanSlot] < replacementSequence)
						{
							replacementSlot = scanSlot;
							replacementSequence = m_queuedRequestSequences[scanSlot];
						}
					}
				}

				scanSlot++;
				if (scanSlot >= PATHFIND_QUEUE_LEN)
				{
					scanSlot = 0;
				}
			}

			if (replacementSlot >= 0)
			{
				PerfTrace::NoteQueueReplaced(frame);
				m_queuedPathfindRequests[replacementSlot] = id;
				m_queuedRequestFrames[replacementSlot] = frame;
				m_queuedRequestClasses[replacementSlot] = static_cast<UnsignedByte>(requestClass);
				m_queuedRequestDestinations[replacementSlot] = requestedDest;
				m_queuedRequestGoalFingerprints[replacementSlot] = goalFingerprint;
				m_queuedRequestLocomotorFingerprints[replacementSlot] = locomotorFingerprint;
				m_queuedRequestSequences[replacementSlot] = m_nextQueuedRequestSequence++;
				PerfTrace::NotePathRequestEnqueued(frame, requestClass);
				PerfTrace::NotePathQueueDepth(frame, getQueuedPathRequestCount());
				return true;
			}
		}

		PerfTrace::NotePathRequestFailed(frame, requestClass);
		DEBUG_CRASH(("Ran out of pathfind queue slots."));
		return false;
	}
	m_queuedPathfindRequests[m_queuePRTail] = id;
	m_queuedRequestFrames[m_queuePRTail] = frame;
	m_queuedRequestClasses[m_queuePRTail] = static_cast<UnsignedByte>(requestClass);
	m_queuedRequestDestinations[m_queuePRTail] = requestedDest;
	m_queuedRequestGoalFingerprints[m_queuePRTail] = goalFingerprint;
	m_queuedRequestLocomotorFingerprints[m_queuePRTail] = locomotorFingerprint;
	m_queuedRequestSequences[m_queuePRTail] = m_nextQueuedRequestSequence++;
	m_queuePRTail = nextSlot;
	PerfTrace::NotePathRequestEnqueued(frame, requestClass);
	PerfTrace::NotePathQueueDepth(frame, getQueuedPathRequestCount());
	return true;
}
