// ----------------------------------------------------------------------------
// Copyright 2006-2007, Martin D. Flynn
// All rights reserved
// ----------------------------------------------------------------------------
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ----------------------------------------------------------------------------
// Description:
//  Packet queue manager.
//  Circular buffer for managing queued packets.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/08/01  Martin D. Flynn
//     -Added "pqueHasUnsentPacket" function
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Dropped support for non-malloc'ed event queues (all current reference
//      implementation platforms support 'malloc')
// ----------------------------------------------------------------------------

#include "defaults.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include "log.h"

#include "packet.h"
#include "pqueue.h"

#include "io.h"
#include "strtools.h"

// ----------------------------------------------------------------------------

#ifdef PQUEUE_THREAD_LOCK
#include "threads.h"
#define QUEUE_LOCK_INIT(Q)  threadMutexInit(&((Q)->queMutex));
#define QUEUE_LOCK(Q)       MUTEX_LOCK(&((Q)->queMutex));
#define QUEUE_UNLOCK(Q)     MUTEX_UNLOCK(&((Q)->queMutex));
#else
#define QUEUE_LOCK_INIT(Q)  // implement queue lock init here
#define QUEUE_LOCK(Q)       // implement queue lock here
#define QUEUE_UNLOCK(Q)     // implement queue unlock here
#endif

#define PACKET_STATUS_FILLED	1
#define PACKET_STATUS_PRESERVED	2
#define PACKET_STATUS_SENT		8
// ----------------------------------------------------------------------------
static char event_queue_backup_file[64] = EVENT_QUEUE_BACKUP_NAME;
static bool preserve_restored = false;
static bool preserve_preserved = false;
static FILE *pqfs = NULL;
// ----------------------------------------------------------------------------
static void pqueDownsizeQueue(PacketQueue_t *pq);

/* initialize packet queue */
void pqueInitQueue(PacketQueue_t *pq, int queSize)
{
	int i;
	if (pq) {
		memset(pq, 0, sizeof(PacketQueue_t));
		QUEUE_LOCK_INIT(pq)
		pq->queFirst = 0L;
		pq->queLast  = 0L;
		pq->queSize = PACKETS_PER_PAGE;
		pq->expandable = (queSize > PACKETS_PER_PAGE)? utTrue : utFalse;
		pq->queOverwrite = utTrue; // overwrites allowed by default
		for (i = 0; i < PAGE_ARRAY_SIZE; i++)
			pq->queue[i] = NULL;
		pq->queue[0] = (Packet_t*)malloc(sizeof(Packet_t) * PACKETS_PER_PAGE);
		memset(pq->queue[0], 0, sizeof(Packet_t) * PACKETS_PER_PAGE);
	}
}

/* reset queue to 'empty' state */
void pqueResetQueue(PacketQueue_t *pq)
{
	if (pq->queFirst == 0 && pq->queLast == 0)
		return;
	QUEUE_LOCK(pq) {
	if (pq->expandable && pq->queSize > PACKETS_PER_PAGE)
		pqueDownsizeQueue(pq);
	else {
		pq->queFirst = 0L;
		pq->queLast  = 0L;
		pq->queOverwrite = utTrue;
		pq->queSize = PACKETS_PER_PAGE;
    }
	memset(pq->queue[0], 0, sizeof(Packet_t) * PACKETS_PER_PAGE);
	} QUEUE_UNLOCK(pq)
}
/* release all allocated memory */
void pqueReleaseQueue(PacketQueue_t *pq)
{
	int i;
    if (pq) {
	QUEUE_LOCK(pq) {
	for (i = 0; i < PAGE_ARRAY_SIZE; i++) {
		if (pq->queue[i] != NULL)	
			free(pq->queue[i]);
		else
			break;
	}
        } QUEUE_UNLOCK(pq)
    }
}

/* expand queue to one page bigger */
utBool expandQueue(PacketQueue_t *pq)
{
	int page_num;
	utBool success = utFalse;
	if (pq->queSize < PAGE_ARRAY_SIZE * PACKETS_PER_PAGE) {
		page_num = pq->queSize >> PACKET_INDEX_BITS;
		pq->queue[page_num] = malloc(sizeof(Packet_t) * PACKETS_PER_PAGE);
		if (pq->queue[page_num] != NULL) {
			memset(pq->queue[page_num], 0, sizeof(Packet_t) * PACKETS_PER_PAGE);
			pq->queSize += PACKETS_PER_PAGE;
			success = utTrue;
		}
	}
	return success;
}

/* enable/disable overwrite of oldest entry */
void pqueEnableOverwrite(PacketQueue_t *pq, utBool overwrite)
{
    if (pq) {
        QUEUE_LOCK(pq) {
            pq->queOverwrite = overwrite;
        } QUEUE_UNLOCK(pq)
    }
}

// ----------------------------------------------------------------------------
/* return the queue index of the next item in the list following the specified index */
static Int32 _pqueNextIndex(PacketQueue_t *pq, Int32 ndx)
{
    return ((ndx + 1L) < pq->queSize)? (ndx + 1L) : 0L;
}
/* return the queue index of the prior item in the list preceeding the specified index */
static Int32 _pquePriorIndex(PacketQueue_t *pq, Int32 ndx)
{
    return ((ndx - 1L) < 0L)? (pq->queSize - 1L) : (ndx - 1L);
}
static Packet_t *_pqueGetPacketAt(PacketQueue_t *pq, Int32 entry)
{
	int page_index, pkt_num;
	page_index = (entry >> PACKET_INDEX_BITS) & PAGE_INDEX_MASK;
	pkt_num = entry & PACKET_INDEX_MASK;
	return (pq->queue[page_index] + pkt_num);
    //return &(pq->queue[entry]); <-- non-malloc'ed
}

/* return true if the queue is non-empty */
utBool pqueHasPackets(PacketQueue_t *pq)
{
    Int32 len = 0L;
	QUEUE_LOCK(pq) {
	if (pq->queLast >= pq->queFirst) {
		len = pq->queLast - pq->queFirst;
	} else {
		len = pq->queSize - (pq->queFirst - pq->queLast);
	}
	} QUEUE_UNLOCK(pq)
    return ((len > 0)? utTrue : utFalse);
}

/* return the number of current entries in the queue */
Int32 pqueGetPacketCount(PacketQueue_t *pq)
{
	Int32 m, cnt = 0;
	Packet_t *pkt;
	QUEUE_LOCK(pq) {
	m = pq->queFirst;
	while (m != pq->queLast) {
		pkt = _pqueGetPacketAt(pq,m);
		if (pkt->status != 0)
			++cnt;
		m = _pqueNextIndex(pq, m);
	}
	} QUEUE_UNLOCK(pq)
    return cnt;
}

// ----------------------------------------------------------------------------

static void _pqueFreePacketAt(PacketQueue_t *pq, Int32 entry)
{
	Packet_t *pkt_queue = _pqueGetPacketAt(pq, entry);		
	pkt_queue->status = 0;
}

static void _pqueSetPacketAt(PacketQueue_t *pq, Int32 entry, Packet_t *pkt)
{
	Packet_t *pkt_queue = _pqueGetPacketAt(pq, entry);		
	memcpy(pkt_queue, pkt, sizeof(Packet_t));
}

// ----------------------------------------------------------------------------
/* allocate an entry from the queue */
static Int32 _pqueAllocateNextEntry(PacketQueue_t *pq)
{
    Int32 newLast;
    Int32 newEntry = pq->queLast;
    
	newLast  = _pqueNextIndex(pq, newEntry);
	if ((pq->expandable) && (newLast == 0)) {
		if (!expandQueue(pq)) {
			logWARNING(LOGSRC,"Packet queue overflow !");
		} else {
			newLast  = _pqueNextIndex(pq, newEntry);
		}
	}
    if (newLast == pq->queFirst) {
        // We've run out of space in the queue
        if (pq->queOverwrite) {
            pq->queFirst = _pqueNextIndex(pq, pq->queFirst);
        } else {
            // overwrites not allowed, the newest entry is ignored
            logWARNING(LOGSRC,"Packet queue overflow !");
            return -1L;
        }
    }
    pq->queLast = newLast;
    return newEntry ;
}

/* add (copy) the specified packet to the queue */
utBool pqueAddPacket(PacketQueue_t *pq, Packet_t *pkt)
{
    utBool didAdd = utFalse;
    if (pq && pkt) {
		QUEUE_LOCK(pq) {
		Int32 entry = _pqueAllocateNextEntry(pq);
		if (entry >= 0L) {
			pkt->status = PACKET_STATUS_FILLED;
			_pqueSetPacketAt(pq, entry, pkt);
			didAdd = utTrue;
		}
		} QUEUE_UNLOCK(pq)
    }
    return didAdd;
}

/* return true if the queue contains any unsent Packet entries */
// need to optimize this by just checking the last packet
utBool pqueHasUnsentPacket(PacketQueue_t *pq)
{
    utBool found = utFalse;
	Int32 m;
	Packet_t *pkt;
	QUEUE_LOCK(pq) {
	m = pq->queFirst;
	while (m != pq->queLast) {
		pkt = _pqueGetPacketAt(pq,m);
		if ((pkt->status & PACKET_STATUS_FILLED) && !(pkt->status & PACKET_STATUS_SENT)) {
			found = utTrue;
			break;
		}
		m = _pqueNextIndex(pq, m);
	}
	} QUEUE_UNLOCK(pq)
    return found;
}

// ----------------------------------------------------------------------------
/* restore any sent packets to status filled */
utBool pqueRestoreSentPacket(PacketQueue_t *pq)
{
    utBool found = utFalse;
	Packet_t *pkt;
	QUEUE_LOCK(pq) {
	Int32 m = pq->queFirst;
	while (m != pq->queLast) {
		pkt = _pqueGetPacketAt(pq,m);
		if (pkt->status & PACKET_STATUS_SENT) {
			pkt->status &= ~PACKET_STATUS_SENT;
			found = utTrue;
		}
		m = _pqueNextIndex(pq, m);
	}
	} QUEUE_UNLOCK(pq)
    return found;
}

// ----------------------------------------------------------------------------
/* return the last sequence in the queue */ 
UInt32 pqueGetLastSequence(PacketQueue_t *pq, time_t * timestamp)
{
    UInt32 seq = SEQUENCE_ALL;
	Packet_t *pkt;
	long t12;
	QUEUE_LOCK(pq)
	if (pq->queFirst != pq->queLast) {
		pkt = _pqueGetPacketAt(pq, _pquePriorIndex(pq, pq->queLast));
		seq = pkt->sequence;
		t12 = (pkt->data[2] << 24) | (pkt->data[3] << 16) | (pkt->data[4] << 8) | pkt->data[5];
		printf("Time of the last stored event (%d): %lld\n", pq->queLast, (long long) t12);
		if (t12 < NEW_MILLENNIUM)
			*timestamp = t12;
    }
	QUEUE_UNLOCK(pq)
    return seq;
}
/* True if the target sequence is withing the sent packets range */
utBool pqueDeleteSentPackets(PacketQueue_t *pq, UInt32 sequence, int total)
{
	int m, i = 0, j = -1, h = -1;
	Packet_t *pkt;

	QUEUE_LOCK(pq)
	m = pq->queFirst;
	pkt = _pqueGetPacketAt(pq, m);
	if (!((pkt->status & PACKET_STATUS_SENT) && (pkt->sequence == sequence)))
		h = m;
	else if (pkt->sequence < sequence) {
		QUEUE_UNLOCK(pq)
		return utFalse;
	}
	while ((i < total) && (m != pq->queLast)) {
		if ((pkt->status & PACKET_STATUS_SENT) && (pkt->sequence >= sequence)) {
			_pqueFreePacketAt(pq, pq->queFirst);
			if (i++ == 0)
				j = m;
		}
		m = _pqueNextIndex(pq, m);
		pkt = _pqueGetPacketAt(pq, m);
	}
	if (i > 0) {
		if (h < 0 || h == j) 
			pq->queFirst = m;
		else if (m == pq->queLast)
			pq->queLast = j;
		QUEUE_UNLOCK(pq)
		return utTrue;
	}
	else {
		QUEUE_UNLOCK(pq)
		return utFalse;
	}
}

// ----------------------------------------------------------------------------
/* return the highest priority found in the queue */
PacketPriority_t pqueGetHighestPriority(PacketQueue_t *pq)
{
    PacketPriority_t maxPri = PRIORITY_NONE;
    if (pq) {
        QUEUE_LOCK(pq) {
            Int32 m = pq->queFirst;
            while (m != pq->queLast) {
                Packet_t *pkt = _pqueGetPacketAt(pq,m);
                if (pkt->priority > maxPri) {
                    maxPri = pkt->priority;
                }
                m = _pqueNextIndex(pq, m);
            }
        } QUEUE_UNLOCK(pq)
    }
    return maxPri;
}
// ----------------------------------------------------------------------------
#if 0
void pquePrintQueue(PacketQueue_t *pq)
{
    if (pq) {
        int i;
        char m[10];
        PacketQueueIterator_t iter;
        pqueGetIterator(pq, &iter);
        for (i = 0;;i++) {
            Packet_t *pkt = pqueGetNextPacket((Packet_t*)0, &iter);
            if (!pkt) { break; }
            sprintf(m, "%d", i);
            pktPrintPacket(pkt, m, ENCODING_CSV);
        }
    }
}

#endif
// ----------------------------------------------------------------------------

/* initialize and return an interator for the queue */
PacketQueueIterator_t *pqueGetIterator(PacketQueue_t *pq, PacketQueueIterator_t *i)
{
    i->pque  = pq;
	i->index = pq->queFirst;
    return i;
}

/* return true if the iterator has at least one more entry available */
utBool pqueHasNextPacket(PacketQueueIterator_t *i)
{
    utBool rtn = utFalse;
    if (i) {
        PacketQueue_t *pq = i->pque;
        QUEUE_LOCK(pq) {
            Int32 n = i->index;
            if (n < 0L) {
                n = pq->queFirst;
            } else
            if (n != pq->queLast) {
                n = _pqueNextIndex(pq, n);
            }
            rtn = (n != pq->queLast)? utTrue : utFalse;
        } QUEUE_UNLOCK(pq)
    }
    return rtn;
}

/* mark the packet as SENT */
void pqueMarkPacketSent(Packet_t *pkt)
{
	pkt->status |= PACKET_STATUS_SENT;
}
/* mark the packet as SENT */
utBool pqueIsPacketSent(Packet_t *pkt)
{
	return ((pkt->status & PACKET_STATUS_SENT)? utTrue : utFalse);
}
/* return the next Packet in the queue as specified by the iterator, or return
** null if there are no more entries in the queue */
Packet_t *pqueGetNextPacket(Packet_t *pktCopy, PacketQueueIterator_t *i)
{
	Packet_t *pkt = (Packet_t*)0;
	Int32	new_index;
	PacketQueue_t *pq = i->pque;
	QUEUE_LOCK(pq) {
	if (i->index == pq->queLast) {
		QUEUE_UNLOCK(pq)
		return pkt;
	}
	pkt = _pqueGetPacketAt(pq, i->index);
	new_index = _pqueNextIndex(pq, i->index);
	while (new_index != pq->queLast && pkt->status == 0) {
		pkt = _pqueGetPacketAt(pq, new_index);
		new_index = _pqueNextIndex(pq, new_index);
	}
	if (pkt->status == 0) {
		QUEUE_UNLOCK(pq)
		return (Packet_t*)0;
	}
	i->index = new_index;
	if (pktCopy) {
		pktCopy->sequence = pkt->sequence;
		pktCopy->hdrType = pkt->hdrType;
		pktCopy->priority = pkt->priority;
		pktCopy->status = pkt->status;
		pktCopy->seqPos = pkt->seqPos;
		pktCopy->seqLen = pkt->seqLen;
		pktCopy->dataLen = pkt->dataLen;
		strncpy(pktCopy->dataFmt, pkt->dataFmt, sizeof(pkt->dataFmt));
		memcpy(pktCopy->data, pkt->data, pkt->dataLen);
	}
	} QUEUE_UNLOCK(pq)
	return pkt;
}
// ----------------------------------------------------------------------------
void pqueDownsizeQueue(PacketQueue_t *pq)
{
	int i;
	for (i = ((pq->queSize >> PACKET_INDEX_BITS) & PAGE_INDEX_MASK) - 1; i > 0; i--) {
		free(pq->queue[i]);
		pq->queue[i] = NULL;
	}
	pq->queFirst = 0;
	pq->queLast = 0;
	pq->queSize = PACKETS_PER_PAGE;
}
/* save queue to file */
int pquePreserveQueue(PacketQueue_t *pq)
{
printf("%s: pq->queLast = %ld\n", __FUNCTION__, pq->queLast);
	Int32 m, n = 0, ret = -1;
	size_t pkt_len;
	Packet_t *pkt;
	if (pqfs == NULL) {
		if ((pqfs = fopen(event_queue_backup_file, "a+")) == NULL) {
			perror("Creating event backup file");
			return -1;
		}
	}
	QUEUE_LOCK(pq)
	pkt_len = sizeof(Packet_t);
	m = pq->queFirst;
	while (m != pq->queLast) {
		pkt = _pqueGetPacketAt(pq,m);
		if (!(pkt->status & PACKET_STATUS_PRESERVED)) {
			pkt->status |= PACKET_STATUS_PRESERVED;
			if (fwrite(pkt, pkt_len, 1, pqfs) < 1)
				goto exit;
			n++;
		}
		m = _pqueNextIndex(pq, m);
	}
	ret = 0;
exit:
	QUEUE_UNLOCK(pq)
	if (n > 0) {
		preserve_preserved = true;
		fflush(pqfs);
	}
    return ret;
}
/* restore queue from file */
/*return: 0:nothing restored; -1:error; n > 0:restored*/
int pqueRestoreQueue(PacketQueue_t *pq)
{
	int entry, n = 0;
	long sz1, start1;
	size_t pkt_len;
	Packet_t pkt1;
	FILE *f1;
	if ((f1 = fopen(event_queue_backup_file, "r")) == NULL) {
		if (errno == ENOENT)
			return 0;
		else
			return -1;
	}
	if ((fseek(f1, 0, SEEK_END) == 0)) {
		if ((sz1 = ftell(f1)) <= 0) {
			fclose(f1);
			return 0;
		}
		if (sz1 > PRESERVE_RESTORE_SIZE * sizeof(Packet_t))
			start1 = sz1 - PRESERVE_RESTORE_SIZE * sizeof(Packet_t);
		else
			start1 = 0;
		fseek(f1, start1, SEEK_SET);
	} else
		return -1;
	QUEUE_LOCK(pq)
	pkt_len = sizeof(Packet_t);
	while (!feof(f1)) {
		if (fread(&pkt1, pkt_len, 1, f1) < 1)
			break; 
		entry = _pqueAllocateNextEntry(pq);
		if (entry >= 0) {
			_pqueSetPacketAt(pq, entry, &pkt1);
			++n;
		} else
			break;
	}
	QUEUE_UNLOCK(pq)
	fclose(f1);
	preserve_restored = true;
    return n;
}
void pqueResetPreserve(void)
{
	if (preserve_preserved && (pqfs != NULL)) {
		fclose(pqfs);
		unlink(event_queue_backup_file);
		preserve_preserved = false;
		pqfs = NULL;
	}
	if (preserve_restored) {
		unlink(event_queue_backup_file);
		preserve_restored = false;
	}
}
void pqueUpdateTimestamp(PacketQueue_t *pq, long adjustment)
{
	Int32 m;
	long time_stamp;
	Packet_t *pkt;
	
	QUEUE_LOCK(pq)
	m = pq->queFirst;
	while (m != pq->queLast) {
		pkt = _pqueGetPacketAt(pq, m);
		time_stamp = (pkt->data[2] << 24) | (pkt->data[3] << 16) | (pkt->data[4] << 8) | pkt->data[5];
		if (time_stamp < NEW_MILLENNIUM) {
			time_stamp += adjustment;
			pkt->data[2] = (time_stamp >> 24) & 0xFF;
			pkt->data[3] = (time_stamp >> 16) & 0xFF;
			pkt->data[4] = (time_stamp >> 8) & 0xFF;
			pkt->data[5] = time_stamp  & 0xFF;
		}
		m = _pqueNextIndex(pq, m);
	}
	QUEUE_UNLOCK(pq)
}
void pqueTuneTimestamp(PacketQueue_t *pq, long adjustment)
{
	Int32 m;
	long time_stamp;
	Packet_t *pkt;

	QUEUE_LOCK(pq)
	m = pq->queFirst;
	while (m != pq->queLast) {
		pkt = _pqueGetPacketAt(pq, m);
		time_stamp = (pkt->data[2] << 24) | (pkt->data[3] << 16) | (pkt->data[4] << 8) | pkt->data[5];
		time_stamp += adjustment;
		pkt->data[2] = (time_stamp >> 24) & 0xFF;
		pkt->data[3] = (time_stamp >> 16) & 0xFF;
		pkt->data[4] = (time_stamp >> 8) & 0xFF;
		pkt->data[5] = time_stamp  & 0xFF;
		m = _pqueNextIndex(pq, m);
	}
	QUEUE_UNLOCK(pq)
}
