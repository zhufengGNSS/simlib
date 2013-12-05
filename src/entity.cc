/////////////////////////////////////////////////////////////////////////////
//! \file entity.cc  Active entities base class
//
// Copyright (c) 1991-2004 Petr Peringer 
//
// This library is licensed under GNU Library GPL. See the file COPYING.
//

/// \module entity
/// base class of the entity hierarchy (Process, Event)
///

////////////////////////////////////////////////////////////////////////////
// interface
//

#include "simlib.h"
#include "internal.h"

////////////////////////////////////////////////////////////////////////////
// implementation
//

namespace simlib3 {

SIMLIB_IMPLEMENTATION;

// TODO: remove
#define CHECKENTITY(e) if (!e) SIMLIB_error(EntityRefError);
#define CHECKQUEUE(q) if (!q) SIMLIB_error(QueueRefError);

/// current number of entities in model
static unsigned long SIMLIB_Entity_Count = 0L; // # of entities in model
/// serial number of created entity
unsigned long Entity::_Number = 0L;     // # of entity creations

////////////////////////////////////////////////////////////////////////////
///  constructor
Entity::Entity(Priority_t p) :
  _Ident(SIMLIB_Entity_Count++), // unique identification
  _MarkTime(0.0), 
  _SPrio(0),
  Priority(p),
  _evn(0) // pointer to calendar item
{
  _Number++;                      // # of entities
  Dprintf(("Entity#%lu{%p}::Entity(%d)", _Ident, this, p));
}

////////////////////////////////////////////////////////////////////////////
///  destructor
Entity::~Entity() {
  Dprintf(("Entity#%lu{%p}::~Entity()", _Ident, this));
  if (!Idle()) {
    SQS::Get(this);           // remove from calendar
//  _warning(DeletingActive); // TODO:can be important? if sim SIMLIB_error else _warn
  }
  _Number--;                 // # of entities in model
}

////////////////////////////////////////////////////////////////////////////
///  entity activation at current Time
void Entity::Activate()      // activate now
{
  Dprintf(("%s.Activate()", Name()));
  Activate(Time);
}

///  entity activation at given time
void Entity::Activate(double t)
{
  Dprintf(("%s.Activate(%g)", Name(), t));
  CHECKENTITY(this);
  if (!Idle()) {                // rescheduling
    SQS::Get(this);             // remove from calendar
  }
  SQS::ScheduleAt(this,t);
}


////////////////////////////////////////////////////////////////////////////
//  Wait - wait for dtime
//
//void Entity::Wait(double dtime)
//{
//  Dprintf(("%s.Wait(%g)",Name(),dtime));
//  Activate(double(Time)+dtime);  // scheduling
//}

////////////////////////////////////////////////////////////////////////////
//  Passivate - deactivation of process (entity)
//
void Entity::Passivate()
{
  Dprintf(("%s.Passivate()",Name()));
  CHECKENTITY(this);
  if(!Idle())           // if scheduled
      SQS::Get(this);   // remove from calendar
}


////////////////////////////////////////////////////////////////////////////
// Into --- inserting into queue (TODO: problem with continuation)
//
#if 0
void Entity::Into(Queue *q)
{
  CHECKQUEUE(q);
  if(where()!=0) Out(); // if already in queue then remove ### +warning ???
  q->Insert(this);      // insert
}
#endif 

////////////////////////////////////////////////////////////////////////////
//  Out - leaving queue
//
void Entity::Out()
{
  Link::Out(); // equal
}

////////////////////////////////////////////////////////////////////////////
///  passivate and destroy entity
void Entity::Terminate()
{
  Dprintf(("%s.Terminate()",Name()));
  if(!Idle()) 
      SQS::Get(this);  // remove from calendar

  if(isAllocated() && this != SIMLIB_Current)
      delete this;     // destroy entity (if not currently running Behavior)
}

////////////////////////////////////////////////////////////////////////////
/// get name - either explicit or default "Entity#{ptr}"
const char *Entity::Name() const
{
  const char *name = SimObject::Name();
  if(*name) return name; // return explicit name
  else      return SIMLIB_create_name("Entity%lu{%p}", _Ident, this);
}

} // namespace

// end

