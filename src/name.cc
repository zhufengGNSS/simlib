/////////////////////////////////////////////////////////////////////////////
//! \file name.cc  object-name relation
//
// Copyright (c) 1991-2004 Petr Peringer 
//
// This library is licensed under GNU Library GPL. See the file COPYING.
//

//
// All SimObject instance can have name associated.
// Exported functions:
//
//  void SetName(SimObject *o, char *name) -- name of object o
//  void SetName(SimObject &o, char *name)
//  void RemoveName(SimObject *o)          -- remove name of o
//  void RemoveName(SimObject &o)
//  const char *GetName(SimObject *o)      -- get name of o
//  const char *GetName(SimObject &o)
//


////////////////////////////////////////////////////////////////////////////
//  interface
//

#include "simlib.h"
#include "internal.h"

////////////////////////////////////////////////////////////////////////////
//  implementation
//

#include <cstdarg> // ...
#include <cstdio>  // vsprintf()

namespace simlib3 {

SIMLIB_IMPLEMENTATION;

/////////////////////////////////////////////////////////////////////////////
/// assign name to object
void SetName(SimObject & o, const char *name)
{
    o.SetName(name);
}

/// assign name to object
void SetName(SimObject * o, const char *name)
{
    o->SetName(name);
}

/// remove name
void RemoveName(SimObject & o)
{
    o.SetName(0);
}

/// remove name
void RemoveName(SimObject * o)
{
    o->SetName(0);
}

/// get name of object
const char *GetName(SimObject & o)
{
    return o.Name();
}

/// get name of object
const char *GetName(SimObject * o)
{
    return o->Name();
}

// TODO: this is insecure
/// function to create temporary name in special static buffer
const char *SIMLIB_create_name(const char *fmt, ...)
{
    const int number = 4;
    static char s[number][128];
    static int index = 0;
    int i = index;
    index = (index+1) % number;

    va_list va;
    va_start(va, fmt);
    vsprintf(s[i], fmt, va);
    va_end(va);

    return s[i];
}

} // namespace

