/////////////////////////////////////////////////////////////////////////////
//! \file  print.cc  Printing - like printf 
//
// Copyright (c) 1991-2004 Petr Peringer 
//
// This library is licensed under GNU Library GPL. See the file COPYING.
//

//
// implementation of direct output to user
//

#include "simlib.h"
#include "internal.h"

#include <cstdio>
#include <cstdarg>

////////////////////////////////////////////////////////////////////////////
// implementation
//

namespace simlib3 {

SIMLIB_IMPLEMENTATION;


////////////////////////////////////////////////////////////////////////////
//  OutFile
//

static FILE *OutFile = stdout;

////////////////////////////////////////////////////////////////////////////
//  SetOutput
//
void SetOutput(const char *name)
{
  if (name)
  {
    if(OutFile!=stdout)
      fclose(OutFile); // added 940507
    OutFile = fopen(name,"wt");
    if(!OutFile) OutFile = stdout;
  }
}

////////////////////////////////////////////////////////////////////////////
//  _Print
//
int _Print(const char *fmt, ...)
{
   va_list argptr;
   int cnt;

   va_start(argptr, fmt);
   cnt = vfprintf(OutFile, fmt, argptr);
   va_end(argptr);

   fflush(OutFile);             // added 24.5.1995

   if (OutFile!=stdout) {
       // copy the same output to stderr
       va_start(argptr, fmt);
       cnt = vfprintf(stderr, fmt, argptr);
       va_end(argptr);
   }

   return(cnt);
}

////////////////////////////////////////////////////////////////////////////
//  Print
//
int Print(const char *fmt, ...)
{
   va_list argptr;
   int cnt;

   va_start(argptr, fmt);
   cnt = vfprintf(OutFile, fmt, argptr);
   fflush(OutFile);             // added 24.5.1995
   va_end(argptr);

   return(cnt);
}

int Print(const double x)
{
  return Print(" %g ", x);
}

int Print(const double x, const double y)
{
  return Print(" %g %g ", x, y);
}

int Print(const double x, const double y, const double z)
{
  return Print(" %g %g %g ", x, y, z);
}

////////////////////////////////////////////////////////////////////////////
//  Error - print message & end of program
//
void Error(const char *fmt, ...)
{
  va_list argptr;

  va_start(argptr, fmt);
  vfprintf(OutFile, fmt, argptr);
  fflush(OutFile);             // added 24.5.1995
  if (OutFile!=stdout)
    vfprintf(stderr, fmt, argptr);
  va_end(argptr);

  _Print("\n");
  SIMLIB_error(UserError);
}

} // end

