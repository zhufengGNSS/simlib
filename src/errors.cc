//
// errors.cc
//
//
//
//
/* Generated from file 'errors.txt' by program GENERR */

#include "simlib.h"
namespace simlib3 {
#include "errors.h"

static char _Errors[] = {
/* 0 */ "SIMLIB/C++ Simulation Library, "SIMLIB_COPYRIGHT"\0" 
/* 1 */ "Undocumented error\0" 
/* 2 */ "Internal error\0" 
/* 3 */ "No memory\0" 
/* 4 */ "Bad Init() arguments\0" 
/* 5 */ "Init() called twice before Run()\0" 
/* 6 */ "Init() can not be called during simulation run\0" 
/* 7 */ "Run() should be called after Init()\0" 
/* 8 */ "Bad SetStep(min,max) arguments\0" 
/* 9 */ "Requested integration step is too small\0" 
/* 10 */ "SetStep: Too big difference of min/max step\0" 
/* 11 */ "SetAccuracy: Too small relative accuracy requested\0" 
/* 12 */ "Special function called and simulation is not running\0" 
/* 13 */ "Numerical integration error greater than requested\0" 
/* 14 */ "Bad reference to list item\0" 
/* 15 */ "Deleted item is linked in some list\0" 
/* 16 */ "Removed item not in list\0" 
/* 17 */ "Calendar should be singleton\0" 
/* 18 */ "Deleting active item in calendar\0" 
/* 19 */ "Scheduling before current Time\0" 
/* 20 */ "Calendar is empty\0" 
/* 21 */ "Bad reference to process\0" 
/* 22 */ "Procesis is not initialized\0" 
/* 23 */ "Bad reference to histogram\0" 
/* 24 */ "Bad histogram step (step<=0)\0" 
/* 25 */ "Bad histogram interval count (max=10000)\0" 
/* 26 */ "Bad link to list\0" 
/* 27 */ "List does not have active item\0" 
/* 28 */ "Empty list\0" 
/* 29 */ "Bad queue reference\0" 
/* 30 */ "Empty WaitUntilList - can't Get() (internal error)\0" 
/* 31 */ "Bad graph reference\0" 
/* 32 */ "Bad entity reference\0" 
/* 33 */ "Entity not scheduled\0" 
/* 34 */ "Bad statistic object reference\0" 
/* 35 */ "Bad time statistic reference\0" 
/* 36 */ "Time statistic not initialized\0" 
/* 37 */ "Can't create new integrator in dynamic section\0" 
/* 38 */ "Can't destroy integrator in dynamic section\0" 
/* 39 */ "Can't create new status variable in dynamic section\0" 
/* 40 */ "Can't destroy status variable in dynamic section\0" 
/* 41 */ "Bad facility reference\0" 
/* 42 */ "Seize(): Can't interrupt facility service\0" 
/* 43 */ "Release(): Facility is released by other than currently serviced process\0" 
/* 44 */ "Release(): Can't release empty facility\0" 
/* 45 */ "Bad store reference\0" 
/* 46 */ "Enter() request exceeded the store capacity\0" 
/* 47 */ "Leave() leaves more than currently used\0" 
/* 48 */ "SetCapacity(): can't reduce store capacity\0" 
/* 49 */ "SetQueue(): deleted (old) queue is not empty\0" 
/* 50 */ "Weibul(): lambda<=0.0 or alfa<=1.0\0" 
/* 51 */ "Erlang(): beta<1\0" 
/* 52 */ "NegBin(): q<=0 or k<=0\0" 
/* 53 */ "NegBinM(): m<=0\0" 
/* 54 */ "NegBinM(): p not in range 0..1\0" 
/* 55 */ "Poisson(lambda): lambda<=0\0" 
/* 56 */ "Geom(): q<=0\0" 
/* 57 */ "HyperGeom(): m<=0\0" 
/* 58 */ "HyperGeom(): p not in range 0..1\0" 
/* 59 */ "Can't write output file\0" 
/* 60 */ "Output file can't be open between Init() and Run()\0" 
/* 61 */ "Can't open output file\0" 
/* 62 */ "Can't close output file\0" 
/* 63 */ "Algebraic loop detected\0" 
/* 64 */ "Parameter low>=high\0" 
/* 65 */ "Parameter of quantizer <= 0\0" 
/* 66 */ "Library and header (simlib.h) version mismatch \0" 
/* 67 */ "Semaphore::V() -- bad call\0" 
/* 68 */ "Uniform(l,h) -- bad arguments\0" 
/* 69 */ "Stat::MeanValue()  No record in statistics\0" 
/* 70 */ "Stat::Disp()  Can't compute (n<2)\0" 
/* 71 */ "AlgLoop: t_min>=t_max\0" 
/* 72 */ "AlgLoop: t0 not in  <t_min,t_max>\0" 
/* 73 */ "AlgLoop: method not convergent\0" 
/* 74 */ "AlgLoop: iteration limit exceeded\0" 
/* 75 */ "AlgLoop: iterative block is not in loop\0" 
/* 76 */ "Unknown integration method\0" 
/* 77 */ "Integration method name not unique\0" 
/* 78 */ "Integration step <=0\0" 
/* 79 */ "Start-method is not single-step\0" 
/* 80 */ "Method is not multi-step\0" 
/* 81 */ "Can't switch methods in dynamic section\0" 
/* 82 */ "Can't switch start-methods in dynamic section\0" 
/* 83 */ "Rline: argument n<2\0" 
/* 84 */ "Rline: array is not sorted\0" 
/* 85 */ "Library compiled without debugging support\0" 
/* 86 */ "Dealy is too small (<=MaxStep)\0" 
/* 87 */ "Parameter can not be changed during simulation run\0" 
/* 88 */ "General error\0" 
};

char *_ErrMsg(enum _ErrEnum N)
{
  char *p = _Errors;
  int i = N;
  while( i-- > 0 )
    while( *p++ != '\0' ) { /*empty*/ }
  return p;
};
} // namespace simlib3 

