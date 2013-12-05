/////////////////////////////////////////////////////////////////////////////
//! \file delay.h       Simple delay block interface
//
// Copyright (c) 1998-2004 Petr Peringer
//
// This library is licensed under GNU Library GPL. See the file COPYING.
//

//
//  This is the interface for delay blocks
//  
//  WARNING: needs some testing --- this is the prototype
//

#ifndef __SIMLIB__
#   error "delay.h: 16: you should include simlib.h first"
#endif
#if __SIMLIB__ < 0x0213 
#   error "delay.h: 19: requires SIMLIB version 2.13 and higher"
#endif

namespace simlib3 {

////////////////////////////////////////////////////////////////////////////
//  class Delay --- continuous signal delay blocks
//
class Delay : public aContiBlock1 {
    Delay(const Delay&);                 // disable copy ctor
    void operator= (const Delay&);       // disable assignment
  protected: // status
  public:  //## repair
#ifdef SIMLIB_public_Delay_Buffer
    struct Buffer { 		// INTERFACE: memory for delayed signal
        virtual void put(double value, double time) = 0;
        virtual double get(double time) = 0; // with interpolation
        virtual void clear() = 0; // initialize buffer
        virtual ~Buffer() {}; 
    };
#else
    struct Buffer; 		// memory for delayed signal
#endif
  protected: //## repair
    double last_time;           // last output (for optimization)
    double last_value;
    Buffer *buffer;		// memory for past values
  protected: // parameters
    double dt;                  // delay time (should be > MaxStep)???###
    double initval;             // initial value
  public: // interface
    Delay(Input i, double dt, double initvalue=0); // dt > MaxStep
    ~Delay(); 
    void Init();		// initialize delay block
    void Sample();              // sample input (called automatically)
    virtual double Value();     // output of delay block
    double Set(double newDT);   // change delay time (EXPERIMENTAL)
}; // class Delay

}

// end

