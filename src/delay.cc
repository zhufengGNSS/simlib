/////////////////////////////////////////////////////////////////////////////
//! \file delay.cc  Delay block implementation (experimental)
//
// Copyright (c) 1991-2004 Petr Peringer 
//
// This library is licensed under GNU Library GPL. See the file COPYING.
//

//
//  This module contains implementation of continuous delay class
//
//  classes:
//     Delay -- real delay blocks
//     SIMLIB_Delay -- internal class for registration of delay blocks
//     SIMLIB_DelayBuffer -- delay memory
//
// LIMITS:  
//     dt <= MaxStep --- problem with too small delay time
//     increasing dt --- problem with buffer length 
//

////////////////////////////////////////////////////////////////////////////
// interface
//

#include "simlib.h"
#include "delay.h"
#include "internal.h"

// we use standard C++ library (STL)
#include <deque>		// for buffer implementation
#include <list>			// for list of delays


////////////////////////////////////////////////////////////////////////////
// implementation
//

namespace simlib3 {


SIMLIB_IMPLEMENTATION;

// Delay subsystem public symbols
class Delay;

// local symbols:
class SIMLIB_Delay;
class SIMLIB_DelayBuffer;


////////////////////////////////////////////////////////////////////////////
// SIMLIB_Delay --- list of all continuous delay blocks
//
class SIMLIB_Delay {
    static std::list<Delay *> *listptr;	// list of delay objects -- singleton
  public:
    static void Register(Delay *p) { 	// must be called by Delay ctr
	if( listptr == 0 ) Initialize();
	listptr->push_back(p);
    }
      static void UnRegister(Delay *p) {// must be called from Delay destructor
	listptr->remove(p);
	if( listptr->size() == 0 ) Destroy();// is really important???
    }
  private:
    static void Initialize() { 		// initialize delay subsystem
	listptr = new std::list<Delay*>();	// create new list of delays
	// install 'hooks' into simulation control algorithm:
	INSTALL_HOOK( Delay, SIMLIB_Delay::SampleAll );
	INSTALL_HOOK( DelayInit, SIMLIB_Delay::InitAll );
    }
    static void Destroy() {  	// should be called by ExitSimulation()? ###???
	delete listptr; 	// remove list 
	listptr = 0;
	// disable hooks into simulation control algorithm
	INSTALL_HOOK( Delay, 0 );
	INSTALL_HOOK( DelayInit, 0 );
    }
    // SampleAll --- function to scan inputs of all delay objects
    static void SampleAll() { 	// called each continuous step (and more)
	if( listptr == 0 ) return; 	// ### should never be reached (2remove)
        std::list<Delay *>::iterator i;
        for( i=listptr->begin(); i!=listptr->end(); i++) // for each delay object
	    (*i)->Sample();	// sample input value
    }
    // InitAll --- function to initialize all delay objects
    static void InitAll() { 	// called at Init()
	if( listptr == 0 ) return; 	// no delays ###
        std::list<Delay *>::iterator i;
        for( i=listptr->begin(); i!=listptr->end(); i++) // for each delay object
	    (*i)->Init();	// set initial value
    }
};

// static member must be initializad
std::list<Delay *> *SIMLIB_Delay::listptr = 0; 


#ifndef SIMLIB_public_Delay_Buffer
struct Delay::Buffer { 		// INTERFACE: memory for delayed signal
        virtual void put(double value, double time) = 0;
        virtual double get(double time) = 0; // with interpolation
        virtual void clear() = 0; // initialize buffer
        virtual ~Buffer() {}; 
};
#endif
/////////////////////////////////////////////////////////////////////////////
// SIMLIB_DelayBuffer --- memory for delayed pairs (Time,value)
//
// this buffer inherits interface from Delay::Buffer (we can use various
// implementations later)
// method get() does linear interpolation ??? should be split ### !!!!
//
class SIMLIB_DelayBuffer : public Delay::Buffer { // memory for delayed signal
    struct Pair { // pair (t,val) for storing in buffer
        double time;
        double value;
        Pair(double t, double v) : time(t), value(v) {}
	bool operator == (Pair &p) { return p.time==time && p.value==value; }
    };
    std::deque<Pair> buf;	// use deque -- should be good enough
//    deque<Pair>::iterator lastOK; // for optimization (if LIMIT>2)
    Pair last_insert;		// last inserted value (optimization)
 public:
    SIMLIB_DelayBuffer(): buf(), last_insert(-2,0) { /*empty*/ }
    virtual void clear() {
        last_insert = Pair(-2,0); // we need it for optimization
        buf.clear();		// empty buffer
    }
    virtual void put(double value, double time) {
        Pair p(time,value);
#ifndef NO_DELAY_OPTIMIZATION
	// ??? can be improved using interpolation
	if( last_insert == p ) 	// do not allow duplicate records
	    return;
	last_insert = p;
#endif
        buf.push_back(p);	// add to end
    }
    virtual double get(double time) // get delayed value (with interpolation)
    { 
        // this code is EXPERIMENTAL !!! ### (not effective enough)
//Print("bs=%d\n", buf.size());
	Pair p(-1,0);
	Pair l(-1,0);
	std::deque<Pair>::iterator i;
	int n; // number of checked records
	// ASSERT: there should be at least one record in the buffer
	for( n=0, i=buf.begin(); i!=buf.end(); ++i ) {
	    l = p;
	    p = *i;
	    n++;
	    if( p.time > time ) break;
	}
	// ASSERT: n>0
	if( n < 2 )	// we want time before first recorded sample 
	    return p.value; 	// use first buffer value as default
	else {		// at least two records read
	    if( p.time < time ) { 	// too small delay ###
		SIMLIB_error(DelayTimeErr);// ###!!! do it better
	    }
	    // standard situation 
	    const int LIMIT=2; // >=2  --- allows change of delay if bigger 
	                       // WARNING: slow for bigger LIMIT
	    // remove old items in buffer
	    for(; n>LIMIT; n--)
	        buf.pop_front(); // remove front records
	    // linear interpolation
	    double dt = p.time - l.time;
	    double dy = p.value - l.value;
	    // ASSERT: dt > 0
	    return l.value + dy*(time-l.time)/dt;
	}
    } // get
}; // class SIMLIB_DelayBuffer


/////////////////////////////////////////////////////////////////////////////
// Delay constructor --- initialize and register delay block
//
Delay::Delay(Input i, double _dt, double ival) : 
    aContiBlock1( i ),  		// input expression
    last_time( Time ),
    last_value( ival ),
    buffer( new SIMLIB_DelayBuffer ),	// allocate delay buffer
    dt( _dt ),          		// delay time
    initval( ival )	    		// initial value of delay
{   // constructor body
    Dprintf(("Delay::Delay(in=%p, dt=%g, ival=%g)", &i, _dt, ival));
    SIMLIB_Delay::Register( this );	// register delay in list of delays
    Init(); // initialize -- important for dynamically created delays
}

/////////////////////////////////////////////////////////////////////////////
// Delay destructor --- remove buffer and delay from list
//
Delay::~Delay() 
{
    Dprintf(("Delay::~Delay()"));
    delete buffer;    			// free the delay buffer
    SIMLIB_Delay::UnRegister(this);	// remove from delay list
}

/////////////////////////////////////////////////////////////////////////////
// Delay::Init --- initialize delay status
//  called automatically by Init()
// initialize: buffer and last output value
//WARNING: does not set/change dt !!!### (if multiple experiments)
void Delay::Init() {
    buffer->clear();			// empty buffer
    buffer->put( last_value=initval, last_time=Time );	// set initial value
}    


/////////////////////////////////////////////////////////////////////////////
// Delay::Sample --- sample input value 
//
void Delay::Sample() 
{   
    Dprintf(("Delay::Sample()"));
    buffer->put( InputValue(), Time );	// store into memory
}

/////////////////////////////////////////////////////////////////////////////
// Delay::Value --- get delay output value
//
double Delay::Value() 
{
    Dprintf(("Delay::Value()"));
    double oldtime = Time - dt;		// past time 
    if( last_time != oldtime ) {        // is not already computed?
        last_value = buffer->get( oldtime );
        last_time = oldtime;
    }
    return last_value;
}

/////////////////////////////////////////////////////////////////////////////
// Delay::Set --- change delay time
//
// EXPERIMENTAL --- this should be used with care
//
double Delay::Set(double newdelay) 
{
   double last = dt;
   if( newdelay>=0 && newdelay<=Time )  // too weak condition ###
      dt = newdelay;
   return last;
}

}

// end of module

