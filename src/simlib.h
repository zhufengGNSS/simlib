/////////////////////////////////////////////////////////////////////////////
//! \file simlib.h    SIMLIB/C++ main interface
///
/// \defgroup simlib SIMLIB/C++
///
///

#ifndef __SIMLIB__

#define __SIMLIB__        0x0302 // version of SIMLIB (numerical)
#define SIMLIB_VERSION    "3.02" // version of SIMLIB (string)

#define SIMLIB_COPYRIGHT  "(c) Petr Peringer, 1991-2011"
// -------------------------------------------------------------------------
// SIMLIB/C++ (SIMulation LIBrary for C++)
// -------------------------------------------------------------------------
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
// USA.
// -------------------------------------------------------------------------
//
// This is main header file for SIMLIB/C++
//
// Supported platforms:
//    Linux: GNU C++ compiler, architectures: i386+, x86_64
// Portable to:
//    FreeBSD: GNU C++, i386+
//    MSDOS: DJGPP v2.0 = GNU C++ compiler for MSDOS (DPMI)
//    WIN95: Borland C++ (v5.0, console application)
//    Windows XP: MinGW
//
// Send bug reports to: peringer AT fit.vutbr.cz
//

#ifndef __cplusplus
# error  "Use C++ compiler, please."
#endif

////////////////////////////////////////////////////////////////////////////
// includes ...
#include <cstdlib>      // size_t
#include <list>
#define dlist std::list // TODO: remove

////////////////////////////////////////////////////////////////////////////
/// \namespace simlib3 is the main SIMLIB namespace
namespace simlib3 {
////////////////////////////////////////////////////////////////////////////

extern const unsigned SIMLIB_version;
const char *SIMLIB_version_string();

////////////////////////////////////////////////////////////////////////////
// system and compiler dependent part
#if defined(__MSDOS__)

# define SIMLIB_SYSTEM "MSDOS"
# if defined(__BCPLUSPLUS__)
#   error  Borland C++ for DOS no longer supported (use DJGPP)
# elif defined(__GNUG__)
#   define SIMLIB_COMPILER "GNU C++"
# else
#   error  This compiler is not supported (under MSDOS)
# endif

#elif defined(__linux__) && defined(__GNUC__)

# define SIMLIB_SYSTEM    "Linux"
# define SIMLIB_COMPILER  "GNU C++"

#elif defined(__WIN32__)

# define SIMLIB_SYSTEM    "WIN32"
#  if defined(__BCPLUSPLUS__)
#   define SIMLIB_COMPILER  "Borland C++"
#  elif defined(__GNUC__)
#   define SIMLIB_COMPILER  "GNU C++"
#  endif

#elif defined(__FreeBSD__) && defined(__GNUC__)

# define SIMLIB_SYSTEM    "FreeBSD"
# define SIMLIB_COMPILER  "GNU C++"

#elif defined(__sun__) && defined(__GNUC__)

# define SIMLIB_SYSTEM    "Solaris"
# define SIMLIB_COMPILER  "GNU C++"

#else
# error  "SIMLIB is not implemented for this system/compiler"
#endif

////////////////////////////////////////////////////////////////////////////
// DEBUGGING: print debug info ON/OFF/mode
//
void DebugON();   //!< start debugging output
void DebugOFF();  //!< stop debugging output
unsigned long Debug(unsigned long mode=~0UL); // start/stop partial debugging
// experimental code for start/stop debugging at global level
#ifdef SIMLIB_DEBUG
static unsigned long SIMLIB_old_debug_flags = Debug(SIMLIB_DEBUG);
#endif

////////////////////////////////////////////////////////////////////////////
// ATEXIT: special internal class for atexit, etc...
// TODO: move to internal.h or should it be in ALL modules ?
/// \class SIMLIB_module internal registration of modules
class SIMLIB_module { // internal class for SIMLIB cleanup (atexit)
  const char *string; // module identification string
  SIMLIB_module(const SIMLIB_module&); // disable copy-ctr
  SIMLIB_module &operator= (const SIMLIB_module&); // disable =
 public:
  SIMLIB_module();
  int Init(const char *s);
  ~SIMLIB_module(); // last can do ATEXIT ?
};

/// create single global registration object in each SIMLIB module
static SIMLIB_module SIMLIB_module_id;

////////////////////////////////////////////////////////////////////////////
// overwiew of basic SIMLIB classes (abstractions)
//

class SimObject;                // base abstract class
// discrete:
class   Link;                   // list item base class (Simula-like)
class     Entity;               // discrete model entity
class       Process;            // process base
class       Event;              // event base
class         Sampler;          // periodic calls of global function
class   List;                   // list of objects of class Link descendants
class     Queue;                // priority queue
class   Stat;                   // statistics
class   TStat;                  // time dependent statistics
class   Histogram;              // histogram
class   Facility;               // SOL-like facility
class   Store;                  // SOL-like store
class   Barrier;                // barrier
class   Semaphore;              // semaphore
// continuous:
class   aBlock;                 // abstract block
class     aContiBlock;          // blocks with continuous output
class       Integrator;         // integrator
class       Status;             // status variables
class         Hyst;             // hysteresis
class         Blash;            // backlash
class         Relay;            // general relay
class       Function;           // general function
class       Constant;           // constant
class       Variable;           // variable
class       Parameter;          // parameter (simulation run constant)
//class aBoolBlock;             // Boolean blocks TODO

// combined:
class    aCondition;            // state condition base class
class      Condition;           // state event detector (Boolean version)
class        ConditionUp;       // action by FALSE-->TRUE change
class        ConditionDown;     // action by TRUE-->FALSE

////////////////////////////////////////////////////////////////////////////
// CATEGORY: global constants

////////////////////////////////////////////////////////////////////////////
// simulation time limits
const double SIMLIB_MINTIME = 0.0;    //
const double SIMLIB_MAXTIME = 1.0e30; // 1e30 works for float, too

////////////////////////////////////////////////////////////////////////////
// CATEGORY: global variables ...
// Warning: NOT THREAD-SAFE! --- TODO: change to methods of SimulationContext

extern Entity *const &Current;    //!< pointer to active (now running) entity

// read-only step limits of numerical integration method
extern const double &MinStep;     // minimal step size
extern const double &StepSize;    // current step size
extern const double &OptStep;     // optimal step size
extern const double &MaxStep;     // maximal step size

// error params for numerical integration methods
extern const double &AbsoluteError; // max absolute error
extern const double &RelativeError; // max relative error

//TODO: do we need it public?
/// \enum SIMLIB_Phase_t  values indicate current phase of experiment
enum SIMLIB_Phase_t {
    START,          // before first Init() call
    INITIALIZATION, // after Init() call
    SIMULATION,     // inside Run() call
    TERMINATION,    // after Run() call
    ERROREXIT       // fatal error handling phase
};
/// variable contains the current phase of experiment
extern const SIMLIB_Phase_t &Phase; // used for user checking  TODO: move to internals?


////////////////////////////////////////////////////////////////////////////
// CATEGORY: global functions ...

/// initialize simulator, prepare for next simulation experiment
inline void Init(double t0, double t1=SIMLIB_MAXTIME)   // initialization
{
  extern void SIMLIB_Init(double t0, double t1, unsigned version);
  SIMLIB_Init(t0,t1,__SIMLIB__); // version consistency check
}


/// set calendar implementation
/// @param name string identification of calendar: "list", "cq"
void SetCalendar(const char *name); // set calendar implementation

/// set integration step interval
/// @param dtmin min. step size
/// @param dtmax max. step size (can be slightly increased)
void SetStep(double dtmin, double dtmax);
/// set fixed integration step
/// @param dt (almost) fixed step size
inline void SetStep(double dt) { SetStep(dt,dt); }

/// set max. error estimate for numerical integration method
/// @param abserr absolute tolerance
/// @param relerr tolerance relative to value
void SetAccuracy(double abserr, double relerr);
/// set max. relative error estimate for numerical integration method
/// @param relerr tolerance relative to value
void SetAccuracy(double relerr);

/// run simulation experiment
void Run();
/// stops the simulation run
void Stop();
/// end simulation program
void Abort();

/// print message and terminate program
void Error(const char *fmt, ...);
// TODO: convert to iostreams
/// non-fatal error messages (stdout)
int  _Print(const char *fmt, ...);
/// for Output methods, can be redirected
int  Print(const char *fmt, ...);

/// output value
int  Print(const double x);
/// output 2 values
int  Print(const double x, const double y);
/// output 3 values
int  Print(const double x, const double y, const double z);

/// redirects Output(), Print() to file
void SetOutput(const char *name);

// functions for special blocks implementation
/// request for shorter step of num. integ.
void ContractStep();
/// request for step end at given time
void ContractStep(double time);


////////////////////////////////////////////////////////////////////////////
// CATEGORY: pseudorandom number generators

/// initialize random number seed
void   RandomSeed(long seed=1537L);
/// base uniform generator 0-0.999999...
double Random();
// default Random can be replaced by user-defined base generator:
/// set another random generator
void   SetBaseRandomGenerator(double (*new_gen)());

// following generators depend on Random()
double Beta(double th, double fi, double min, double max);
double Erlang(double alfa, int beta);
/// Exponential distribution @param mv mean value
double Exponential(double mv);
double Gama(double alfa, double beta);
int    Geom(double q);
int    HyperGeom(double p, int n, int m);
double Logar(double mi, double delta);
int    NegBinM(double p,int m);
int    NegBin(double q, int k);
/// Gauss distribution @param mi mean value @param sigma std. deviation
double Normal(double mi, double sigma);
int    Poisson(double lambda);
double Rayle(double delta);
double Triag(double mod, double min, double max);
/// Uniform distribution @param l low @param h high limit
double Uniform(double l, double h);
double Weibul(double lambda, double alfa);


////////////////////////////////////////////////////////////////////////////
// CATEGORY: basics

////////////////////////////////////////////////////////////////////////////
/// \class SimObject
/// base of class hierarchy of SIMLIB
/// <br> objects can have name
/// <br> objects know they are created dynamically (using new)
class SimObject {
 public:
  enum _Flags { _ALLOCATED_FLAG = 1, _EVAL_FLAG = 2 }; //!< internal flags TODO:bitfields?, move
 protected:
  const char * _name;
  unsigned     _flags;
 public:
  bool TestAndSetFlag(bool new_value, unsigned n);
  SimObject();
  virtual ~SimObject();
  void *operator new(size_t size);     //!< allocate object, set _flags
  void operator delete(void *ptr);     //!< deallocate object
  bool isAllocated() const { return (_flags >> _ALLOCATED_FLAG)&1; }

  virtual const char *Name() const;    //!< get object name
  bool HasName() const { return _name !=0; }
  void SetName(const char *name);      //!< assign the name

  virtual void Output() const;         //!< print object to default output
 private:
  SimObject(SimObject&);               //!< disabled operation
  void operator= (SimObject&);         //!< disabled operation
//TODO:  void *operator new[](size_t size);   // allocate array, set _flags
};

////////////////////////////////////////////////////////////////////////////
//! base class for all double-linked list items
/// <br> item can be at single place only (identified by where() method)
class Link : public SimObject {
  Link(Link&);                         // disable operation
  void operator= (Link&);              // disable operation
  Link *pred;                          //!< previous object in List
  Link *succ;                          //!< next object in List
  List *head;                          //!< pointer to List (if any)
  friend class List;                   // only list can access pred,succ,head
 protected:
  Link(Link *p, Link *s, List *h);
 public:
  Link();
  virtual ~Link();
  virtual void Into(List *l);          //!< insert last
  virtual void Out();                  //!< remove from list
  List *where() { return head; }       //!< where is linked TODO: rename
  virtual bool isInQueue() { return where()!=NULL; } //!< present in queue
};

////////////////////////////////////////////////////////////////////////////
// CATEGORY: discrete blocks

//! entity/process priority values
enum {
  LOWEST_PRIORITY  = 0,
  HIGHEST_PRIORITY = 255,
  DEFAULT_PRIORITY = LOWEST_PRIORITY
};
typedef unsigned char EntityPriority_t;  //!< process priority
typedef unsigned char ServicePriority_t; //!< service priority (see Facility)

/// class EventNotice is  private to calendar implementation
class EventNotice;           // here we use only pointer to this class

////////////////////////////////////////////////////////////////////////////
//! abstract base class for active entities (Process, Event)
///
/// instances of derived classes provide Behavior(),
/// execution can be scheduled in calendar
class Entity : public Link {
    Entity(const Entity&);           // disable
    Entity&operator=(const Entity&); // disable
  protected:
    static unsigned long _Number;   // current number of entities
    unsigned long _Ident;           // unique identification of entity
    ////////////////////////////////////////////////////////////////////////////
    // TODO: next attributes will be changed/removed:
    // all should be in special records stored into queues
    // Queue stores insertion time for statistics:
    friend class Queue;             // ### remove
    double _MarkTime;               // beginning of waiting in queue ###!!!
    // Facility and Store use these data
    friend class Facility;
    friend class Store;
    // TODO: this should be stored in queues at Facility/Store
    union {
        double _RemainingTime; // rest of time of interrupted service (Facility) ###
        unsigned long _RequiredCapacity; // required store capacity of Store
    };
    ServicePriority_t _SPrio;           // priority of service (in Facility) ###
    ////////////////////////////////////////////////////////////////////////////
  public:
    typedef EntityPriority_t Priority_t;
    /// priority of the entity (scheduling,queues)
    Priority_t Priority;
    Entity(Priority_t p = DEFAULT_PRIORITY);
    virtual ~Entity();

    // common interface for Events and Processes
    operator Entity* () { return this; } // default conversion
    virtual const char *Name() const;
    double ActivationTime();            // get activation time if scheduled

    virtual void Activate();            // activate now
    virtual void Activate(double t);    // activate in time t (schedule)
    virtual void Passivate();           // deactivation
    virtual void Terminate() = 0;       // end of behavior and remove entity
    bool Idle() { return _evn==0; }     // entity activation is not scheduled in calendar
    void Cancel() { Terminate(); }      // end
//    virtual void Into(Queue *q);         // insert itself into queue
    virtual void Out();                 // remove from queue

  private:
    // Simulation control algorithm interface:
    /// run the Behavior() function
    virtual void _Run() throw() = 0;
    // friends:
    friend void SIMLIB_DoActions();     // internal function - event dispatcher

    // Calendar interface:
    friend class EventNotice;           // internal calendar class sets _Ev
    EventNotice *_evn;                  //!< points to calendar item, if scheduled
  public:
    EventNotice *GetEventNotice() { return _evn; }
};

// global wrapper functions:
inline void Activate(Entity *e)  { e->Activate(); }   // activate entity e
inline void Passivate(Entity *e) { e->Passivate(); }  // passivate entity e

////////////////////////////////////////////////////////////////////////////
/**
  * abstract base class for all simulation processes
  * @ingroup process
  */
class Process : public Entity {
    Process(const Process&);            // disable copying
    Process&operator=(const Process&);  // disable copying
  /// process context pointer
  void * _context;
  virtual void _Run() throw(); // point of activation

  /// possible process status values
  enum ProcessStatus_t {
      _PREPARED=1, _RUNNING, _INTERRUPTED, _TERMINATED
  } _status;
  bool isPrepared()    { return (_status==_PREPARED);    } // Before start
  bool isCurrent()     { return (_status==_RUNNING);     } // Behavior runs
  bool isInterrupted() { return (_status==_INTERRUPTED); } // Can continue
  bool isTerminated()  { return (_status==_TERMINATED);  } // Zombie

  friend class WaitUntilList;
  bool _wait_until;                     // waiting for condition
  void _WaitUntilRemove();

 public:
  Process(Priority_t p=DEFAULT_PRIORITY);
  virtual ~Process();

  /// behavior description
  virtual void Behavior() = 0;

  virtual void Output();                // print object
  virtual const char *Name() const;     // name of object
  virtual void Activate();              // activate now
  virtual void Activate(double t);      // activate in time t (schedule)
  virtual void Passivate();             // sleeping process (deactivation)
  virtual void Wait(double dtime);      // wait dtime interval
  bool  _WaitUntil(bool test);          // wait for condition (slow!)
/// wait until condition is true
#define WaitUntil(expr)  while(_WaitUntil(expr));
  void Interrupt(); // test of WaitUntil list, allow running others
  virtual void Terminate();             // kill process

  // TODO: add global functions using Current?
  void Seize(Facility &f, ServicePriority_t sp=0);  // seize facility
  void Release(Facility &f);                        // release facility
  void Enter(Store &s, unsigned long ReqCap=1); // acquire some capacity
  void Leave(Store &s, unsigned long ReqCap=1); // return some capacity
  virtual void Into(Queue &q);          // insert process into queue
};

////////////////////////////////////////////////////////////////////////////
//! abstract base class for events
//
class Event : public Entity {
  virtual void _Run() throw();
  virtual void Terminate();       // do not use (deprecated)
 public:
  Event(Priority_t p=DEFAULT_PRIORITY);
  virtual ~Event();

  /// behavior description
  virtual void Behavior() = 0;

  virtual void Output();          // print object
  virtual const char *Name() const;
  // inherited: Activate, Passivate, Cancel
};

////////////////////////////////////////////////////////////////////////////
//! objects of this class call global function periodically
//  (typicaly used for output of continuous model)
//
class Sampler: public Event {
    Sampler(const Sampler&);            //## disable
    Sampler&operator=(const Sampler&);  //## disable
    static Sampler *First;              // list of objects TODO: use container
    Sampler *Next;                      // next object
  protected:
    /// function to call periodically
    void (*function)();
    /// last sample time -- prevents Stop() from sample duplication
    double last;
    /// step of sampling
    double step;
    /// switch on/off
    bool on;
    /// behavior description
    virtual void Behavior();
  public:
    Sampler(void (*pf)(), double dt=0.0);
    virtual ~Sampler();
    virtual void Output();          // print object
    void Start();                   // start + sample
    void Stop();                    // sample + stop
    void Sample();                  // performs sample (function call)
    double SetStep(double dt=0.0);  // change step
    double GetStep() const { return step; } // get current step
  private:
    static void InitAll();          // initialize all samplers (Init)
    static void ActivateAll();      // start all samplers (Run)
};


////////////////////////////////////////////////////////////////////////////
// CATEGORY: discrete blocks - pasive

#ifdef new_version
////////////////////////////////////////////////////////////////////////////
//! abstract base class of all statistics
// TODO:
class aStat : public SimObject {
 protected:
  double min;                   // minimal recorded value
  double max;                   // maximal recorded value
  unsigned long n;              // number of values recorded
 public:
    class aStatException {};
  aStat();
  aStat(const char *name);
  ~aStat();
  virtual void Clear() = 0;                 // initialize

  virtual void operator () (double x) = 0;  // record value
  virtual void Record(double x) = 0;        // record value

  virtual double MeanValue() = 0;           // compute mean value
  double Min() const { /* test n==0 +exception */ return min; }
  double Max() const { /* test n==0 */ return max; }
  unsigned long NumberOfRecords() const { return n; }
   /// print statistics
  virtual void Output();
};
#endif

////////////////////////////////////////////////////////////////////////////
//! class for statistical information gathering
//
class Stat : public SimObject {
 protected:
  double sx;                    // sum of values
  double sx2;                   // sum of value square
  double min;                   // min value
  double max;                   // max value
  unsigned long n;              // number of values recorded
 public:
  Stat();
  Stat(const char *name);
  ~Stat();
  virtual void Clear();         // initialize
  void operator () (double x);  // record the value
// Stat &operator = (Stat &x);  // TODO: copy semantics
// Stat &operator += (Stat &x); // sum two statistics + template operator +
  virtual void Output();        // print statistic
  unsigned long Number() const { return n; }
  double Min() const           { /* TODO: test n==0 */ return min; }
  double Max() const           { /* test n==0 */ return max; }
  double Sum() const           { return sx; }
  double SumSquare() const     { return sx2; }
  double MeanValue() const;
  double StdDev() const;
};


////////////////////////////////////////////////////////////////////////////
//! time dependent statistic
//
class TStat : public SimObject {
 protected:
  double sxt;                   // sum of x*time
  double sx2t;                  // sum of squares
  double min;                   // min value x
  double max;                   // max value x
  double t0;                    // time of initialization
  double tl;                    // last record time
  double xl;                    // last recorded value x
  unsigned long n;              // number of records
  friend class Facility; // needs to correct n -- TODO: remove
  friend class Store;
  friend class Queue;
 public:
  TStat(double initval=0.0);
  TStat(const char *name, double initval=0.0);
  ~TStat();
  virtual void Clear(double initval=0.0);        // initialize
  virtual void Output();                         // print
  virtual void operator () (double x);           // record value
  unsigned long Number() const { return n; }
  double Min() const           { /*TODO: only if(n>0)*/ return min; }
  double Max() const           { return max; }
  double Sum() const           { return sxt; }
  double SumSquare() const     { return sx2t; }
  double StartTime() const     { return t0; }
  double LastTime() const      { return tl; }
  double LastValue() const     { return xl; }
  double MeanValue() const;
};


////////////////////////////////////////////////////////////////////////////
//! list of Link* items, uses data from Link
//
//prehistoric implementation, TODO: use list<Link*>
//
class List : public Link { // circular list of Link items
    List(List&);                         // disable
    void operator= (List&);              // disable
    static Link *next(Link*p) { return p->succ; }
    static Link *previous(Link*p) { return p->pred; }
    unsigned n;                          // number of objects in list
  protected:
    class iterator {
	    Link *p; // position in List
    public:
	    iterator(const iterator&x): p(x.p) {}
	    iterator &operator=(const iterator&x) { p = x.p; return *this; }
	    iterator(Link *pos) : p(pos) {}
	    iterator &operator++() { p = List::next(p); return *this; }
	    iterator &operator--() { p = List::previous(p); return *this; }
	    iterator operator++(int) {
	        Link *tmp = p; p = List::next(p); return tmp;
	    }
	    iterator operator--(int) {
	        Link *tmp = p; p = List::previous(p); return tmp;
	    }
	    Link * operator*() const { return p; }
	    bool operator != (iterator q) const { return p!=q.p; }
	    bool operator == (iterator q) const { return p==q.p; }
    }; // iterator
    friend class iterator;
 public:
    List();
    List(const char *_name);
    ~List();
    virtual void Output();               // print
    // should be renamed:
    void InsFirst(Link *e);              // insert operations
    void InsLast (Link *e);
    void PredIns(Link *e, iterator pos); //
    void PostIns(Link *e, iterator pos); //
    Link *GetFirst();                    // removing operations
    Link *GetLast();                     //
    virtual Link *Get(iterator pos);     // remove at position
    //  unsigned Length() { return size(); }  // COMPATIBILITY
    //  virtual void Clear() { clear(); }    // remove all items
    // new-style:
    unsigned size() const  { return n; }
    iterator begin()  { return Link::succ; }
    iterator end()    { return this; }
    Link *front()     { return empty() ? 0 : Link::succ; }
    Link *back()      { return empty() ? 0 : Link::pred; }
    bool empty()      { return Link::succ == this; }
    void clear();
}; // List


////////////////////////////////////////////////////////////////////////////
//! priority queue
//
class Queue : public List { // don't inherit interface for now
//TODO:remove
    friend class Facility;
    friend class Store;
  public:
    typedef List::iterator iterator;
    TStat StatN;
    Stat  StatDT;                        // statistics
    Queue();
    Queue(const char *_name);
    ~Queue();
    virtual const char *Name() const;
    virtual void Output();               // print
    operator Queue* () { return this; }  // allows Queue instead Queue*
    iterator begin()   { return List::begin(); }
    iterator end()     { return List::end(); }
    Entity *front()    { return (Entity*)List::front(); }
    Entity *back()     { return (Entity*)List::back(); }
    void clear();
    // size(), empty() inherited
    // backward COMPATIBILITY
    void Clear()  { clear(); }
    bool Empty()  { return empty(); }
    unsigned Length() { return size(); }
    // to rename:
    virtual void Insert  (Entity *e);            // priority insert
    void InsFirst(Entity *e);
    void InsLast (Entity *e);
    void PredIns (Entity *e, iterator pos); // insert at position
    void PostIns (Entity *e, iterator pos);
    virtual Entity *Get(iterator pos);           // remove entity
    Entity *GetFirst();
    Entity *GetLast();
};

////////////////////////////////////////////////////////////////////////////
//! histogram and statistics
//
class Histogram : public SimObject {
    Histogram(const Histogram&);           // disable
    Histogram&operator=(const Histogram&); // disable
 protected:
  unsigned *dptr;            // value array
  double   low;              // low bound
  double   step;             // interval width
  unsigned count;            // number of intervals
 public:
  Stat     stat;             // statistics
  Histogram();
  Histogram(double low, double step, unsigned count=10);
  Histogram(const char *_name, double low, double step, unsigned count=10);
  ~Histogram();
  virtual void Output();               // print
  void Init(double low, double step, unsigned count);
  void operator () (double x);         // record value x
  virtual void Clear();                // initialize (zero) value array
  double Low() const     { return low; }
  double High() const    { return low + step*count; }
  double Step() const    { return step; }
  unsigned Count() const { return count; }
  unsigned operator [](unsigned i) const;  // # of items in interval[i]
};



////////////////////////////////////////////////////////////////////////////
//! (SOL-like) facility
//
class Facility : public SimObject {
    Facility(const Facility&);                  // disable
    Facility&operator=(const Facility&);        // disable
  unsigned char _Qflag;
 public:
  Entity *in;                // entity in service
  Queue  *Q1;                // input queue
  Queue  *Q2;                // interrupted requests queue
  TStat tstat;               // stat
  Facility();
  Facility(const char *_name);
  Facility(Queue *_queue1);
  Facility(const char *_name, Queue *_queue1);
  virtual ~Facility();
  virtual void Output();                         // print output
  operator Facility* () { return this; }
  void SetQueue(Queue *queue1);                  // change input queue
  bool OwnQueue() const;                                // test
  bool Busy() const { return in!=0; }
  unsigned QueueLen() const { return Q1->size(); }
  virtual void Seize(Entity *e, ServicePriority_t sp=DEFAULT_PRIORITY);
  virtual void Release(Entity *e);
  virtual void QueueIn(Entity *e, ServicePriority_t sp); // go into queue Q1
  virtual void Clear();                          // init
 protected:
  virtual void QueueIn2(Entity *e);              // do into Q2
};

////////////////////////////////////////////////////////////////////////////
//! (SOL-like) store
//
class Store : public SimObject {
    Store(const Store&);              // disable
    Store&operator=(const Store&);    // disable
  unsigned char _Qflag;
  unsigned long capacity;        // capacity of store
  unsigned long used;            // used capacity
 public:
  Queue *Q;
  TStat tstat;               // statistics
  Store();
  Store(unsigned long _capacity);
  Store(const char *_name, unsigned long _capacity);
  Store(unsigned long _capacity, Queue *queue);
  Store(const char *_name, unsigned long _capacity, Queue *queue);
  virtual ~Store();
  virtual void Output();                         // print
  operator Store* () { return this; }
  void SetCapacity(unsigned long _capacity);
  void SetQueue(Queue *queue);
  unsigned long Free() const { return capacity - used; }   // free capacity
  unsigned long Used() const { return used; }              // used capacity
  unsigned long Capacity() const { return capacity; }      // max capacity
  bool Full() const   { return Free() == 0; }           // store is full
  bool Empty() const  { return Used() == 0; }           // store is empty
  bool OwnQueue();
  unsigned QueueLen() const { return Q->size(); }
  virtual void Enter(Entity *e, unsigned long rcap); // allocate
  virtual void Leave(unsigned long rcap);            // deallocate
  virtual void QueueIn(Entity *e, unsigned long c);  // go into queue
  virtual void Clear();                          // init
};


////////////////////////////////////////////////////////////////////////////
// CATEGORY: continuous blocks
//

////////////////////////////////////////////////////////////////////////////
//! abstract base class for all blocks
//
class aBlock : public SimObject {                // base class
};

////////////////////////////////////////////////////////////////////////////
//! abstract base for continuous blocks with single output
//
class aContiBlock : public aBlock {
    aContiBlock(const aContiBlock&); // disable copy ctr
    void operator= (const aContiBlock&); // disable assignment
    /// evaluate without loop detection
    virtual void Eval() {};
  protected:
    bool isEvaluated;                   //TODO:remove flag for algebraic loop detection
  public:
    aContiBlock(): isEvaluated(false) {}
    /// evaluate with loop detection
    virtual void _Eval();
    /// get block output value <br>
    /// this method should be defined in classes derived from aContiBlock
    virtual double Value() = 0;
};

////////////////////////////////////////////////////////////////////////////
//! constant block - value can't be changed
//
class Constant : public aContiBlock {
  const double value;
 public:
  Constant(double x) : value(x) {}
  virtual double Value ()       { return value; }
};

////////////////////////////////////////////////////////////////////////////
//! block: variable (value can be changed)
//
class Variable : public aContiBlock {
  double value;
 public:
  Variable(double x=0) : value(x) {}
  Variable &operator= (double x)  { value = x; return *this; }
  virtual double Value ()         { return value; }
};

////////////////////////////////////////////////////////////////////////////
//! variable parameter (can't be changed at simulation run)
//
class Parameter : public aContiBlock {
  double value;
 public:
  Parameter(double x) : value(x) {}
  Parameter &operator= (double x) { value = x; return *this; }
  virtual double Value ()         { return value; }
};


// time values:
extern const double & StartTime;       // time of simulation start
extern const double & NextTime;        // next-event time
extern const double & EndTime;         // time of simulation end
//!WARNING: Time cannot be used in block expressions!
extern const double & Time;            // model time (is NOT the block)
extern aContiBlock  & T;               // model time (continuous block)


//TODO: add map<aContiBlockPtr,int> for  reference counting
// speed is not important Inputs are created only in the model construction phase
// (and in dynamical models at simulation time, too).
#define RegisterReference(aContiBlockPtr)
#define UnRegisterReference(aContiBlockPtr)

////////////////////////////////////////////////////////////////////////////
//! continuous block connection (transparent reference)
/// wrapper for pointer to objects of aContiBlock derived classes <br>
/// (small object, without virtual methods)
/// <br>
/// It is used for referencing of continuous blocks in block-expressions.
/// All references are transparent, so there is problem with initialization
/// order. Using uninitialized Input is an error. If you
/// need block independent on initialization order, use Expression.

class Input {
  aContiBlock *bp;
  Input(); // disable default constructor
 public:
  /// transparent copy of block reference
  Input(const Input &i): bp(i.bp) { RegisterReference(bp); }

  Input(aContiBlock &cb): bp(&cb) { RegisterReference(bp); } //!< reference to block
  Input(aContiBlock *cb): bp(cb)  { RegisterReference(bp); } //!< pointer to block

  /// create block of type Constant
  Input(const double c):  bp(new Constant(c)) { RegisterReference(bp); }
  // WARNING: using C++ variable in block expression leads to constant

  ~Input() { UnRegisterReference(bp); }

  /// Redirect reference
  Input Set(Input i)     {
      Input p = bp; // old value
      UnRegisterReference(bp);
      bp=i.bp;
      RegisterReference(bp);
      // TODO: LoopCheck();
      return p; // returns old value
  }

  /// Redirect reference
  Input &operator= (const Input&x) {
      UnRegisterReference(bp);
      bp = x.bp;
      RegisterReference(bp);
      // TODO: LoopCheck();
      return *this;
  }

  double Value() const { return bp->Value(); } // transparent

  bool operator ==(aContiBlock *p) const { return bp==p; } // for tests only
};


////////////////////////////////////////////////////////////////////////////
//! base for continuous blocks with single input and algebraic loop check
//
class aContiBlock1 : public aContiBlock {
  Input input;
 public:
  aContiBlock1(Input i);
  double InputValue() { return input.Value(); }
};

////////////////////////////////////////////////////////////////////////////
//! reference to block expression
//
struct Expression : public aContiBlock1 {
  Expression(Input i) : aContiBlock1(i) {}
  double Value();
};

////////////////////////////////////////////////////////////////////////////
//! base for continuous blocks with two inputs
//
class aContiBlock2 : public aContiBlock {
  Input input1;
  Input input2;
 public:
  aContiBlock2(Input i1, Input i2);
  double Input1Value() { return input1.Value(); }
  double Input2Value() { return input2.Value(); }
};

////////////////////////////////////////////////////////////////////////////
//! base for continuous blocks vith three inputs and algebraic loop check
//
class aContiBlock3 : public aContiBlock {
  Input input1;
  Input input2;
  Input input3;
 public:
  aContiBlock3(Input i1, Input i2, Input i3);
  double Input1Value() { return input1.Value(); }
  double Input2Value() { return input2.Value(); }
  double Input3Value() { return input3.Value(); }
};


////////////////////////////////////////////////////////////////////////////
//! internal
//  IntegratorContainer - container of integrators,
//                        it is a singleton
//TODO: move to implementation header
class IntegratorContainer {
private:
  static dlist<Integrator*> * ListPtr;  // list of integrators
  IntegratorContainer();  // forbid constructor
  static dlist<Integrator*> * Instance(void);  // return list (& create)
public:
  typedef dlist<Integrator*>::iterator iterator;
  // is there any integrator in the list? (e.g. list is not empty)
  static bool isAny(void) {
    return ListPtr!=0 && !(ListPtr->empty());
  }
  // # of elements in the list
  static size_t Size(void) {
    return (ListPtr!=0) ? (ListPtr->size()) : 0;
  }
  // return iterator to the first element
  static iterator Begin(void) {
    return Instance()->begin();
  }
  // return iterator to the end (not to the last element!)
  static iterator End(void) {
    return Instance()->end();
  }
  static iterator Insert(Integrator* ptr);  // insert element into container
  static void Erase(iterator it);  // exclude element
  static void InitAll();           // initialize all
  static void EvaluateAll();       // evaluate all integrators
  static void LtoN();              // last -> now
  static void NtoL();              // now -> last
}; // class IntegratorContainer


////////////////////////////////////////////////////////////////////////////
//! internal
//  StatusContainer -- container of status variables,
//                     it is a singleton
//TODO: move to implementation header
class StatusContainer {
private:
  static dlist<Status*>* ListPtr;  // list of integrators
  StatusContainer();  // forbid constructor
  static dlist<Status*>* Instance(void);  // return list (& create)
public:
  typedef dlist<Status*>::iterator iterator;
  // is there any integrator in the list? (e.g. list is not empty)
  static bool isAny(void) {
    return ListPtr!=0 && !(ListPtr->empty());
  }
  // # of elements in the list
  static size_t Size(void) {
    return (ListPtr!=0) ? (ListPtr->size()) : 0;
  }
  // return iterator to the first element
  static iterator Begin(void) {
    return Instance()->begin();
  }
  // return iterator to the end (not to the last element!)
  static iterator End(void) {
    return Instance()->end();
  }
  static iterator Insert(Status* ptr);  // insert element into container
  static void Erase(iterator it);  // exclude element
  static void InitAll();           // initialize all
  static void ClearAllValueOK();   // invalidate values of all items
  static void EvaluateAll();       // evaluate all integrators
  static void LtoN();              // last -> now
  static void NtoL();              // now -> last
}; // class StatusContainer


////////////////////////////////////////////////////////////////////////////
//! internal
//  IntegrationMethod
//  Abstract base class for integration methods
//
//TODO: reduce to simple public interface,
//      delegate implementation to internal (hidden) class
class IntegrationMethod {
    IntegrationMethod(const IntegrationMethod&); // ## disable
    IntegrationMethod&operator=(const IntegrationMethod&); // ## disable
private:
  static IntegrationMethod* CurrentMethodPtr;  // method used at present
  static dlist<IntegrationMethod*>* MthLstPtr; // list of registrated methods
  dlist<IntegrationMethod*>::iterator ItList;  // position in the list
  const char* method_name;  // C-string --- the name of the method
protected:  //## repair
  class Memory;  // forward declaration
  friend class Memory;
private:   //## repair
  size_t PrevINum;  // # of integrators in previous step
  dlist<Memory*> MList;  // list of auxiliary memories
  static dlist<Memory*> * PtrMList;  // pointer to list being filled
  IntegrationMethod();  // forbid implicit constructor
  IntegrationMethod(IntegrationMethod&);  // forbid implicit copy-constructor
  static bool Prepare(void);  // prepare system for integration step
  static void Iterate(void);  // compute new values of state blocks
  static void Summarize(void);  // set up new state after integration
protected:
  static bool IsEndStepEvent; // flag - will be event at the end of the step?
  typedef IntegratorContainer::iterator Iterator;  // iterator of intg. list
  static Iterator FirstIntegrator(void) {  // it. to first integrator in list
    return IntegratorContainer::Begin();
  }
  static Iterator LastIntegrator(void) {  // it. to last integrator in list
    return IntegratorContainer::End();
  }
  static bool StateCond(void);  // check on changes of state conditions
  static IntegrationMethod* SearchMethod(const char* name);  // find method

  /* memory of auxiliary values */

  class Memory {
      Memory(const Memory&); // ## disable
      Memory&operator=(const Memory&); // ## disable
    private:
      static const size_t page_size;  // size of memory page
      double *arr;  // array
      size_t mem_size;  // allocated memory size
      Memory(Memory&);  // forbid copy constructor
      void operator= (Memory&);  // disable the operation
      dlist<Memory*>::iterator it_list;  // position in list
      dlist<Memory*> * ListPtr;  // which list is memory in
    public:
      Memory(dlist<Memory*> * PtrList=PtrMList);  // create empty memory
      virtual ~Memory();
      double& operator[](size_t ind) {  // access the element of the array
        //      dprintf(("IntegrationMethod::Memory::operator[](%lu) = %g",
        //         (long unsigned)ind, arr[ind]));
        return arr[ind];
      }
      virtual void Resize(size_t cs); // change size, content will be undefined!
  }; // class Memory

public:
  IntegrationMethod(const char* name);  // registrate method and name it
  virtual ~IntegrationMethod();  // destructor unregistrates method
  virtual bool IsSingleStep(void)=0; // is it a single-step method?
  virtual void TurnOff(void);  // turn off integration method
  virtual void Integrate(void) = 0;  // the method does integration
  virtual bool PrepareStep(void);  // prepare object for integration step
  virtual void Resize(size_t size);  // resize all memories to given size
  static void StepSim(void);  // single step of numerical integration method
  static void IntegrationDone(void) {  // terminate integration
    CurrentMethodPtr->TurnOff();  // suspend present method
  }
  static void SetMethod(const char* name);  // set method which will be used
  static const char* GetMethod(void) {  // get name of method which is used
    return CurrentMethodPtr->method_name;
  }
  // auxiliary functions (interface) for user to add own method
  static void InitStep(double step_frag); // initialize step
  static void FunCall(double step_frag); // evaluate y'(t) = f(t, y(t))
  static void SetOptStep(double opt_step) { // set optimal step size
    extern double SIMLIB_OptStep; // available without including internal.h
    SIMLIB_OptStep = opt_step;
  }
  static void SetStepSize(double step_size) { // set step size
    extern double SIMLIB_StepSize; // available without including internal.h
    SIMLIB_StepSize = step_size;
  }
  static bool IsConditionFlag(void) { // wer any changes of condition vector?
    extern bool SIMLIB_ConditionFlag; // available without ... blah, blah
    return SIMLIB_ConditionFlag;
  }
  static int GetErrNo(void) { // return # of errors
    extern int SIMLIB_ERRNO;
    return SIMLIB_ERRNO;
  }
  static void SetErrNo(int num) { // set # of errors
    extern int SIMLIB_ERRNO;
    SIMLIB_ERRNO = num;
  }
}; // class IntegrationMethod


////////////////////////////////////////////////////////////////////////////
//! internal
//  single-step integration method
//
class SingleStepMethod : public IntegrationMethod {
private:
  // flag: single-step method is used to start multi-step method
  // (e. g. to restrict step increasing)
  bool StartMode;
public:
  SingleStepMethod(const char* name) :  // initialize method and registrate it
    IntegrationMethod(name),
    StartMode(false)
  { /* NOTHING */ }
  virtual void Integrate(void) = 0;  // the method does integration
  virtual bool IsSingleStep(void) {  // it's a single-step method
    return true;
  }
  void SetStartMode(bool start_mode) {  // set flag on/off
    StartMode=start_mode;
  }
  bool IsStartMode(void) {  // is method used to start multi-step method?
    return StartMode;
  }
}; // class SingleStepMethod


////////////////////////////////////////////////////////////////////////////
//! internal
//  multi-step integration method
//
class MultiStepMethod : public IntegrationMethod {
    MultiStepMethod(const MultiStepMethod&); // ## disable
    MultiStepMethod&operator=(const MultiStepMethod&); // ## disable
private:
  char* SlaveName;  // the name of the method used for starting
  SingleStepMethod* Slave_Ptr;  // pointer to the method used for starting
protected:
  SingleStepMethod* SlavePtr(void);  // return pointer to the starting method
public:
  // initialize method
  MultiStepMethod(const char* name, const char* slave_name);
  ~MultiStepMethod();  // destructor frees dynamic data
  virtual bool IsSingleStep(void) {  // it's a multi-step method
    return false;
  }
  // set starting method for given method
  static void SetStarter(const char* name, const char* slave_name);
  // set starting method for the method
  virtual void SetStarter(const char* slave_name);
  // get the name of starting method of given method
  static const char* GetStarter(const char* name);
  virtual bool PrepareStep(void);  // prepare the object for the step
  virtual void Integrate(void) = 0;  // the method does integration
  virtual void TurnOff(void);  // turn off method & its slave
}; // class MultiStepMethod


////////////////////////////////////////////////////////////////////////////
//! internal
//  Abstract base class for integration methods
//  with status auxiliary memories
//
class StatusMethod : public SingleStepMethod {
private:
  StatusMethod();  // forbid implicit constructor
  StatusMethod(StatusMethod&);  // forbid implicit copy-constructor
  size_t PrevStatusNum;  // # of status variables in previous step
  dlist<Memory*> StatusMList;  // list of auxiliary memories
  static dlist<Memory*>* PtrStatusMList;  // pointer to list being filled
protected:
  typedef StatusContainer::iterator StatusIterator;  // iterator of intg. list
  static StatusIterator FirstStatus(void) {  // it. to first status in list
    return StatusContainer::Begin();
  }
  static StatusIterator LastStatus(void) {  // it. to last status in list
    return StatusContainer::End();
  }

//## repair
  friend class EULER;
  friend class RKE;
  /* memory of auxiliary status values */
  class StatusMemory : public Memory {
  private:
    StatusMemory(StatusMemory&);  // forbid copy constructor
    void operator= (StatusMemory&);  // disable the operation
  public:
    // create empty memory
    StatusMemory(dlist<Memory*>* PtrList=PtrStatusMList) : Memory(PtrList) { }
    // free dynamic data
    virtual ~StatusMemory() { }
  }; // class StatusMemory

  friend class StatusMemory;

  // store state of integrators and status variables
  static void StoreState(Memory& di, Memory& si, StatusMemory& xi);
  // restore state of integrators and status variables
  static void RestoreState(double dthlf, Memory& di,
                           Memory& si, StatusMemory& xi);
  // move startpoint to given state
  static void GoToState(Memory& di, Memory& si, StatusMemory& xi);

public:
  StatusMethod(const char* name);  // initailization
  virtual ~StatusMethod() { } // free dynamic datas
  virtual void TurnOff(void);  // turn off integration method
  virtual void Integrate(void) = 0;  // the method does integration
  virtual bool PrepareStep(void);  // prepare integration step
  virtual void StatusResize(size_t size);  // resize status memories
}; // StatusMethod


////////////////////////////////////////////////////////////////////////////
//  interface for class IntegrationMethod & its descendants
//

//! select the integration method
//! Methods: "abm4", "euler", "fw", "rke"(default), "rkf3", "rkf5", "rkf8"
inline void SetMethod(const char* name)
{
  IntegrationMethod::SetMethod(name);
}

//! get the name of the method which is used
inline const char* GetMethod(void)
{
  return IntegrationMethod::GetMethod();
}

//! set the method which will be used to start given multistep method
inline void SetStarter(const char* name, const char* slave_name)
{
  MultiStepMethod::SetStarter(name, slave_name);
}

//! set the method which will be used to start currently used multistep method
inline void SetStarter(const char* slave_name)
{
  MultiStepMethod::SetStarter(IntegrationMethod::GetMethod(), slave_name);
}

//! get the name of the method which is used to start given multistep method
inline const char* GetStarter(const char* name)
{
  return MultiStepMethod::GetStarter(name);
}

//! get the name of the method which is used
//! to start currently used multistep method
inline const char* GetStarter(void)
{
  return MultiStepMethod::GetStarter(IntegrationMethod::GetMethod());
}


////////////////////////////////////////////////////////////////////////////
//! block for numerical integration
//
class Integrator : public aContiBlock {   // integrator
 private:
  Integrator &operator= (const Integrator &x); // disable assignment
  double dd;                           // input value: y'=f(t,y)
  double ddl;                          // the same from previous step
  double ss;                           // status: y = S f(t,y) dt
  double ssl;                          // the same from previous step
 protected:
  Input input;                         // input expression: f(t,y)
  double initval;                      // initial value: y(t0)
  void CtrInit();
  IntegratorContainer::iterator it_list; // position in list of integrators
 public:
  Integrator();                        // implicit CTR (input = 0)
  Integrator(Input i, double initvalue=0);
  Integrator(Integrator &i, double initvalue=0); // copy-ctr
  ~Integrator();
  void Init(double initvalue);         // set value
  void Init() { Init(initval); }       // set initial value
  void Set(double value);              // set value
  Integrator &operator= (double x) { Set(x); return *this; }
  Input SetInput(Input inp) { return input.Set(inp); }
  void Eval();                         // input evaluation
  double Value();                      // status
  double InputValue() { return input.Value(); }
  // interface
  void Save(void) { ddl=dd; ssl=ss; }  // save status
  void Restore(void) { dd=ddl; ss=ssl; }  // restore saved status
  void SetState(double s) { ss=s; }
  double GetState(void) { return ss; }
  void SetOldState(double s) { ssl=s; }
  double GetOldState(void) { return ssl; }
  void SetDiff(double d) { dd=d; }
  double GetDiff(void) { return dd; }
  void SetOldDiff(double d) { ddl=d; }
  double GetOldDiff(void) { return ddl; }
  virtual const char *Name() const;
};


////////////////////////////////////////////////////////////////////////////
//! Status variables (memory)
//
class Status : public aContiBlock1 {   // state-variables
  Status(Status&);                     // disable copy ctor
  void operator= (Status&);            // disable assignment
 protected:
  double initval;                      // initial value
  bool ValueOK;                        // valid-flag ###
  void CtrInit();
  StatusContainer::iterator it_list;   // position in list of status variables
 public:
  double st;                           // status
  double stl;                          // status from previous step
  Status(Input i, double initvalue=0);
  ~Status();
  void Init(double initvalue);         // value initialization
  void Init() { Init(initval); }
  void Set(double value);              // set status of block
  virtual void Eval();                 // block evaluation
  virtual double Value();              // block status
  // interface
  void Save() { stl = st; }            // save status
  void Restore();                      // restore saved status
  void SetState(double s) { st=s; }
  double GetState(void) { return st; }
  void SetOldState(double s) { stl=s; }
  double GetOldState(void) { return stl; }
  void SetValid(bool flag) { ValueOK = flag; } // set valid flag
  virtual const char *Name() const;
};


////////////////////////////////////////////////////////////////////////////
// Nonlinear blocks

////////////////////////////////////////////////////////////////////////////
//! nonlinear block - hysteresis
//
class Hyst : public Status {
  protected:
    double p1,p2;
    double y1,y2;
    double tga;
    virtual void Eval();           // block evaluation
  public:
    Hyst(Input i, double p1, double p2, double y1, double y2, double tga);
  virtual const char *Name() const;
};

typedef Hyst Hysteresis; // alias name


////////////////////////////////////////////////////////////////////////////
//! nonlinear block - backlash
//
class Blash : public Status {    // backlash
  protected:
  double p1,p2;
  double tga;
 public:
  Blash(Input i, double p1, double p2, double tga);
  virtual void Eval();           // block evaluation
  virtual const char *Name() const;
};

typedef Blash Backlash; // alias

////////////////////////////////////////////////////////////////////////////
//! Relay detects exact time of switch
//
class Relay : public Status {   // general relay block
  protected:
  double p1,p2,p3,p4;           // points on x-axis
  double y1,y2;                 // points on y-axis
 public:
  Relay(Input i, double p1, double p2, double p3, double p4,
	         double y1, double y2);
  virtual void Eval();          // block evaluation
  virtual const char *Name() const;
};


////////////////////////////////////////////////////////////////////////////
// Continuous block functions
//

//! Block binary arithmetic operators:
Input operator + (Input a, Input b);
Input operator - (Input a, Input b);
Input operator * (Input a, Input b);
Input operator / (Input a, Input b);

//! Block unary operators:
Input operator - (Input a);

//! Block functions:
Input Abs(Input x);             // abs(x)     absolute value
Input Sin(Input x);             // sin(x)
Input Cos(Input x);             // cos(x)
Input Tan(Input x);             // tan(x)
Input ASin(Input x);            // asin(x)
Input ACos(Input x);            // acos(x)
Input ATan(Input x);            // atan(x)
Input ATan2(Input y, Input x);  // atan2(y,x)
Input Exp(Input x);             // exp(x)     e ** x
Input Log10(Input x);           // log10(x)
Input Ln(Input x);              // ln(x)      natural logarithm
Input Pow(Input x, Input y);    // pow(x,y)   x ** y
Input Sqr(Input x);             // square     x * x
Input Sqrt(Input x);            // square root
Input Min(Input x, Input y);    // minimum
Input Max(Input x, Input y);    // maximum
Input Sign(Input x);            // signum  ###warning-steps-not-detected


////////////////////////////////////////////////////////////////////////////
//! block parametrized by function with single argument
//
class Function1 : public aContiBlock1 {
    Function1(const Function1&); // ## disable
    Function1&operator=(const Function1&); // ## disable
  double (*f)(double); // pointer to function
 public:
  Function1(Input i, double (*pf)(double));
  virtual double Value();
  virtual const char *Name() const;
};

////////////////////////////////////////////////////////////////////////////
//! block parametrized by function with two arguments
//
class Function2 : public aContiBlock2 {
    Function2(const Function2&); // ## disable
    Function2&operator=(const Function2&); // ## disable
  double (*f)(double,double); // pointer to function
 public:
  Function2(Input i1, Input i2, double (*pf)(double,double));
  virtual double Value();
  virtual const char *Name() const;
};


////////////////////////////////////////////////////////////////////////////
// CATEGORY: state conditions

////////////////////////////////////////////////////////////////////////////
//! abstract base class for all state-event blocks
//
class aCondition : public aBlock {
  static aCondition *First;            // list of all conditions
  aCondition *Next;                    // next condition in list
  void operator= (aCondition&);        // disable operation
  aCondition(aCondition&);             // disable operation
 public:
  aCondition();
  ~aCondition();
  static void InitAll();
  static void SetAll();
  static void TestAll();
  static void AllActions();
  static bool isAny();
  virtual bool Value()=0;              // state of condition
 private:
  virtual void Init()=0;
  virtual void SetNewStatus()=0;
  virtual bool Test()=0;               // test of condition
  virtual void Action()=0;             // state event action
};

////////////////////////////////////////////////////////////////////////////
//! change detector - Boolean version (int)
//
class Condition : public aCondition {  // state condition
  Input in;                            // block input
  unsigned char cc;                    // state
  unsigned char ccl;                   // old state
  virtual void Init();
  virtual void SetNewStatus();
 protected:
  virtual bool Test();                  // test function (input >= 0.0)
  bool Up()     { return ccl<cc; }      // change: FALSE->TRUE
  bool Down()   { return ccl>cc; }      // change: TRUE->FALSE
  bool Change() { return ccl!=cc; }     // both changes
 public:
  Condition(Input i);
  ~Condition();
  virtual bool Value() { return cc; }   // status
  Input SetInput(Input inp) { return in.Set(inp); } // change input block
  virtual void Action()=0;             // state event
  virtual const char *Name() const;
};

////////////////////////////////////////////////////////////////////////////
//! detector of FALSE-->TRUE
//
class ConditionUp : public Condition { // state event on FALSE-->TRUE change
 public:
  ConditionUp(Input i) : Condition(i) {}
  bool Test() { return Condition::Test() && Up(); }
  virtual void Action()=0;             // state event
  virtual const char *Name() const;
};

////////////////////////////////////////////////////////////////////////////
//! detector of TRUE-->FALSE
//
class ConditionDown : public Condition { // state event on TRUE-->FALSE change
 public:
  ConditionDown(Input i) : Condition(i) {}
  bool Test() { return Condition::Test() && Down(); }
  virtual void Action()=0;                // state event
  virtual const char *Name() const;
};




////////////////////////////////////////////////////////////////////////////
// CATEGORY: non-linear blocks


////////////////////////////////////////////////////////////////////////////
//! limitation
//
class Lim : public aContiBlock1 {
  protected:
  double low, high;               // limits
  double tgalpha;                 //
 public:
  Lim(Input in, double l, double h, double tga=1.0);
  virtual void _Eval();           // evaluation with loop check
  virtual double Value();         // returned value
  virtual const char *Name() const;
};

typedef Lim Limitation;

////////////////////////////////////////////////////////////////////////////
//! dead zone
//
class Insv : public aContiBlock1 {
  protected:
  double low, high;               // limits
  double tgalpha,tgbeta;          //
 public:
  Insv(Input in, double l, double h, double tga=1.0, double tgb=1.0);
  virtual void _Eval();           // evaluation with loop check
  virtual double Value();         // returned value
  virtual const char *Name() const;
};

typedef Insv DeadZone;

////////////////////////////////////////////////////////////////////////////
//! quantizer
//
class Qntzr : public aContiBlock1 {
  protected:
  double step;                    // quantum
 public:
  Qntzr(Input in, double qstep=1.0);
  virtual void _Eval();           // evaluation with loop check
  virtual double Value();         // returned value
  virtual const char *Name() const;
};

typedef Qntzr Quantizer;

////////////////////////////////////////////////////////////////////////////
//! friction
//
class Frict : public aContiBlock1 {
  protected:
  double low, high;               // limits
  double tgalpha;                 //
 public:
  Frict(Input in, double l, double h, double tga=1.0);
  virtual void _Eval();           // evaluation with loop check
  virtual double Value();         // returned value
  virtual const char *Name() const;
};

typedef Frict Friction;

////////////////////////////////////////////////////////////////////////////
//! function defined by given table of values
//
//TODO: reimplement with std::vector
class Rline : public aContiBlock1 {
    Rline(const Rline&); // ## disable
    Rline&operator=(const Rline&); // ## disable
  protected:
  int n;                          // table item number
  double *tableX;                 // tabulka X
  double *tableY;                 // tabulka Y
 public:
  Rline(Input in, int num, double *X, double *Y);
  ~Rline();
  virtual void _Eval();           // evaluation with loop check
  virtual double Value();         // returned value
  virtual const char *Name() const;
};

////////////////////////////////////////////////////////////////////////////
// CATEGORY: algebraic loop solvers


////////////////////////////////////////////////////////////////////////////
//! abstract base for algebraic loop solvers
//
class AlgLoop : public aContiBlock1 {
 protected:
  double Eps; // required accuracy
  unsigned long MaxIt; // max. number of iterations
  double TA; // boundary values of interval
  double TB; // (also used for convergency check)
  double T0; // initial value
  bool was_cycle; // flag for going through loop
  int phase; // phase of computation
  double root; // root of equation
 public:
  AlgLoop(Input i, double eps, unsigned long max_it, // constructor
          double t_min, double t_max, double t0);
  void Set(double eps, unsigned long max_it, // set parameters
           double t_min, double t_max, double t0);
  void Set(double eps, unsigned long max_it, // set parameters
           double t_min, double t_max);
  virtual double Value()=0; // returned value
  virtual const char *Name() const;
}; // AlgLoop


////////////////////////////////////////////////////////////////////////////
//! solve using iterations
//
class Iterations : public AlgLoop {
 public:
  Iterations(Input i, double eps, unsigned long max_it,
             double t_min, double t_max, double t0):
    AlgLoop(i, eps, max_it, t_min, t_max, t0)
    {/*nothing*/}
  virtual double Value(); // returned value
};


////////////////////////////////////////////////////////////////////////////
//! solve using halving interval method
//
class Halvint : public AlgLoop {
 public:
  Halvint(Input i, double eps, unsigned long max_it,
          double t_min, double t_max):
    AlgLoop(i, eps, max_it, t_min, t_max, t_min)
    {/*nothing*/}
  virtual double Value(); // returned value
};


////////////////////////////////////////////////////////////////////////////
//! solve using regula falsi method
//
class RegulaFalsi : public AlgLoop {
  double eps_root; // root for force precision test
 public:
  RegulaFalsi(Input i, double eps, unsigned long max_it,
              double t_min, double t_max):
    AlgLoop(i, eps, max_it, t_min, t_max, t_min),
    eps_root(0)
    {/*nothing*/}
  virtual double Value(); // returned value
};


////////////////////////////////////////////////////////////////////////////
//! solve using modified Newton's method
//
class Newton : public AlgLoop {
  double eps_root; // root for force precision test
  double prev_root; // root from previous iteration step
 public:
  Newton(Input i, double eps, unsigned long max_it,
         double t_min, double t_max, double t0):
    AlgLoop(i, eps, max_it, t_min, t_max, t0),
    eps_root(0),
    prev_root(0)
    {/*nothing*/}
  virtual double Value(); // returned value
};


////////////////////////////////////////////////////////////////////////////
// CATEGORY: global functions

////////////////////////////////////////////////////////////////////////////
// Naming Subsystem --- SimObject names (text identifiers)
// this is system dictionary:   relation: objectptr---textname
//
void SetName(SimObject &o, const char *name);
void SetName(SimObject *o, const char *name);
void RemoveName(SimObject &o);
void RemoveName(SimObject *o);
const char *GetName(SimObject &o);
const char *GetName(SimObject *o);


////////////////////////////////////////////////////////////////////////////
// InstallBreak --- set function for checking if user breaks simulation
//
// f is called at each step of simulation
//
void InstallBreak(void (*f)());  // f can call Stop() or Abort()


////////////////////////////////////////////////////////////////////////////
//! basic synchronization tool for processes
//
class Semaphore : public SimObject {
  protected:
  int n;
 public:
  Queue Q;                                      // internal quqeue
  Semaphore();                                  // constructor
  Semaphore(const char *_name);                 // with associated name
  virtual ~Semaphore();
  virtual void Clear();                         // initialization
  virtual void Output();
  virtual void P();                             // P operation
  virtual void V();                             // V operation
  operator Semaphore* () { return this; }
};

////////////////////////////////////////////////////////////////////////////
//! synchronization tool for processes
//
class Barrier : public SimObject {
  Barrier(const Barrier&);              // ## disable
  Barrier&operator=(const Barrier&);    // ## disable
 protected:
  Entity **waiting;     // array of waiting entities (fixed size)
  unsigned n;           // current number of waiting entities
  unsigned maxn;        // barrier height/size
  void Init();          // initialization
 public:
  Barrier(unsigned N);
  Barrier(const char *_name, unsigned N);
  virtual ~Barrier();
  void ChangeHeight(unsigned new_maxn);         // change size
  unsigned Height() const { return maxn; }      // barrier size
  virtual void Enter(Entity *e);        // wait for barrier break TODO: remove
  virtual bool Wait();                  // wait for barrier break (Current)
  virtual int Break();                  // activate all waiting entities
  virtual void Clear();                 // initialization
  virtual void Output() const;          // print status
};

/////////////////////////////////////////////////////////////////////////////
/// internal statistics structure
// TODO: add interface and hide variables
struct SIMLIB_statistics_t {
  double StartTime;
  double EndTime;
  long   EventCount;
  long   StepCount;
  double MinStep;
  double MaxStep;
  /// constructor runs SIMLIB_statistics_t::Init()
  SIMLIB_statistics_t();
  /// initialize - used at the start of each Run()
  void Init();
  /// print run-time statistics to output
  void Output() const;
};

/// interface to internal run-time statistics structure
extern const SIMLIB_statistics_t & SIMLIB_statistics;

} // namespace simlib3

using namespace simlib3;

#endif // ifndef __SIMLIB__

