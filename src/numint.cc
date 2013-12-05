/////////////////////////////////////////////////////////////////////////////
//! \file numint.cc  Numerical integration methods 
//
// Copyright (c) 1991-2004 Petr Peringer 
// Copyright (c) 1996-1997 David Leska
//
// This library is licensed under GNU Library GPL. See the file COPYING.
//

//
//  numerical integration -- base file
//


////////////////////////////////////////////////////////////////////////////
//  interface
//
#include "simlib.h"
#include "internal.h"
#include "ni_abm4.h"
#include "ni_euler.h"
#include "ni_fw.h"
#include "ni_rke.h"
#include "ni_rkf3.h"
#include "ni_rkf5.h"
#include "ni_rkf8.h"
#include <cstddef>
#include <cstring>


////////////////////////////////////////////////////////////////////////////
//  implementation
//

namespace simlib3 {

SIMLIB_IMPLEMENTATION;

// stack options in Borland C++ compiler
#ifdef __BORLANDC__
  // increase stack size
  // WARNING: under MS-Windows must be specified in *.DEF file !!! ###
  // extern unsigned _stklen = 0xA000;
  // set option 'check stack owerflow' on
#  pragma option -N
#endif


////////////////////////////////////////////////////////////////////////////
//  outline methods of class IntegrationMethod
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::StepSim
//  step of numerical integration method
//
void IntegrationMethod::StepSim(void)
{
  Dprintf(("==================== continuous step BEGIN %.15g",Time));
#ifndef NDEBUG
  double Step_StartTime = Time;
#endif
  SIMLIB_DynamicFlag = true; // numerical integration is running
  if(Prepare()) { // initialize integration step (condition is not changed)
    if(IntegratorContainer::isAny()) { // are there any integrators?
      CurrentMethodPtr->Integrate(); // * numerical integration *
    } else {     // model without integrators
      Iterate(); // compute new values of state blocks
    }
    Summarize(); // set up new state in the system
  }
  SIMLIB_DynamicFlag = false; // end of numerical integration
  Dprintf((" Step length = %g ", Time - Step_StartTime ));
  Dprintf(("==================== continuous step END %.15g",Time));
} // IntegrationMethod::StepSim


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::SetMethod
//  set method which will be used
//
void IntegrationMethod::SetMethod(const char* name)
{
  Dprintf(("SetMethod(%s, %s)", name));
  if(SIMLIB_DynamicFlag) {
    SIMLIB_error(NI_CantSetMethod);  // can't in 'dynamic section' !!!
  }
  CurrentMethodPtr->TurnOff();  // suspend present method
  CurrentMethodPtr=SearchMethod(name);  // set new method
} // IntegrationMethod::SetMethod


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::Iterate
//  compute new values of state blocks in model without integrators
//
void IntegrationMethod::Iterate(void)
{
  Dprintf(("IntegrationMethod::Iterate()"));
  while(1) {
    SIMLIB_StepSize = max(SIMLIB_MinStep, SIMLIB_StepSize);
    SIMLIB_ContractStepFlag = false;           // don't reduce step
    SIMLIB_ContractStep = 0.5*SIMLIB_StepSize; // implicitly reduce to half
    _SetTime(Time, SIMLIB_StepStartTime + SIMLIB_StepSize);
    SIMLIB_DeltaTime = SIMLIB_StepSize;

    SIMLIB_Dynamic(); // evaluate new state of model - only state blocks
    Condition::TestAll(); // check on changes of state conditions

    if(!SIMLIB_ContractStepFlag) 
      break;
    if(SIMLIB_StepSize<=SIMLIB_MinStep) 
      break;
    IsEndStepEvent = false;     // no event will be at end of step
    SIMLIB_StepSize = SIMLIB_ContractStep;
    StatusContainer::LtoN();
  }
} // IntegrationMethod::Iterate


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::Summarize
//  set up new state after execution of integration step
//
void IntegrationMethod::Summarize(void)
{
  Dprintf(("IntegrationMethod::Summarize()"));
  SIMLIB_StepStartTime = Time;
  SIMLIB_DeltaTime = 0.0;
  IntegratorContainer::NtoL();
  StatusContainer::NtoL();
  if(IsEndStepEvent)          // event at the end of step
    _SetTime(Time, NextTime); // suppress inaccuracy of float
} // IntegrationMethod::Summarize


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::Prepare -- initialize integration step
//
bool IntegrationMethod::Prepare(void)
{
  SIMLIB_StepSize = SIMLIB_OptStep; // optimal step size at start 

  Dprintf(("IntegrationMethod::Prepare()"));

 // If an event is scheduled within the step,
  // set on flag, that will be event at the end of the step
  IsEndStepEvent=(bool)(double(Time)+1.01*SIMLIB_StepSize>=NextTime);//1.1???
  // and adjust step size, so that event will take place at end of step
  if(IsEndStepEvent) 
    SIMLIB_StepSize = double(NextTime)-double(Time);

  // set up auxiliary variables
  SIMLIB_StepStartTime = Time; // start time of integration
  SIMLIB_DeltaTime = 0.0;      // time since beginning of integration

  if(SIMLIB_ResetStatus) { // initialization of integration is requested
                           // eg. on event - non-continuous change of state

    // --------------- initialization of step sequence ----------------

    IntegratorContainer::NtoL();// store initial value ...
    StatusContainer::NtoL();

    SIMLIB_Dynamic();           // compute new (initial) state (0)
    Condition::TestAll();       // check on changes of state conditions

    IntegratorContainer::NtoL();// store new value ...
    StatusContainer::NtoL();

    SIMLIB_ResetStatus=false;   // clear reset flag
    if(SIMLIB_ConditionFlag)
      return false;             // condition has been changed => terminate step
  }

  if(SIMLIB_StepSize<=0)
    SIMLIB_error(NI_IlStepSize); // error of integration

  CurrentMethodPtr->PrepareStep(); // prepare current method for single step

  return true;
} // IntegrationMethod::Prepare


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::StateCond
//  check on changes of state conditions
//
bool IntegrationMethod::StateCond(void)
{
  Dprintf(("IntegrationMethod::StateCond()"));

  Condition::TestAll(); // check on changes

  if(SIMLIB_ContractStepFlag && SIMLIB_StepSize>SIMLIB_MinStep) { 
    // step reducing is requested and it is possible
    SIMLIB_StepSize = SIMLIB_ContractStep; // reduce step to demanded size
                                           // implicitly to quater of step
    IsEndStepEvent = false; // no event will be scheduled at end of step
    return true;
  }
  return false;
} // IntegrationMethod::StateCond


////////////////////////////////////////////////////////////////////////////
/// Constructor registrates method and names it
//
IntegrationMethod::IntegrationMethod(const char *name):
  PrevINum(0)
{
    Dprintf(("constructor[IntegrationMethod]: \"%s\"(%p)", name, MthLstPtr));

    // name the method
    char *Name = new char[strlen(name) + 1];
    strcpy(Name, name);
    method_name = Name;

    // is list of methods not created?
    if (MthLstPtr == NULL) {
        MthLstPtr = new dlist < IntegrationMethod * >;  // create it
    }
    //TODO use map<name,method> ?
    // registrate the method in list
    for (ItList = MthLstPtr->begin(); ItList != MthLstPtr->end(); ItList++) {
        if (strcmp((*ItList)->method_name, method_name) == 0) { // name should be unique
            SIMLIB_error(NI_MultDefMeth);
        }
    }
    ItList = MthLstPtr->insert(MthLstPtr->end(), this); // insert method into list
    PtrMList = &MList;          // initialize pointer to list of memory buffers
} // IntegrationMethod::IntegrationMethod


////////////////////////////////////////////////////////////////////////////
/// Destructor unregistrates method
//
IntegrationMethod::~IntegrationMethod() {
  Dprintf(("destructor[IntegrationMethod]"));
  // is list of methods not created?
  if(MthLstPtr==NULL) {
    SIMLIB_internal_error();  // internal error (probably impossible)
  }
  MthLstPtr->erase(ItList);  // unregistrate the method
  delete[] method_name;  // free dynamic data
  if(MthLstPtr->empty()) {  // destroy the list
    delete MthLstPtr;
    MthLstPtr=NULL;
  }
} // IntegrationMethod::~IntegrationMethod


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::SearchMethod
//  search method in the list of registrated methods
//
IntegrationMethod* IntegrationMethod::SearchMethod(const char* name)
{
  dlist<IntegrationMethod*>::iterator it; // iterator for searching in the list
  dlist<IntegrationMethod*>::iterator end_it; // iterator to end of the list

  Dprintf(("IntegrationMethod::SearchMethod(\"%s\")", name));
  // is list of methods created?
  if(MthLstPtr!=NULL) {
    // search for method in the list of registrated methods
    for(it=MthLstPtr->begin(), end_it=MthLstPtr->end(); it!=end_it; it++) {
      if(strcmp((*it)->method_name,name)==0) {  // find?
        return *it; // return pointer to the method
      }
    }
  }
  SIMLIB_error(NI_UnknownMeth); // name is unknown (or list is not created)
  return 0; // never reached
} // IntegrationMethod::SearchMethod


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::TurnOff
//  turn off integration method: flush memories etc...
//
void IntegrationMethod::TurnOff(void)
{
  Dprintf(("IntegrationMethod::TurnOff()"));
  Resize(0);  // flush memories
  PrevINum=0; // remember it for next usage
} // IntegrationMethod::TurnOff


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::Resize
//  resize all the memories to the given size
//
void IntegrationMethod::Resize(size_t size)
{
  Dprintf(("IntegrationMethod::Resize(%lu)", (long unsigned)size));
  for(dlist<Memory*>::iterator it=MList.begin(); it!=MList.end(); it++) {
    (*it)->Resize(size);
  }
} // IntegrationMethod::Resize


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::PrepareStep
//  prepare object for integration step
//
bool IntegrationMethod::PrepareStep(void)
{
  Dprintf(("IntegrationMethod::PrepareStep()"));
  // has # of integrators been changed?
  if(PrevINum!=IntegratorContainer::Size()) {
    PrevINum=IntegratorContainer::Size();  // retain # of integrators
    Resize(PrevINum);  // change size of auxiliary memories
    return true;  // there are changes in system
  } else {
    return false; // no changes
  }
} // IntegrationMethod::PrepareStep

/* auxiliary functions for user to add own method */

////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::InitStep
//  initialize step
//
void IntegrationMethod::InitStep(double step_frag)
{
  SIMLIB_StepSize = max(SIMLIB_StepSize, SIMLIB_MinStep); // low step limit
  SIMLIB_StepSize = min(SIMLIB_StepSize, SIMLIB_MaxStep); // high step limit
  SIMLIB_ContractStepFlag = false;  // clear reduce step flag
  // implicitly reduce to half step
  SIMLIB_ContractStep = step_frag*SIMLIB_StepSize;
} /* InitStep */


////////////////////////////////////////////////////////////////////////////
//  IntegrationMethod::FunCall
//  evaluate y'(t) = f(t, y(t))
//
void IntegrationMethod::FunCall(double step_frag)
{
  if(step_frag==1.0) {  // accuracy!
    // substep time
    _SetTime(Time, SIMLIB_StepStartTime + SIMLIB_StepSize);
    SIMLIB_DeltaTime = SIMLIB_StepSize;
  } else {
    // substep time
    _SetTime(Time, SIMLIB_StepStartTime + step_frag*SIMLIB_StepSize);
    SIMLIB_DeltaTime = double(Time) - SIMLIB_StepStartTime;
  }
  SIMLIB_Dynamic(); // evaluate new state of model
} /* FunCall */


////////////////////////////////////////////////////////////////////////////
//  outline methods of class IntegrationMethod::Memory
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//  Memory::Resize
//  change size of array to cs, content will be undefined!
//
void IntegrationMethod::Memory::Resize(size_t cs)
{
  Dprintf(("IntegrationMethod::Memory::Resize(%lu)",(long unsigned)cs));
  if(cs == 0) {  // zero size
    delete[] arr;
    arr = NULL;
    mem_size = 0;
  } else {
    cs = (1+(cs-1)/page_size)*page_size;  // round new size to page size
    if(cs != mem_size) {  // demanded size is too small or too large
      delete[] arr;       // new allocation
      arr = new double[cs];
      if(arr==NULL) {  // cannot allocate memory
        SIMLIB_internal_error();  // there is an exception in new versions of C++
      }

#ifdef __BCPLUSPLUS__
      // problems with FU***NG MeS-DOS segment memory model!
      if(cs > 0xFFFF/sizeof(double) ) SIMLIB_internal_error();
#endif

      mem_size = cs;

      Dprintf(("##### reallocation to mem_size=%lu",(long unsigned)mem_size));

    }
  }
} // Memory::Resize


////////////////////////////////////////////////////////////////////////////
//  Memory::Memory
//  create empty memory, put object into list
//
IntegrationMethod::Memory::Memory(dlist<Memory*>* PtrList) :
  arr(NULL), mem_size(0), ListPtr(PtrList)
{
//  Dprintf(("constructor: IntegrationMethod::Memory::Memory(%p)", PtrList));
  it_list=ListPtr->insert(ListPtr->end(),this); // put memory into list
}


////////////////////////////////////////////////////////////////////////////
//  Memory::~Memory
//  free dynamic data, remove object from list of memories
//
IntegrationMethod::Memory::~Memory()
{
//  Dprintf(("destructor: IntegrationMethod::Memory::~Memory()"));
  delete[] arr;  // free allocated memory
  arr=NULL;
  mem_size=0;
  ListPtr->erase(it_list);  // remove object from list
}


////////////////////////////////////////////////////////////////////////////
//  outline methods of class MultiStepMethod
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//  MultiStepMethod::MultiStepMethod
//  Constructor initializes method
//
MultiStepMethod::MultiStepMethod(const char* name, const char* slave_name) :
  IntegrationMethod(name),
  Slave_Ptr(NULL)
{
  Dprintf(("constructor[MultiStepMethod](%s, %s)", name, slave_name));
  // name the starting method
  SlaveName=new char[strlen(slave_name)+1];
  strcpy(SlaveName, slave_name);
  // note: cannot initialize Slave_Ptr here,
  // slave need not exist now because it is a global object
  // you must determine it by a name
} // MultiStepMethod::MultiStepMethod


////////////////////////////////////////////////////////////////////////////
//  MultiStepMethod::~MultiStepMethod
//  Destructor, it frees dynamic data
//
MultiStepMethod::~MultiStepMethod()
{
  Dprintf(("destructor[MultiStepMethod]"));
  delete[] SlaveName;  // free dynamic data
} // MultiStepMethod::~MultiStepMethod


////////////////////////////////////////////////////////////////////////////
//  MultiStepMethod::SlavePtr
//  return pointer to the starting method (initialize it also)
//
SingleStepMethod* MultiStepMethod::SlavePtr(void)
{
  if(Slave_Ptr==NULL) {  // pointer is not valid
    Slave_Ptr=(SingleStepMethod*)SearchMethod(SlaveName);  // obtain method
    if(!Slave_Ptr->IsSingleStep()) {  // Is it a single-step method?
      SIMLIB_error(NI_NotSingleStep);  // no -- error
    }
  }
  return Slave_Ptr;
} // MultiStepMethod::SlavePtr


////////////////////////////////////////////////////////////////////////////
//  MultiStepMethod::PrepareStep
//  prepare the object for the step of integration
//
bool MultiStepMethod::PrepareStep(void)
{
  bool ichanges;
  Dprintf(("MultiStepMethod::PrepareStep()"));
  // prepare inherited part & test if # of integrators has been changed
  ichanges = IntegrationMethod::PrepareStep();
  SlavePtr()->SetStartMode(true); // called method is used for starting
  (void)SlavePtr()->PrepareStep(); // prepare your slave
  return ichanges; // indicate changes
} // MultiStepMethod::PrepareStep


////////////////////////////////////////////////////////////////////////////
//  MultiStepMethod::TurnOff
//  turn off integration method and its slave (starter)
//
void MultiStepMethod::TurnOff(void)
{
  Dprintf(("MultiStepMethod::TurnOff()"));
  IntegrationMethod::TurnOff();  // turn off inherited part
  SlavePtr()->SetStartMode(false);  // free slave
  SlavePtr()->TurnOff();  // turn off your slave
} // MultiStepMethod::TurnOff


////////////////////////////////////////////////////////////////////////////
//  MultiStepMethod::SetStarter
//  set starting method for given multi-step method
//
void MultiStepMethod::SetStarter(const char* name, const char* slave_name)
{
  Dprintf(("SetStarter(%s, %s)", name, slave_name));
  MultiStepMethod* ptr=(MultiStepMethod*)SearchMethod(name);
  if(ptr->IsSingleStep()) {  // is current method single-step?
    SIMLIB_error(NI_NotMultiStep);  // yes -- error
  }
  ptr->SetStarter(slave_name);  // set it
} // MultiStepMethod::SetStarter


////////////////////////////////////////////////////////////////////////////
//  MultiStepMethod::SetStarter
//  set starting method for the multi-step method
//
void MultiStepMethod::SetStarter(const char* slave_name)
{
  Dprintf(("SetStarter(%s)", slave_name));
  if(SIMLIB_DynamicFlag) {
    SIMLIB_error(NI_CantSetStarter);  // can't in 'dynamic section' !!!
  }
  // name the starting method
  delete[] SlaveName;
  SlaveName=new char[strlen(slave_name)+1];
  strcpy(SlaveName, slave_name);
  Slave_Ptr=NULL;  // throw away old starting method
} // MultiStepMethod::SetStarter


////////////////////////////////////////////////////////////////////////////
//  MultiStepMethod::GetStarter
//  obtain the name of the starting method of given method
//
const char* MultiStepMethod::GetStarter(const char* name)
{
  Dprintf(("GetStarter(%s)", name));
  MultiStepMethod* ptr=(MultiStepMethod*)SearchMethod(name);
  if(ptr->IsSingleStep()) {  // is the method single-step?
    return NULL;  // no starter is used
  }
  return ptr->SlaveName;  // get it
} // MultiStepMethod::GetStarter


////////////////////////////////////////////////////////////////////////////
//  outline methods of class StatusIntegrationMethod
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//  StatusMethod::StatusMethod -- initailization
//
StatusMethod::StatusMethod(const char* name):
  SingleStepMethod(name),
  PrevStatusNum(0)
{
  Dprintf(("constructor[StatusIntegrationMethod]: \"%s\"", name));
  PtrStatusMList=&StatusMList;  // initialize
} // StatusMethod


////////////////////////////////////////////////////////////////////////////
//  StatusMethod::TurnOff
//  turn off integration method: flush memories etc...
//
void StatusMethod::TurnOff(void)
{
  Dprintf(("StatusMethod::TurnOff()"));
  IntegrationMethod::TurnOff();  // turn off inherited part
  StatusResize(0);  // flush status memories
  PrevStatusNum=0; // remember it for next usage
} // TurnOff


////////////////////////////////////////////////////////////////////////////
//  StatusMethod::PrepareStep
//  prepare object for integration step
//
bool StatusMethod::PrepareStep(void)
{
  Dprintf(("StatusMethod::PrepareStep()"));
  // prepare inherited part -- test for changes of # of integrators
  bool ichanges = IntegrationMethod::PrepareStep();
  // has # of status variables been changed?
  if(PrevStatusNum!=StatusContainer::Size()) {
    PrevStatusNum=StatusContainer::Size();  // retain # of status variables
    StatusResize(PrevStatusNum);  // change size of status auxiliary memories
    return true;  // there are changes in system
  } else {
    return ichanges;  // changes would be only in iherited part
  }
} // PrepareStep


////////////////////////////////////////////////////////////////////////////
//  StatusMethod::StatusResize
//  resize status memories to the given size
//
void StatusMethod::StatusResize(size_t size)
{
  Dprintf(("StatusMethod::StatusResize(%lu)", (long unsigned)size));
  for(dlist<Memory*>::iterator it=StatusMList.begin();
      it!=StatusMList.end();
      it++) {
    (*it)->Resize(size);
  }
} // StatusResize


////////////////////////////////////////////////////////////////////////////
//  StatusMethod::StoreState
//  store state of integrators and status variables
//
void StatusMethod::StoreState(Memory& di, Memory& si, StatusMemory& xi)
{
  register size_t i;
  IntegratorContainer::iterator ip, end_it;
  StatusContainer::iterator sp, status_end_it;

  for(ip=IntegratorContainer::Begin(), end_it=IntegratorContainer::End(), i=0;
      ip!=end_it;
      ip++, i++)
  {
    di[i]=(*ip)->GetDiff();
    si[i]=(*ip)->GetState();
  }

  for(sp=StatusContainer::Begin(), status_end_it=StatusContainer::End(), i=0;
      sp!=status_end_it; sp++, i++)
  {
    xi[i]=(*sp)->GetState();
  }
} // StoreState


////////////////////////////////////////////////////////////////////////////
//  StatusMethod::RestoreHalfState
//  restore state of integrators and status variables
//
void StatusMethod::RestoreState(double dthlf, Memory& di, Memory& si,
                                StatusMemory& xi)
{
  register size_t i;
  IntegratorContainer::iterator ip, end_it;
  StatusContainer::iterator sp, status_end_it;

  for(ip=IntegratorContainer::Begin(), end_it=IntegratorContainer::End(), i=0;
      ip!=end_it;
      ip++, i++)
  {
    (*ip)->SetDiff(di[i]);
    (*ip)->SetState(si[i]);
  }

  for(sp=StatusContainer::Begin(), status_end_it=StatusContainer::End(), i=0;
      sp!=status_end_it; sp++, i++)
  {
    (*sp)->SetState(xi[i]);
  }

  _SetTime(Time, SIMLIB_StepStartTime + dthlf); // half step
  IsEndStepEvent = false; // no event will be scheduled at end of step
} // RestoreState


////////////////////////////////////////////////////////////////////////////
//  StatusMethod::GoToState
//  move startpoint to given state
//
void StatusMethod::GoToState(Memory& di, Memory& si, StatusMemory& xi)
{
  register size_t i;
  IntegratorContainer::iterator ip, end_it;
  StatusContainer::iterator sp, status_end_it;

  for(ip=IntegratorContainer::Begin(), end_it=IntegratorContainer::End(), i=0;
      ip!=end_it;
      ip++, i++)
  {
    (*ip)->SetOldDiff(di[i]);
    (*ip)->SetOldState(si[i]);
  }

  for(sp=StatusContainer::Begin(), status_end_it=StatusContainer::End(), i=0;
      sp!=status_end_it; sp++, i++)
  {
    (*sp)->SetOldState(xi[i]);
  }
} // GoToState


////////////////////////////////////////////////////////////////////////////
//  initialize static members of classes
//

// size of memory page
const size_t IntegrationMethod::Memory::page_size = 256;

// flag - will be event at the end of the step?
bool IntegrationMethod::IsEndStepEvent=false;

// list of registrated methods
dlist<IntegrationMethod*>* IntegrationMethod::MthLstPtr=NULL;

// pointer to the filled list of memories
dlist<IntegrationMethod::Memory*>* IntegrationMethod::PtrMList;

// pointer to the filled list of status memories
dlist<IntegrationMethod::Memory*>* StatusMethod::PtrStatusMList;


////////////////////////////////////////////////////////////////////////////
//  constitute integration methods
//
ABM4 abm4("abm4", "rkf5");
EULER euler("euler");
FW fw("fw");
RKE rke("rke");
RKF3 rkf3("rkf3");
RKF5 rkf5("rkf5");
RKF8 rkf8("rkf8");


// pointer to the method used at present
// rke is a predefined method
IntegrationMethod* IntegrationMethod::CurrentMethodPtr = &rke;

}
// end

