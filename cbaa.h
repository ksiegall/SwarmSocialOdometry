#ifndef CBAA_H
#define CBAA_H

#include <buzz/argos/buzz_loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include "task.h"

class CCBAA : public CBuzzLoopFunctions {

public:

   CCBAA() {}
   virtual ~CCBAA() {}

   /**
    * Executes user-defined initialization logic.
    * @param t_tree The 'loop_functions' XML configuration tree.
    */
   virtual void Init(TConfigurationNode& t_tree);

   /**
    * Executes user-defined reset logic.
    * This method should restore the state of the simulation at it was right
    * after Init() was called.
    * @see Init()
    */
   virtual void Reset();

   /**
    * Executes user-defined logic right after a control step is executed.
    */
   virtual void PostStep();
   
   /**
    * Returns true if the experiment is finished, false otherwise.
    *
    * This method allows the user to specify experiment-specific ending
    * conditions. If this function returns false and a time limit is set in the
    * .argos file, the experiment will reach the time limit and end there. If no
    * time limit was set, then this function is the only ending condition.
    *
    * @return true if the experiment is finished.
    */
   virtual bool IsExperimentFinished();

   /**
    * Executes user-defined destruction logic.
    * This method should undo whatever is done in Init().
    * @see Init()
    */
   virtual void Destroy();

   /**
    * Returns the floor color at the given position.
    */
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

   /**
    * Called by ARGoS when you press the 'compile' button.
    */
   virtual void BuzzBytecodeUpdated();

   /* Returns task information */
   std::vector<STask> GetTasks() const;

private:

   /** Returns the number of robots */
   int GetNumRobots() const;

private:

   /** Task data */
   std::vector<STask> m_vecTasks;

   /** The output file name */
   std::string m_strOutFile;

   /** The output file stream */
   std::ofstream m_cOutFile;

   /** Random number generator */
   CRandom::CRNG* m_pcRNG;
};

#endif
