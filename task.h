#ifndef TASK_H
#define TASK_H

#include <argos3/core/utility/math/vector2.h>

struct STask {
   CVector2 Position; ///< where the task is located
   Real Radius;       ///< Radius of the task
};

struct SAssignment {
   // Robot position
   CVector2 position;
   
   // State information
   struct {
       CVector2 approx_food;
       Real food_confidence;
       CVector2 approx_home;
       Real home_confidence;
   } state;
   
   // Food recovery tracking
   int food_recovered;
   
   // Distances to neighbors (will be filled as an array)
   std::vector<Real> dist_to_neighbor;
   
   // Constructor
   SAssignment(size_t un_tasks) : 
       position(0, 0),
       food_recovered(0) {
       state.approx_food = CVector2(0, 0);
       state.food_confidence = 0.0;
       state.approx_home = CVector2(0, 0);
       state.home_confidence = 0.0;
   }
};

#endif // TASK_H
