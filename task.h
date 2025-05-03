#ifndef TASK_H
#define TASK_H

#include <argos3/core/utility/math/vector2.h>

struct STask {
   CVector2 Position; ///< where the task is located
   Real Reward;       ///< the reward received for completing the task
};

struct SAssignment {
   std::vector<bool> xi; ///< vector x_i in the slides, each element is x_ij
   std::vector<Real> yi; ///< vector y_i in the slides, each element is y_ij
   
   SAssignment(size_t un_tasks) :
      xi(un_tasks, false),
      yi(un_tasks, 0.0) {}
};

#endif // TASK_H
