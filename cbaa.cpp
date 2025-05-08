#include "cbaa.h"
#include "buzz/buzzvm.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <map>
#include <utility>

/****************************************/
/****************************************/


// static bool BuzzFetchXij(buzzvm_t t_vm, int j) {
//    buzzobj_t e = BuzzTableGet(t_vm, j);
//    if(!e) {
//       THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s xi[" << j << "] does not exist");
//    }
//    if(!buzzobj_isint(e)) {
//       THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s xi[" << j << "] is not an integer");
//    }
//    int xij = buzzobj_getint(e);
//    if(xij < 0 || xij > 1) {
//       THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s xi[" << j << "] is not 0 or 1");
//    }
//    return xij;
// }

// static Real BuzzFetchYij(buzzvm_t t_vm, int j) {
//    buzzobj_t e = BuzzTableGet(t_vm, j);
//    if(!e) {
//       THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s yi[" << j << "] does not exist");
//    }
//    if(!buzzobj_isfloat(e)) {
//       THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s yi[" << j << "] is not a float");
//    }
//    float yij = buzzobj_getfloat(e);
//    if(yij < 0.0) {
//       THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s yi[" << j << "] cannot be negative");
//    }
//    return yij;
// }


static Real BuzzFetch(buzzvm_t t_vm, const std::string& str_key) {
   buzzobj_t e = BuzzTableGet(t_vm, str_key);
   if(!e) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'" << str_key << "' does not exist");
   }
   if(!buzzobj_isfloat(e)) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'" << str_key << "' is not a float, " << typeid(e).name());
   }
   return buzzobj_getfloat(e);
}

static int BuzzFetchInt(buzzvm_t t_vm, const std::string& str_key) {
   buzzobj_t e = BuzzTableGet(t_vm, str_key);
   if(!e) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'" << str_key << "' does not exist");
   }
   if(!buzzobj_isint(e)) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'" << str_key << "' is not an int, " << typeid(e).name());
   }
   return buzzobj_getint(e);
}


/**
 * Functor to get data from the robots
 */
struct GetRobotData : public CBuzzLoopFunctions::COperation {

   /** Constructor */
   // GetRobotData(size_t n_tasks) : m_nTasks(n_tasks) {}
   GetRobotData(size_t n_tasks){}

   /*
      THINGS WE NEED HERE:
      - (x,y)
      - state:
         - approx_food (x,y), confidence
         - approx_home (x,y), confidence
      - food_recovered (per robot, analyze in post)
      - [dist_to_neighbor] array (used for aggregation analysis)
            - Needs to be added to buzz too

      OTHER:
      - log time
   */
   
   /** The action happens here */
   virtual void operator()(const std::string& str_robot_id,
                           buzzvm_t t_vm) {
   
   //    /* Empty assignment to fill */
   //    SAssignment sAssignment(m_nTasks);
   //    /* Make sure 'xi' and 'yi' exist and are vectors */
   //    BuzzAssertVector(t_vm, "xi");
   //    BuzzAssertVector(t_vm, "yi");
   //    /* Extract data from xi */
   //    BuzzTableOpen(t_vm, "xi");
   //    for(size_t j = 0; j < m_nTasks; ++j) {
   //       sAssignment.xi[j] = BuzzFetchXij(t_vm, j);
   //    }
   //    BuzzTableClose(t_vm);
   //    /* Extract data from yi */
   //    BuzzTableOpen(t_vm, "yi");
   //    for(size_t j = 0; j < m_nTasks; ++j) {
   //       sAssignment.yi[j] = BuzzFetchYij(t_vm, j);
   //    }
   //    BuzzTableClose(t_vm);
   //    /* Save assignment data */
   //    m_mapAssignments.insert(
   //          std::make_pair(str_robot_id, sAssignment));
   //    }


      SAssignment sAssignment(m_nTasks);
       // Get robot position (x,y)
       BuzzTableOpen(t_vm, "pose");
       BuzzTableOpenNested(t_vm, "position");
         sAssignment.position.Set(
               BuzzFetch(t_vm, "x"),
               BuzzFetch(t_vm, "y"));
      BuzzTableCloseNested(t_vm);
      BuzzTableClose(t_vm);

      // Get state information
      BuzzTableOpen(t_vm, "approx_food_pos");
         sAssignment.state.approx_food.Set(
               BuzzFetch(t_vm, "x"),
               BuzzFetch(t_vm, "y"));
      BuzzTableClose(t_vm);
      BuzzTableOpen(t_vm, "approx_home_pos");
         sAssignment.state.approx_home.Set(
               BuzzFetch(t_vm, "x"),
               BuzzFetch(t_vm, "y"));
      BuzzTableClose(t_vm);
      BuzzTableOpen(t_vm, "state");
      sAssignment.state.food_confidence = BuzzFetch(t_vm, "conf_food");
      sAssignment.state.home_confidence = BuzzFetch(t_vm, "conf_home");
         sAssignment.food_recovered = BuzzFetchInt(t_vm, "food_recovered");   
      BuzzTableClose(t_vm);


      /* Save assignment data */
      m_mapAssignments.insert(
            std::make_pair(str_robot_id, sAssignment));
      }
   
      size_t m_nTasks;
      std::map<std::string, SAssignment> m_mapAssignments;
   };
   
   // /****************************************/
   /****************************************/
   
   // /**
   //  * Functor to put the task info in the Buzz VMs.
   //  */
   struct PutTables : public CBuzzLoopFunctions::COperation {
      
      PutTables(const std::vector<STask>& nest_pair_var) : nest_pair_one(nest_pair_var) {}
      
      /** The action happens here */
      virtual void operator()(const std::string& str_robot_id, buzzvm_t t_vm) {
         /* Set the values of the table 'food' in the Buzz VM */
         BuzzTableOpen(t_vm, "nest_pair_one");
         /* Put (x,y) in the food table */
         BuzzTableOpenNested(t_vm, "nest");
         BuzzTablePut(t_vm, "x", static_cast<float>(nest_pair_one[0].Position.GetX()));
         BuzzTablePut(t_vm, "y", static_cast<float>(nest_pair_one[0].Position.GetY()));
         BuzzTablePut(t_vm, "radius", static_cast<float>(nest_pair_one[0].Radius));
         /* Done with the food table */
         BuzzTableCloseNested(t_vm);
         
         /* Set the values of the table 'nest' in the Buzz VM */
         BuzzTableOpenNested(t_vm, "food");
         /* Put (x,y) in the nest table */
         BuzzTablePut(t_vm, "x", static_cast<float>(nest_pair_one[1].Position.GetX()));
         BuzzTablePut(t_vm, "y", static_cast<float>(nest_pair_one[1].Position.GetY()));
         BuzzTablePut(t_vm, "radius", static_cast<float>(nest_pair_one[1].Radius));
         /* Done with the nest table */
      BuzzTableCloseNested(t_vm);
      /* Done with the tasks table */
      BuzzTableClose(t_vm);
   }
   
   const std::vector<STask>& nest_pair_one;
};
struct PutSimClock : public CBuzzLoopFunctions::COperation {
   /** The action happens here */
   PutSimClock() {}

   virtual void operator()(const std::string& str_robot_id, buzzvm_t t_vm) {
      BuzzPut(t_vm, "dt", (float) CPhysicsEngine::GetSimulationClockTick());  
   }
};


/****************************************/
/****************************************/

int nRobots;
void CCBAA::Init(TConfigurationNode& t_tree) {
   /* Parse XML tree */
   GetNodeAttribute(t_tree, "outfile", m_strOutFile);
   GetNodeAttribute(t_tree, "robots", nRobots);
   int nTasks;
   GetNodeAttribute(t_tree, "tasks", nTasks);
   int nMsgSize;
   GetNodeAttribute(t_tree, "msg_size", nMsgSize);
   Real commsRange;
   GetNodeAttribute(t_tree, "commsRange", commsRange);

   // Defined in cbaa.cpp now
   Real nest_x, nest_y;
   Real food_x, food_y;
   Real radius;
   GetNodeAttribute(t_tree, "nest_one_pos_x", nest_x);
   GetNodeAttribute(t_tree, "nest_one_pos_y", nest_y);
   GetNodeAttribute(t_tree, "food_one_pos_x", food_x);
   GetNodeAttribute(t_tree, "food_one_pos_y", food_y);
   GetNodeAttribute(t_tree, "radius", radius);


   /* Create a new RNG */
   m_pcRNG = CRandom::CreateRNG("argos");

   /* Place the robots randomly in the center of the environment */
   CRange<Real> cRobotRange(-2,2);
   for(size_t i = 0; i < nRobots; ++i) {

      /* Pick an orientation at random */
      CQuaternion cOrient;
      cOrient.FromAngleAxis(
         m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
         CVector3::Z);

      /* Prepare the robot id */
      std::ostringstream ossId;
      ossId << "fb" << i;

      /* Place the robot in the arena */
      CVector3 cPos;
      bool bDone = false;
      do {
         /* Pick a position uniformly at random */
         cPos.Set(m_pcRNG->Uniform(cRobotRange),
                  m_pcRNG->Uniform(cRobotRange),
                  0.0);

         /* Create a foot-bot with an initial configuration */
         CFootBotEntity* pcFB = new CFootBotEntity(
            ossId.str(), // robot id
            "fbc",       // controller id as defined in .argos file
            cPos,        // position
            cOrient,     // orientation
            commsRange,        // communication range in meters
            nMsgSize);   // max message size in bytes

         /* Add the foot-bot to the simulation */
         AddEntity(*pcFB);

         /* Check for collisions */
         if(!pcFB->GetEmbodiedEntity().IsCollidingWithSomething())
            break;
         RemoveEntity(*pcFB);
      } while(1);
   }

   /* Call parent Init() so Buzz can do some housekeeping */
   CBuzzLoopFunctions::Init(t_tree);


   /* Make the nests and food */
   // manually define the task nests position and food position vectors
   nest_pair_one.resize(2);
   // nest_pair_two.resize(2);


   CVector2 nest_one_pos;
   nest_one_pos.Set(nest_x, nest_y);
   CVector2 food_one_pos;
   food_one_pos.Set(food_x, food_y);
   // nest one
   nest_pair_one[0].Position = nest_one_pos;
   nest_pair_one[0].Radius = radius;
   // food one
   nest_pair_one[1].Position = food_one_pos;
   nest_pair_one[1].Radius = radius;
   
   // CVector2 cPos;
   // nest two
   // cPos.Set(0.0, -8.0);
   // nest_pair_two[0].Position = food_two_pos;
   // cPos.Set(0.0, 8.0);
   // nest_pair_two[1].Position = cPos;

   /* Finalize initialization */
   Reset();
}

/****************************************/
/****************************************/

void CCBAA::Reset() {
   // /* Tell all the robots about the tasks */
   BuzzForeachVM(PutTables(nest_pair_one));
   BuzzForeachVM(PutSimClock());

   /* Reset the output file */
   m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);

   // define the header of the output file
   m_cOutFile << "ts, robot, x, y, approx_food_x, approx_food_y, food_confidence, approx_home_x, approx_home_y, home_confidence, num_food_collected";
   
   // for(size_t j = 0; j < m_vecTasks.size(); ++j)
   //    m_cOutFile << "\t" << "x_i" << j;
   // for(size_t j = 0; j < m_vecTasks.size(); ++j)
   //    m_cOutFile << "\t" << "y_i" << j;
   m_cOutFile << std::endl;
}

/****************************************/
/****************************************/

void CCBAA::Destroy() {
   m_cOutFile.close();
}

/****************************************/
/****************************************/

// static Real TASK_RADIUS = 1;
// static Real TASK_RADIUS_2 = TASK_RADIUS * TASK_RADIUS;
CColor CCBAA::GetFloorColor(const CVector2& c_position_on_plane) {
   // for(UInt32 i = 0; i < m_vecTasks.size(); ++i) {
      //    if((c_position_on_plane - m_vecTasks[i].Position).SquareLength() < TASK_RADIUS_2) {
         //       return CColor::BLACK;
         //    }
         // }
   Real TASK_RADIUS_2 = nest_pair_one[0].Radius * nest_pair_one[0].Radius;

   // define the nest and food position vector
   if((c_position_on_plane - nest_pair_one[0].Position).SquareLength() < TASK_RADIUS_2) {
      return CColor::PURPLE;
   }

   if((c_position_on_plane - nest_pair_one[1].Position).SquareLength() < TASK_RADIUS_2) {
      return CColor::GREEN;
   }
  
   // if((c_position_on_plane - nest_pair_two[0].Position).SquareLength() < TASK_RADIUS_2) {
   //    return CColor::GREEN;
   // }

   // if((c_position_on_plane - nest_pair_two[1].Position).SquareLength() < TASK_RADIUS_2) {
   //    return CColor::BLUE;
   // }

   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CCBAA::PostStep() {
   // /* Get robot data */
   GetRobotData cGetRobotData(nRobots);
   BuzzForeachVM(cGetRobotData);

   /* Write it to the output file */
   /* Go through each robot */
   for(auto i = cGetRobotData.m_mapAssignments.begin();
       i != cGetRobotData.m_mapAssignments.end();
       ++i) {
      /* Time step */
      m_cOutFile << GetSpace().GetSimulationClock();
      /* Robot id */
      m_cOutFile << ", " << i->first;

      /* Robot position */
      m_cOutFile << ", " << i->second.position.GetX() << ", " << i->second.position.GetY();

      /* Robot state */
      m_cOutFile << ", " << i->second.state.approx_food.GetX() << ", " << i->second.state.approx_food.GetY() << ", " << i->second.state.food_confidence;
      m_cOutFile << ", " << i->second.state.approx_home.GetX() << ", " << i->second.state.approx_home.GetY() << ", " << i->second.state.home_confidence;
      
      /* Food recovered */
      m_cOutFile << ", " << i->second.food_recovered;

      m_cOutFile << std::endl;
   }
}

/****************************************/
/****************************************/

bool CCBAA::IsExperimentFinished() {
   /* Feel free to try out custom ending conditions */
   return false;
}

/****************************************/
/****************************************/

int CCBAA::GetNumRobots() const {
   return m_mapBuzzVMs.size();
}

/****************************************/
/****************************************/

void CCBAA::BuzzBytecodeUpdated() {
   /* Convey the stimuli to every robot */
   // BuzzForeachVM(PutTables(m_vecTasks));
   BuzzForeachVM(PutTables(nest_pair_one));
   BuzzForeachVM(PutSimClock());

}
/****************************************/
/****************************************/

// std::vector<STask> CCBAA::GetTasks() const {
//    return m_vecTasks;
// }

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CCBAA, "socialodometry");
