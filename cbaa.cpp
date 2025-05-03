#include "cbaa.h"
#include "buzz/buzzvm.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <map>
#include <utility>

static CRange<Real> STIMULUS_RANGE(0.0, 5000.0);

/****************************************/
/****************************************/

static void BuzzAssertVector(buzzvm_t t_vm,
                             const std::string& str_var) {
   buzzobj_t v = BuzzGet(t_vm, str_var);
   if(!v) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\" has no variable called \"" << str_var << "\"");
   }
   if(!buzzobj_istable(v)) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s \"" << str_var << "\" variable should be a table");
   }
}

static bool BuzzFetchXij(buzzvm_t t_vm, int j) {
   buzzobj_t e = BuzzTableGet(t_vm, j);
   if(!e) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s xi[" << j << "] does not exist");
   }
   if(!buzzobj_isint(e)) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s xi[" << j << "] is not an integer");
   }
   int xij = buzzobj_getint(e);
   if(xij < 0 || xij > 1) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s xi[" << j << "] is not 0 or 1");
   }
   return xij;
}

static Real BuzzFetchYij(buzzvm_t t_vm, int j) {
   buzzobj_t e = BuzzTableGet(t_vm, j);
   if(!e) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s yi[" << j << "] does not exist");
   }
   if(!buzzobj_isfloat(e)) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s yi[" << j << "] is not a float");
   }
   float yij = buzzobj_getfloat(e);
   if(yij < 0.0) {
      THROW_ARGOSEXCEPTION("Robot \"fb" << t_vm->robot << "\"'s yi[" << j << "] cannot be negative");
   }
   return yij;
}

/**
 * Functor to get data from the robots
 */
struct GetRobotData : public CBuzzLoopFunctions::COperation {

   /** Constructor */
   GetRobotData(size_t n_tasks) : m_nTasks(n_tasks) {}

   /** The action happens here */
   virtual void operator()(const std::string& str_robot_id,
                           buzzvm_t t_vm) {
      /* Empty assignment to fill */
      SAssignment sAssignment(m_nTasks);
      /* Make sure 'xi' and 'yi' exist and are vectors */
      BuzzAssertVector(t_vm, "xi");
      BuzzAssertVector(t_vm, "yi");
      /* Extract data from xi */
      BuzzTableOpen(t_vm, "xi");
      for(size_t j = 0; j < m_nTasks; ++j) {
         sAssignment.xi[j] = BuzzFetchXij(t_vm, j);
      }
      BuzzTableClose(t_vm);
      /* Extract data from yi */
      BuzzTableOpen(t_vm, "yi");
      for(size_t j = 0; j < m_nTasks; ++j) {
         sAssignment.yi[j] = BuzzFetchYij(t_vm, j);
      }
      BuzzTableClose(t_vm);
      /* Save assignment data */
      m_mapAssignments.insert(
         std::make_pair(str_robot_id, sAssignment));
   }

   size_t m_nTasks;
   std::map<std::string, SAssignment> m_mapAssignments;
};

/****************************************/
/****************************************/

/**
 * Functor to put the task info in the Buzz VMs.
 */
struct PutTasks : public CBuzzLoopFunctions::COperation {

   /** Constructor */
   PutTasks(const std::vector<STask>& vec_tasks) : m_vecTasks(vec_tasks) {}
   
   /** The action happens here */
   virtual void operator()(const std::string& str_robot_id,
                           buzzvm_t t_vm) {
      /* Set the values of the table 'tasks' in the Buzz VM */
      BuzzTableOpen(t_vm, "tasks");
      for(int i = 0; i < m_vecTasks.size(); ++i) {
         /* Create a nested table for this task, indexed numerically */
         BuzzTableOpenNested(t_vm, i);
         /* Create a nested table 'position' */
         BuzzTableOpenNested(t_vm, "position");
         /* Put (x,y) in the position table */
         BuzzTablePut(t_vm, "x", static_cast<float>(m_vecTasks[i].Position.GetX()));
         BuzzTablePut(t_vm, "y", static_cast<float>(m_vecTasks[i].Position.GetY()));
         /* Done with the position table */
         BuzzTableCloseNested(t_vm); // "position"
         /* Put the reward in the current task table */
         BuzzTablePut(t_vm, "reward", static_cast<float>(m_vecTasks[i].Reward));
         /* Done with the current task table */
         BuzzTableCloseNested(t_vm); // i
      }
      /* Done with the tasks table */
      BuzzTableClose(t_vm);
   }

   /** Task info */
   const std::vector<STask>& m_vecTasks;
};

/****************************************/
/****************************************/

void CCBAA::Init(TConfigurationNode& t_tree) {
   /* Parse XML tree */
   GetNodeAttribute(t_tree, "outfile", m_strOutFile);
   int nRobots;
   GetNodeAttribute(t_tree, "robots", nRobots);
   int nTasks;
   GetNodeAttribute(t_tree, "tasks", nTasks);
   int nMsgSize;
   GetNodeAttribute(t_tree, "msg_size", nMsgSize);
   /* Create a new RNG */
   m_pcRNG = CRandom::CreateRNG("argos");
   /* Place the robots randomly in the center of the environment */
   CRange<Real> cRobotRange(-0.5,0.5);
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
            10.0,        // communication range in meters
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
   /* Generate the tasks */
   m_vecTasks.resize(nTasks);
   CRange<Real> cTaskRange(-5.0,5.0);
   for(int i = 0; i < m_vecTasks.size(); ++i) {
      /* Pick a random position outside of the robot init area */
      CVector2 cPos;
      do {
         cPos.Set(m_pcRNG->Uniform(cTaskRange),
                  m_pcRNG->Uniform(cTaskRange));
      } while(cRobotRange.WithinMinBoundIncludedMaxBoundIncluded(cPos.GetX()) &&
              cRobotRange.WithinMinBoundIncludedMaxBoundIncluded(cPos.GetY()));
      /* Distribute the tasks uniformly in x and y */
      m_vecTasks[i].Position = cPos;
      /* Pick the task rewards uniformly */
      m_vecTasks[i].Reward = m_pcRNG->Uniform(CRange(0.0, 1.0));
   }
   /* Finalize initialization */
   Reset();
}

/****************************************/
/****************************************/

void CCBAA::Reset() {
   /* Tell all the robots about the tasks */
   BuzzForeachVM(PutTasks(m_vecTasks));
   /* Reset the output file */
   m_cOutFile.open(m_strOutFile.c_str(),
                   std::ofstream::out | std::ofstream::trunc);
   m_cOutFile << "ts\trobot";
   for(size_t j = 0; j < m_vecTasks.size(); ++j)
      m_cOutFile << "\t" << "x_i" << j;
   for(size_t j = 0; j < m_vecTasks.size(); ++j)
      m_cOutFile << "\t" << "y_i" << j;
   m_cOutFile << std::endl;
}

/****************************************/
/****************************************/

void CCBAA::Destroy() {
   m_cOutFile.close();
}

/****************************************/
/****************************************/

static Real TASK_RADIUS = .1;
static Real TASK_RADIUS_2 = TASK_RADIUS * TASK_RADIUS;

CColor CCBAA::GetFloorColor(const CVector2& c_position_on_plane) {
   for(UInt32 i = 0; i < m_vecTasks.size(); ++i) {
      if((c_position_on_plane - m_vecTasks[i].Position).SquareLength() < TASK_RADIUS_2) {
         return CColor::BLACK;
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CCBAA::PostStep() {
   /* Get robot data */
   GetRobotData cGetRobotData(m_vecTasks.size());
   BuzzForeachVM(cGetRobotData);
   /* Write it to the output file */
   /* Go through each robot */
   for(auto i = cGetRobotData.m_mapAssignments.begin();
       i != cGetRobotData.m_mapAssignments.end();
       ++i) {
      /* Time step */
      m_cOutFile << GetSpace().GetSimulationClock();
      /* Robot id */
      m_cOutFile << "\t" << i->first;
      /* Values of xi */
      for(size_t j = 0; j < i->second.xi.size(); ++j)
         m_cOutFile << "\t" << i->second.xi[j];
      /* Values of yi */
      for(size_t j = 0; j < i->second.yi.size(); ++j)
         m_cOutFile << "\t" << i->second.yi[j];
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
   BuzzForeachVM(PutTasks(m_vecTasks));
}

/****************************************/
/****************************************/

std::vector<STask> CCBAA::GetTasks() const {
   return m_vecTasks;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CCBAA, "cbaa");
