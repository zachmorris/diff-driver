
namespace Robot_States
{
  enum Robot_State
  {
     GO_TO_GOAL,
     AVOID_OBSTACLE,
     STOP
  };
}
typedef Robot_States::Robot_State Robot_State;


struct Robot_Pose
{
    double x;
    double y;
    double yaw;
};
