#ifndef FSA_PROCESS
#define FSA_PROCESS

#include "robot_process.h"

#include <droneMsgsROS/dronePositionRefCommandStamped.h>
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/droneSpeeds.h>
#include "droneMsgsROS/droneYawRefCommand.h"

#include <mutex>
#include <thread>

#include <chrono>
#include <unistd.h>

#include <fstream>

#include "xmlfilereader.h"
#include "cvg_string_conversions.h"

#include <ros/package.h>

// Color definitions for console output
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#define P1 0
#define P2 1

#define LEFT 0
#define RIGHT 1

#define kp 0
#define ki 1
#define kd 2



class FSA_process : public RobotProcess
{
public:
  //! Constructor. \details Same arguments as the ros::init function.
  FSA_process();

  //! Destructor.
  ~FSA_process();

  void ownSetUp();
  void ownStart();
  void ownStop();
  void ownRun();

private:
  //! Internal variables for Fibonacci Search Algorithm
  int method;
  //Default gains: -1.33, 0.0, -0.35
  double p1Range[2] = { 0.3, 5.0 }; //Allowed range for P1
  double p2Range[2] = { 0.0, 2.0 }; //Allowed range for P2
  double parInitial = 0.50; //Initial value of P2
  double accuracy = 0.05;
  int numIterations = 48; //Maximum number of iterations
  int Nc = 50; //Number of samples to collect performance index
  int NcMax = 60; //Number of samples of the trajectory primitive
  int initialDelay = 40;// Number of iterations delay for changing initial parameters

  int numRanges = 6; //Number of new ranges to calculate per bootstrap. NOTE: Rename this variable
  double Js[2]; //Performance index
  double newRange[2]; //Range for next iteration
  double previousRange[2]; //Range for previous iteration
  double params[2]; //Parameters P1 (at index 0) and P2 (at index 1)

  int iterationCounter = 0; //Counts how many iterations have passed
  int NcCounter = 0; //Number of samples made
  double J = 0; //Performance index. Keep adding
  int par = 0; //Parameter being evaluated
  bool running = false;
  bool firstIteration = false;
  int delayCounter = 0;
  int iterationCounterPerCycle = 0; //Counts iterations until change of parameter. NOTE: Rename this variable
  double initialDifference = 0;

  //trajectory primitive (normalized)
  double trajectoryPrimitive[60] = {0, 0.39347, 0.63212, 0.77687, 0.86466, 0.91792, 0.95021, 0.9698, 0.98168, 0.98889, 0.99326, 0.99591, 0.99752, 0.9985, 0.99909, 0.99945, 0.99966, 0.9998, 0.99988, 0.99993, 0.99995, 0.99997, 0.99998, 0.99999, 0.99999, 1, 0.60653, 0.36788, 0.22313, 0.13533, 0.082085, 0.049787, 0.030197, 0.018316, 0.011109, 0.0067379, 0.0040868, 0.0024787, 0.0015034, 0.00091188, 0.00055308, 0.00033546, 0.00020347, 0.00012341, 7.4852e-05, 4.54e-05, 2.7536e-05, 1.6702e-05, 1.013e-05, 6.1442e-06, 3.7266e-06, 2.2603e-06, 1.371e-06, 8.3153e-07, 5.0435e-07, 3.059e-07, 1.8554e-07, 1.1253e-07, 6.8256e-08, 4.1399e-08};
  double scale = 0.7; // Scale of the trajectory primitive
  double offset = 0.8; // Offset of the trajectory primitive (minimum altitude for performing the test)
  float currentX = 0;
  float currentY = 0;
  float currentZ = 0;
  float initialReference[3];
  float finalReference[3];

  float currentSpeed = 0;
  bool positionRecieved = false;

  //const int kp = 0;
  //const int ki = 1;
  //const int kd = 2;
  int gain1type = kp;
  int gain2type = kd;

private:
  //! Time variables
  std::chrono::steady_clock::time_point start_time;

private:
  //! Data recording
  std::string drone_pose_recording;
  std::string gains_recording;
  //std::string drone_pose_recording_path = "/home/antonio/Desktop/drone_pose_recording.txt";
  //std::string gains_recording_path = "/home/antonio/Desktop/gains_recording.txt";
  std::string drone_pose_recording_path = "/home/antonio/Desktop/data_recording";
  std::string gains_recording_path = "/home/antonio/Desktop/data_recording";
  std::string drone_pose_recording_filename = "drone_pose_recording";
  std::string gains_recording_filename = "gains_recording";
  int numberOfTests = 1;
  int testCounter = 1;

private:
  //! Ros Communication
  ros::NodeHandle node_handle;

  ros::Publisher gain_reconfigure_pub;
  ros::Publisher drone_position_ref_pub;

  ros::Publisher drone_yaw_ref_pub;

  ros::Subscriber estimated_pose_sub;
  ros::Subscriber estimated_speeds_sub;

  std::mutex mutex_position;
  std::mutex mutex_speed;

  std::string altitude_gain_reconfigure_topic_name = "altitude_gain_reconfigure";
  std::string position_gain_reconfigure_topic_name = "position_gain_reconfigure";
  std::string gain_reconfigure_topic_name = altitude_gain_reconfigure_topic_name;
  std::string drone_position_ref_topic_name = "dronePositionRefs";

  std::string drone_yaw_ref_pub_name = "droneControllerYawRefCommand";//TEMP?

  std::string estimated_pose_topic_name = "SOEstimatedPose";
  std::string estimated_speeds_topic_name ="SOEstimatedSpeeds";
  //Use this topics for real flight:
  //std::string estimated_pose_topic_name = "ArucoSlam_EstimatedPose";//"" //"/drone4/ArucoSlam_EstimatedPose" "/drone4/EstimatedPose_droneGMR_wrt_GFF"
  //std::string estimated_speeds_topic_name ="ArucoSlam_EstimatedSpeeds"; //"SOEstimatedSpeeds" //"/drone4/ArucoSlam_EstimatedSpeeds" "/drone4/EstimatedSpeed_droneGMR_wrt_GFF"

private:
  void initFibonacciVariables();

private:
  double FibonacciNumber(int n);
  void generateFibonacciRange(double i, double N_max, const double current_range[2], double new_range[2]);

private:
  void (FSA_process::*reconfigureGainsFunction)(float, float);
  void reconfigureAltitudeGains(float, float);
  void reconfigurePositionGains(float, float);
  void sendReference(float[3]);

private:
  void createFile(const std::string& path, const std::string& filename);
  void saveDataToFile(const std::string&, const std::string& filename, std::string&);

private:
  bool readConfigs(const std::string&);
  template<typename T> T readXMLParameter(XMLFileReader &xmlFileReader, const std::string& parameterName, T defaultValue);
  double readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const double& defaultValue);
  int readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const int& defaultValue);
  std::string readParameter(XMLFileReader& xmlFileReader, const std::string& parameterName, const std::string defaultValue);
  void removeSpacesTabsLinebreaks(std::string&);

private:
  double (FSA_process::*costFunction)();
  //double (*costFunction)();
  double altitudeCostFunction();
  double positionCostFunction();

public:
  //! Callbacks
  void estimatedPoseCallBack(const droneMsgsROS::dronePose&);
  void estimatedSpeedsCallBack(const droneMsgsROS::droneSpeeds&);

};
#endif
