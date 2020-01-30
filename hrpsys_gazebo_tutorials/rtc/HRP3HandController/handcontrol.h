/* 
      largely modified by Naoki-Hiraoka

      from
	File	: handcontrol.h
	Date	: Fri Apr 28 12:36:15 JST 2006
	Author	: Kei Okada <k-okada@jsk.t.u-tokyo.ac.jp>	
*/

#ifndef JSK_HANDCONTROL_H
#define JSK_HANDCONTROL_H

#include "interpolator.h"
#include <string>
#include <sstream>
#include <iostream>

struct OpenHRP2_RobotState;
typedef OpenHRP2_RobotState robot_state;
typedef OpenHRP2_RobotState motor_command;
#include "HRP3HandControllerService.hh"
typedef OpenHRP::HRP3HandControllerService::dSequence dsequence;
typedef OpenHRP::HRP3HandControllerService::dSequence_out dsequence_out;

#include "unistd.h"

#define MOTOR_NO		12
#define DOF	MOTOR_NO
#define Potentio_num MOTOR_NO

class handcontrolPlugin_impl {
public:
  handcontrolPlugin_impl();
  bool setup(robot_state *rs,motor_command *mc);
  void control(robot_state *rs,motor_command *mc);
  bool cleanup(robot_state *rs,motor_command *mc);
  ;;
  void m_joint_angles(istringstream& strm);
  void m_wait_interpolation(istringstream& strm);
  void m_set_interpolation_method(istringstream& strm);
  void m_pgain_vector(istringstream& strm);
  void m_dgain_vector(istringstream& strm);
  void m_flimit_vector(istringstream& strm);
  void m_fcthreshold_vector(istringstream& strm);
  void m_controlmode_vector(istringstream& strm);
  void m_flimit_gain_vector(istringstream& strm);
  void set_body_servoOff (const bool _is_servoOff)
  {
    is_body_servoOff = _is_servoOff;
  };
  void set_debug_level(const size_t _debuglevel)
  {
    debug_level = _debuglevel;
  };

  void getRobotState(dsequence_out reference,
		     dsequence_out potentio,
		     dsequence_out error,
		     dsequence_out sense,
		     dsequence_out houtput,
		     dsequence_out motorthermo,
		     dsequence_out pgain,
		     dsequence_out dgain,
		     dsequence_out flimit,
		     dsequence_out controlmode,
		     dsequence_out curlim,
		     dsequence_out fcthreshold,
		     dsequence_out tactiles,
		     dsequence_out thermo
                     );
  void handServoOn();
  void handServoOff();
  void handJointCalib();
  void handReconnect();
  void waitInterpolation();
  bool setJointAngles(const double* jvs, const double tm);
  void set_dt (const double _dt) { m_dt = _dt; };

private:
  interpolator *angle;
  double output[MOTOR_NO], m_dt;
  bool is_body_servoOff;
  size_t debug_level;
};

#ifndef HRPNO
#define HRPNO 16
#endif

#define PGAIN_NO	MOTOR_NO
#define DGAIN_NO	MOTOR_NO

#define F_LIMIT_NO	MOTOR_NO
#define CURLIM_NO       MOTOR_NO
#define	CONTROL_MODE_NO	MOTOR_NO
#define FC_THRESHOLD_NO MOTOR_NO

#define RHAND_T_1P   1
#define RHAND_F1_1P  3
#define RHAND_F1_2R  4
#define RHAND_T_1Y   0
#define RHAND_F1_1R  2
#define RHAND_F2_2R  5
#define LHAND_T_1P   7
#define LHAND_F1_1P  9
#define LHAND_F1_2R 10
#define LHAND_T_1Y   6
#define LHAND_F1_1R  8
#define LHAND_F2_2R 11

#include <math.h>
#if HRPNO==16
#warning Compiling handcontrol for hrp2016c
#elif HRPNO==17
#warning Compiling handcontrol for hrp2017c
#endif

struct servoInfo {
  double neutralAngle;
  double neutralPulse;
  double neutralPotentio;
  double pulsePerAngle;
  double pulsePerPotentio;
  double pulsePerError;
  double pgain, dgain;
  double fc_cycle, fc_threshold;
  double f_limit;
  double curlim;
  int	controlmode;
  unsigned usepgain:1, usedgain:1;
  unsigned usefccycle:1, usefcthreshold:1, useflimit:1;
  unsigned usecurlim:1;
  unsigned usecontrolmode:1;
};
void setServoInfo(void);/**/

void GetRobotState(int wait_msec);/**/
void SetAngleVector(double v[DOF]);/**/
double* ControlModeVector(double modes[DOF]);/**/
double* SetPGainVector(double v[DOF]);/**/
double* SetDGainVector(double v[DOF]);/**/
double* SetFLimitVector(double v[DOF]);/**/
double* SetFCThresholdVector(double v[DOF]);/**/
double* SetControlModeVector(double modes[DOF], double output[DOF]);/**/
double* SetControlModeVector(double mode, double output[DOF]);/**/

typedef struct {
  double	Reference[MOTOR_NO];
  double	Potentio[MOTOR_NO];
  double	Error[MOTOR_NO];
  double	Sense[MOTOR_NO];
  double	Tactiles[16];
  double	Thermo[16];
  double	houtput[MOTOR_NO];
  double	motorthermo[MOTOR_NO];
  double	Pgain[MOTOR_NO];/**/
  double	Dgain[MOTOR_NO];/**/
  double	Flimit[MOTOR_NO];/**/
  double	Controlmode[MOTOR_NO];/**/
  double	Curlim[MOTOR_NO];/**/
  double	Fcthreshold[MOTOR_NO];/**/
} UsbRobotState;

#endif /* JSK_HANDCONTROL_H */
