/* 
      largely modified by Naoki-Hiraoka
      
      from
	File	: handcontrol.cpp
	Date	: Fri Apr 28 12:36:15 JST 2006
	Author	: Kei Okada <k-okada@jsk.t.u-tokyo.ac.jp>	
*/

#include <sys/time.h>
#include <stdio.h>
#include <string.h>

#include "handcontrol.h"

#include "iob2.h"
static iob hc_iob;

static UsbRobotState usb_state;
static double set_angle[DOF];
static double default_flimit[MOTOR_NO];
#define pGAIN  -5.0
#define dGAIN   0.0

handcontrolPlugin_impl::handcontrolPlugin_impl() {
}

bool handcontrolPlugin_impl::setup(robot_state *rs,motor_command *mc) {
  const char idstr[] = "$Id: handcontrol.cpp $";
  fprintf(stderr, "---- handcontrolPlugin::setup ----\n");  
  fprintf(stderr, "---- > %s, dt = %f[s]\n", idstr, m_dt);
  angle = new interpolator(DOF, m_dt, interpolator::HOFFARBIB);

  hc_iob.set_number_of_joints(DOF);
  hc_iob.set_number_of_force_sensors(0);
  hc_iob.set_number_of_gyro_sensors(0);
  hc_iob.set_number_of_accelerometers(0);

  hc_iob.set_signal_period(long(1e9 * m_dt));
  hc_iob.open_iob("HANDCONTROL");
  
  setServoInfo();
  for (int i = 0; i < DOF; i++) output[i] = 0;
  angle->set(output);

  ControlModeVector(usb_state.Controlmode);
  GetRobotState(0);
  for (int i = 0; i < DOF; i++) {
    output[i] = usb_state.Potentio[i];
  }
  if ( debug_level > 1 ) {
    for (int i = 0; i < DOF; i++) {
	fprintf(stderr, "%7.3f ", output[i]);
    }
    fprintf(stderr,"\n");
  }
  angle->set(output);
  is_body_servoOff = true;
  
  return true;
}

void handcontrolPlugin_impl::control(robot_state *rs,motor_command *mc) {
  struct timeval tv0,tv1;
  static int count = 0;
  if ( (count++) % 200 == 0 ) {
    if ( is_body_servoOff && usb_state.Controlmode[0] != 8) { // servo off
      double modes[DOF];
      for (int i = 0 ; i < MOTOR_NO; i++ ) modes[i] = 8 ;
      SetControlModeVector(modes, NULL);
      fprintf(stderr, ";; [handcontrol] move to zero output mode.. \n");
    }
  }

  if ( debug_level > 2 ) {
    fprintf(stderr, "get start ...\n");
    gettimeofday(&tv0,NULL);
  }
  GetRobotState(0);
  if ( debug_level > 2 ) {
    gettimeofday(&tv1,NULL);
    fprintf(stderr, "get done ... %7.3f [msec]\n", 
	    1000 * (double) tv1.tv_sec + (double) tv1.tv_usec / 1000
	    - (1000 * (double) tv0.tv_sec + (double) tv0.tv_usec / 1000)
	    );
  }

  if ( !angle->isEmpty () ) {
    angle->get(output);

    if ( debug_level > 2 ) {
      for (int i = 0; i < DOF; i++) {
	fprintf(stderr, "%7.3f ", output[i]);
      }
      fprintf(stderr,"\n");
    }
  }    
  // debug
  SetAngleVector(output);
}

bool handcontrolPlugin_impl::cleanup(robot_state *rs,motor_command *mc) {
  delete angle;
  double modes[DOF];
  for (int i = 0 ; i < MOTOR_NO; i++ ) modes[i] = 8 ;
  SetControlModeVector(modes, NULL);
  SetAngleVector(output);
  hc_iob.close_iob();
  usleep(1000);

  usleep(1000);
  return true;
}

void handcontrolPlugin_impl::m_joint_angles(istringstream& strm) {
  double tmp_angle[DOF], a = 0.0, tm = 1.0;
  
  for ( int i = 0; i < DOF; i++){
    if ( strm >> a ) {
      tmp_angle[i] = a;
    } else {
      fprintf(stderr, "Invalid length of angle vector. Too short!!\n");
      return;
    }
  }
  if (!(strm >> tm)){
    fprintf(stderr, "Invalid length of angle vector. Too short!!\n");
    return;
  }
  if (strm >> a){
    fprintf(stderr, "Invalid length of angle vector. Too long!!\n");
    return;
  }
  setJointAngles(tmp_angle, tm);
}

void handcontrolPlugin_impl::m_wait_interpolation(istringstream& strm) {
  waitInterpolation();
}

void handcontrolPlugin_impl::waitInterpolation()
{
  while ( !angle->isEmpty() ) {
    usleep(5000);
  }
  usleep(5000);
};

bool handcontrolPlugin_impl::setJointAngles(const double* jvs, const double tm)
{
  angle->setGoal(jvs,tm);

  if ( debug_level > 1 ) {
    cerr << "emptyp = " << angle->isEmpty() << endl;
    cout << "tmp_angle : ";   
    for (int i = 0; i < DOF; i++){
      cerr << jvs[i];
    }
    cerr << ", tm = " << tm << endl;
  }
};

void handcontrolPlugin_impl::m_set_interpolation_method(istringstream& strm) {
  string str;
  strm >> str;

  interpolator::interpolation_mode imode;
  if ( str == ":linear" ) {
    imode =  interpolator::LINEAR;
    cerr << "Using interpolator::LINEAR mode" << endl;
  }else{
    imode =  interpolator::HOFFARBIB;
    cerr << "Using interpolator::HOFFARBIB mode" << endl;    
  }
  while ( ! angle->isEmpty() ) {
    usleep(5000);
  }  
  angle = new interpolator(DOF,m_dt, imode);
  angle->set(output);
}

void handcontrolPlugin_impl::m_pgain_vector(istringstream& strm) {
  double tmp_gain[DOF], a = pGAIN;
  for ( int i = 0; i < DOF; i++){
    strm >> a;
    tmp_gain[i] = a;
  }
  SetPGainVector(tmp_gain);
  fprintf(stderr, "pgain : ");
  for ( int i = 0; i < DOF; i++) fprintf(stderr, "%7.3f", tmp_gain[i]);
  fprintf(stderr, "\n");
}

void handcontrolPlugin_impl::m_dgain_vector(istringstream& strm) {
  double tmp_gain[DOF], a = dGAIN;
  for ( int i = 0; i < DOF; i++){
    strm >> a;
    tmp_gain[i] = a;
  }
  SetDGainVector(tmp_gain);
  fprintf(stderr, "dgain : ");
  for ( int i = 0; i < DOF; i++) fprintf(stderr, "%7.3f\n", tmp_gain[i]);
  fprintf(stderr, "\n");
}

void handcontrolPlugin_impl::m_flimit_vector(istringstream& strm) {
  double tmp_limit[DOF], a = 20;
  for ( int i = 0; i < DOF; i++){
    strm >> a;
    tmp_limit[i] = a;
  }
  SetFLimitVector(tmp_limit);
  fprintf(stderr, "flimit : ");
  for ( int i = 0; i < DOF; i++) fprintf(stderr, "%7.3f\n", tmp_limit[i]);
  fprintf(stderr, "\n");
}

void handcontrolPlugin_impl::m_fcthreshold_vector(istringstream& strm) {
  double tmp_thre[DOF], a = 3;
  for ( int i = 0; i < DOF; i++){
    strm >> a;
    tmp_thre[i] = a;
  }
  SetFCThresholdVector(tmp_thre);
  fprintf(stderr, "fcthreshold : ");
  for ( int i = 0; i < DOF; i++) fprintf(stderr, "%7.3f\n", tmp_thre[i]);
  fprintf(stderr, "\n");
}

void handcontrolPlugin_impl::m_controlmode_vector(istringstream& strm) {
  double tmp_mode[DOF], a = 8;
  for ( int i = 0; i < DOF; i++){
    strm >> a;
    tmp_mode[i] = a;
  }
  SetControlModeVector(tmp_mode, output);
  fprintf(stderr, "controlmode : ");
  for ( int i = 0; i < DOF; i++) fprintf(stderr, "%7.3f\n", tmp_mode[i]);
  fprintf(stderr, "\n");
}

void handcontrolPlugin_impl::m_flimit_gain_vector(istringstream& strm) {
  double tmp_limit[DOF], a = 20;
  for ( int i = 0; i < DOF; i++){
    strm >> a;
    if ( !((1.0 >= a) && (a >= 0.0)) ) {
      fprintf(stderr, "hand flimit gain out of range\n");
      return;
    }
    tmp_limit[i] = a * default_flimit[i];
  }
  SetFLimitVector(tmp_limit);
  fprintf(stderr, "flimit : ");
  for ( int i = 0; i < DOF; i++) fprintf(stderr, "%7.3f\n", tmp_limit[i]);
  fprintf(stderr, "\n");
}

void handcontrolPlugin_impl::getRobotState(dsequence_out reference,
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
					   ) {
  reference = new dsequence;
  potentio = new dsequence;
  error = new dsequence;
  sense = new dsequence;
  houtput = new dsequence;
  motorthermo = new dsequence;
  pgain = new dsequence;
  dgain = new dsequence;     
  flimit = new dsequence;
  controlmode = new dsequence;
  curlim = new dsequence;
  fcthreshold = new dsequence;
  tactiles = new dsequence;
  thermo = new dsequence;
  
  reference->length(MOTOR_NO);
  potentio->length(MOTOR_NO);
  error->length(MOTOR_NO);
  sense->length(MOTOR_NO);
  houtput->length(MOTOR_NO);
  motorthermo->length(MOTOR_NO);
  pgain->length(MOTOR_NO);
  dgain->length(MOTOR_NO);
  flimit->length(MOTOR_NO);
  controlmode->length(MOTOR_NO);
  curlim->length(MOTOR_NO);
  fcthreshold->length(MOTOR_NO);
  tactiles->length(32);
  thermo->length(16);
  for (int i=0; i < 32; i++) tactiles->get_buffer()[i] = 0;
  for (int i=0; i < 16; i++) thermo->get_buffer()[i] = 0;
  //
  for (int i=0; i < MOTOR_NO; i++) reference->get_buffer()[i] = usb_state.Reference[i];
  for (int i=0; i < MOTOR_NO; i++)  potentio->get_buffer()[i] = usb_state.Potentio[i];
  for (int i=0; i < MOTOR_NO; i++) error->get_buffer()[i] = usb_state.Error[i];
  for (int i=0; i < MOTOR_NO; i++) sense->get_buffer()[i] = usb_state.Sense[i];
  for (int i=0; i < MOTOR_NO; i++) houtput->get_buffer()[i] = usb_state.houtput[i];
  for (int i=0; i < MOTOR_NO; i++) motorthermo->get_buffer()[i] = usb_state.motorthermo[i];
  for (int i=0; i < MOTOR_NO; i++) pgain->get_buffer()[i] = usb_state.Pgain[i];
  for (int i=0; i < MOTOR_NO; i++) dgain->get_buffer()[i] = usb_state.Dgain[i];
  for (int i=0; i < MOTOR_NO; i++) flimit->get_buffer()[i] = usb_state.Flimit[i];
  for (int i=0; i < MOTOR_NO; i++) controlmode->get_buffer()[i] = usb_state.Controlmode[i];
  for (int i=0; i < MOTOR_NO; i++) curlim->get_buffer()[i] = usb_state.Curlim[i];
  for (int i=0; i < MOTOR_NO; i++) fcthreshold->get_buffer()[i] = usb_state.Fcthreshold[i];
  for (int i=0; i < 16; i++) tactiles->get_buffer()[i] = usb_state.Tactiles[i];
  for (int i=0; i < 16; i++) thermo->get_buffer()[i] = usb_state.Thermo[i];
  return;
}

void handcontrolPlugin_impl::handServoOn()
{
  SetControlModeVector(8.0, output);
  usleep(static_cast<int>(5000));
  setJointAngles(usb_state.Potentio, 0.5);
  usleep(static_cast<int>(5000));
  waitInterpolation();
  SetControlModeVector(11.0, output);
  usleep(static_cast<int>(1000*1000));
};

void handcontrolPlugin_impl::handServoOff()
{
  SetControlModeVector(8.0, output);
  usleep(static_cast<int>(5000));
};

void handcontrolPlugin_impl::handJointCalib()
{
  double calib_pose1[DOF] = {30, 122, 0, 90, 0, -155,
                             30, 122, 0, 90, 0, -155};
  double calib_pose2[DOF] = {107, 122, 107, 90, -155, -155,
                             107, 122, 107, 90, -155, -155};
  double calib_pose3[DOF] = {0, 0, -95, -95, 40, 40,
                             0, 0, -95, -95, 40, 40};
  double calib_pose4[DOF] = {-92, -95, 0, 0, 0, 0,
                             -92, -95, 0, 0, 0, 0};
  waitInterpolation();
  for (size_t i = 0; i < DOF; i++) calib_pose1[i] = calib_pose1[i] + usb_state.Potentio[i];
  //setJointAngles(calib_pose1, 4.0);
  setJointAngles(calib_pose1, 0.5);
  waitInterpolation();
  for (size_t i = 0; i < DOF; i++) calib_pose2[i] = calib_pose2[i] + usb_state.Potentio[i];
  //setJointAngles(calib_pose2, 4.0);
  setJointAngles(calib_pose2, 0.5);
  waitInterpolation();
  for (size_t i = 0; i < DOF; i++) calib_pose3[i] = calib_pose3[i] + usb_state.Potentio[i];
  //setJointAngles(calib_pose3, 4.0);
  setJointAngles(calib_pose3, 0.5);
  waitInterpolation();
  for (size_t i = 0; i < DOF; i++) calib_pose4[i] = calib_pose4[i] + usb_state.Potentio[i];
  //setJointAngles(calib_pose4, 4.0);
  setJointAngles(calib_pose4, 0.5);
  waitInterpolation();
  SetControlModeVector(4, output);
  usleep(static_cast<int>(1000*1000));
  SetControlModeVector(11, output);
  usleep(static_cast<int>(1000*1000));
  waitInterpolation();
  double tmpoutput[DOF];
  for (int i = 0; i < DOF; i++) {
    tmpoutput[i] = usb_state.Potentio[i];
  }
  angle->set(tmpoutput);
};

#include <signal.h>
void handcontrolPlugin_impl::handReconnect()
{
  fprintf(stderr, "[hc] Reconnect called!!\n");
}

/*
 * END copy from ROBOTusbconf_H (hrp2handusbconf.h)
 */

static struct servoInfo Servo[MOTOR_NO];
void setServoInfo(void)
{
  int i;
  for (i = 0 ; i < MOTOR_NO; i++ ) {
    Servo[i].neutralAngle = 0;
    Servo[i].neutralPulse = 0;
    Servo[i].neutralPotentio = 0;
    Servo[i].pulsePerPotentio = 1.0;
    Servo[i].pulsePerError = 1.0;
    Servo[i].pgain = -5.0;
    Servo[i].dgain = 0.0;
    Servo[i].fc_cycle = 1.0;
    Servo[i].fc_threshold = 3;
    Servo[i].f_limit = 20;
    Servo[i].curlim = 1.0; /* [A] */
    Servo[i].controlmode = 8; // zeroout
    Servo[i].usepgain = Servo[i].usedgain = 1;
    Servo[i].usefccycle = Servo[i].usefcthreshold = Servo[i].useflimit = 1;
    Servo[i].usecurlim = 1;
    Servo[i].usecontrolmode = 1;

    hc_iob.write_pgain(i,5.0);
    hc_iob.write_servo(i,0);
    hc_iob.write_power_command(i,0);
  }

#if HRPNO==16
#warning Compiling handcontrol for hrp2016c
  default_flimit[RHAND_T_1Y ] = 10.0; // 0
  default_flimit[RHAND_T_1P ] = 20.0; // 1
  default_flimit[RHAND_F1_1R] = 10.0; // 2
  default_flimit[RHAND_F1_1P] = 15.0; // 3
  default_flimit[RHAND_F1_2R] =  7.5; // 4
  default_flimit[RHAND_F2_2R] = 20.0*0.95; // 5
  default_flimit[LHAND_T_1Y ] = 10.0; // 0
  default_flimit[LHAND_T_1P ] = 20.0; // 1
  default_flimit[LHAND_F1_1R] = 10.0; // 2
  default_flimit[LHAND_F1_1P] = 15.0; // 3
  default_flimit[LHAND_F1_2R] =  7.5; // 4
  default_flimit[LHAND_F2_2R] = 15.0*0.5; // 5
#elif HRPNO==17
#warning Compiling handcontrol for hrp2017c
  default_flimit[RHAND_T_1Y ] = 10.0; // 0
  default_flimit[RHAND_T_1P ] = 20.0; // 1
  default_flimit[RHAND_F1_1R] = 10.0; // 2
  default_flimit[RHAND_F1_1P] = 15.0; // 3
  default_flimit[RHAND_F1_2R] = 10.0; // 4
  default_flimit[RHAND_F2_2R] = 20.0; // 5
  default_flimit[LHAND_T_1Y ] = 10.0; // 0 
  default_flimit[LHAND_T_1P ] = 20.0; // 1
  default_flimit[LHAND_F1_1R] =  7.5; // 2
  default_flimit[LHAND_F1_1P] = 15.0; // 3
  default_flimit[LHAND_F1_2R] =  7.5; // 4
  default_flimit[LHAND_F2_2R] = 15.0; // 5
#else
#error unsupported devices
#endif
  for (int ii = 0; ii < MOTOR_NO; ii++)
    Servo[ii].f_limit = default_flimit[ii];
}


double* PGainVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++)
    if (Servo[i].usepgain) v[i] = Servo[i].pgain;
    else v[i]=0.0;
  return v;
}
double* DGainVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++)
    if (Servo[i].usedgain) v[i] = Servo[i].dgain;
    else v[i]=0.0;
  return v;
}
double* FCThresholdVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++)
    if (Servo[i].usefcthreshold) v[i]=Servo[i].fc_threshold;
    else v[i]=0.0;
  return v;
}
double* FLimitVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++)
    if (Servo[i].useflimit) v[i]=Servo[i].f_limit;
    else v[i]=0.0;
  return v;
}
double* CurLimVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++)
    if (Servo[i].usecurlim) v[i]=Servo[i].curlim;
    else v[i]=0.0;
  return v;
}
 double* ControlModeVector(double modes[DOF]) {
  int i;
  if (modes==NULL) return modes;
  for (i=0; i<MOTOR_NO; i++) modes[i] = Servo[i].controlmode;
  return modes;
}
double* SetPGainVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++) {
    Servo[i].pgain = v[i];
    hc_iob.write_pgain(i,-v[i]);
  }
  return v;
}
double* SetDGainVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++) {
    Servo[i].dgain = v[i];
    hc_iob.write_dgain(i,-v[i]);
  }
  return v;
}
double* SetFLimitVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++) Servo[i].f_limit = v[i];
  return v;
}
double* SetFCThresholdVector(double v[DOF]) {
  int i;
  if (v==NULL) return v;
  for (i=0; i<MOTOR_NO; i++) Servo[i].fc_threshold = v[i];
  return v;
}
double* SetControlModeVector(double modes[DOF], double output[DOF]) {
  int i;
  
  if (modes==NULL) return modes;

  for (i=0; i<MOTOR_NO; i++) {
    Servo[i].controlmode = (int)modes[i];
    if(Servo[i].controlmode == 11){
      hc_iob.write_power_command(i,1);
      hc_iob.write_servo(i,1);
    }
    if(Servo[i].controlmode == 8){
      hc_iob.write_power_command(i,0);
      hc_iob.write_servo(i,0);
    }
    //controlmode 4 is zerosetmode
    if(Servo[i].controlmode == 4){
      output[i] = 0;
    }
  }
  return modes;
}
double* SetControlModeVector(double mode, double output[DOF]) {
  double tmp_modes[DOF];
  for (size_t i = 0; i < DOF; i++) {
    tmp_modes[i] = mode;
  }
  SetControlModeVector(tmp_modes, output);
  return NULL;
}
void Set_SendData_to_RobotState(UsbRobotState *usb_state)
{
  double v[MOTOR_NO];
  int i;
  PGainVector(v);
  if(usb_state->Pgain){
    for(i=0; i<MOTOR_NO; i++){
      usb_state->Pgain[i] = v[i];
    }}
  DGainVector(v);
  if(usb_state->Dgain){
    for(i=0; i<MOTOR_NO; i++){
      usb_state->Dgain[i] = v[i];
    }}
  double ForceRef[MOTOR_NO];
  FCThresholdVector(v);
  if(usb_state->Fcthreshold){
    for(i=0; i<MOTOR_NO; i++){
      usb_state->Fcthreshold[i] = ForceRef[i];
    }}
  FLimitVector(v);
  if(usb_state->Flimit){
    for(i=0; i<MOTOR_NO; i++){
      usb_state->Flimit[i] = v[i];
    }}
  CurLimVector(v);
  if(usb_state->Curlim){
    for(i=0; i<MOTOR_NO; i++){
      usb_state->Curlim[i] = v[i];
    }}
  ControlModeVector(v);
  if(usb_state->Controlmode){
    for(i=0; i<MOTOR_NO; i++){
      usb_state->Controlmode[i] = v[i];
    }}
}

void Data_to_UsbRobotState(UsbRobotState *usb_state)
{
  int i, index;
  
  for (i = 0; i < MOTOR_NO; i++)
    {
      usb_state->Reference[i] = set_angle[i];
    }

  for (i = 0; i < MOTOR_NO; i++)
    {
      double ret;
      hc_iob.read_actual_angle(i,&ret);
      usb_state->Potentio[i] = ret * 180 / 3.14 ;
    }

  for (i = 0; i < MOTOR_NO; i++)
    {
      usb_state->Error[i] = usb_state->Reference[i]-usb_state->Potentio[i];
    }

  for (i = 0; i < MOTOR_NO; i++)
    {
      usb_state->Sense[i] = 0.0;
    }

  for (i = 0; i < 16; i++)
    {
      usb_state->Thermo[i] = 0.0;
    }

  for (i = 0; i < MOTOR_NO; i++)
    {
      usb_state->motorthermo[i] = 0.0;
    }

  for (i = 0; i < MOTOR_NO; i++)
    {
      usb_state->houtput[i] = 0.0;
    }

  for (i = 0; i < 16; i++)
    {
      usb_state->Tactiles[i] = 0.0;
    }
}

void GetRobotState(int wait_msec)
{
  Data_to_UsbRobotState(&usb_state);
  Set_SendData_to_RobotState(&usb_state);
  return;
}

void SetAngleVector(double v[DOF]) 
{
  int i;
  for (i = 0; i < MOTOR_NO; i++){
    set_angle[i] = v[i];
  }
  double vrad[DOF];
  for (int i=0;i<DOF;i++){
    vrad[i]=v[i]*3.14/180;
  }
  hc_iob.write_command_angles(vrad);
  return;
}
