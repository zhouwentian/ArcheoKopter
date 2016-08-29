#ifndef OROCOS_ISAE_LDC_COMP_HPP
#define OROCOS_ISAE_LDC_COMP_HPP

#include <ComponentBase.hpp>

#include <mavlink.h>
//#include <ORO_COMP_mavlink_bridge/mavlink_bridge.hpp>
#include <actuator_control_target.h>
#include <attitude.h>
#include <attitude_target.h>
#include <imu.h>
#include <battery.h>
#include <px4_aux_data.h>
//#include <px4_mode.h>
//#include "home/dmia/workspaces/workspaceQtCreator/ORO_COMP_Mav2-buildLocal/ORO_TYPE_common/px4_mode.h"
#include "../ORO_COMP_Mav2-buildLocal/ORO_TYPE_common/px4_mode.h"
#include <Full_GPS.h>

namespace ISAE{

class LDC : public ISAE::component_base
{

private:

    //variable de reception des ports
    float _alt_ground_laser;
    ISAE::attitude_euler _attitude;
    ISAE::battery _bat;
    ISAE::PX4_aux_data _px4_aux_data;
    ISAE::PX4_mode_flags _px4_mode = 0;
    mavlink_highres_imu_t _imu;
    ISAE::attitude_target _att_tgt_out;
    ISAE::attitude_target _att_tgt_feedback;
    STR_GPSFUSION  _GPI_tmp;
    mavlink_local_position_ned_t  _local_pos;

    // booleens d'affichage
    bool _b_imu_press;
    bool _b_upt_port;
    bool _b_elr_dspl;
    bool _b_drt_ctrl_dspl;
    bool _b_gpi_dspl;
    bool _b_laser_dspl;

    //variable pour loop
    float _pow =0;
    float _incre = 0.001;
    bool _b_simul_Outputs;

    //tableau conteant les valeurs des sorti
    float outputsValue[8] = {0,0,0,0,0,0,0,0};
    /* Information we want to grab from pixhawk */


//    std::string logHook() const;
//    std::string logHeader() const;

    // creation des port d'entrée et de sorti
    RTT::OutputPort<ISAE::actuator_control_target> _direct_control_output;
    RTT::OutputPort<ISAE::attitude_target> _attitude_control_output;

    RTT::InputPort<STR_GPSFUSION> _GPI_Control_input;
    RTT::InputPort<ISAE::attitude_target> _attitude_control_feedback_input;
    RTT::InputPort<ISAE::attitude_euler> _attitude_input;
    RTT::InputPort<mavlink_highres_imu_t> _imu_input;
    RTT::InputPort<ISAE::battery> _battery_input;
    RTT::InputPort<ISAE::PX4_mode_flags> _px4_mode_input;
    RTT::InputPort<ISAE::PX4_aux_data> _px4_aux_data_input;
    RTT::InputPort<mavlink_local_position_ned_t> _local_position_input;
    RTT::InputPort<float> _alt_input;

  public:
    LDC(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void resetDisplay();
    //port
    void updateDataPort();


    //fonction display
    void toggleDisplay(char i);
    void displayAttitudeEuler();
    void displayAttitudeControlFeedBack();
    void displayImuPressure();
    void displayGpi();
    void displayLaserData();
    void splitDisplay();

    // fonction send attitude target
    void sendAttitudeTarget(ISAE::attitude_target m_tmp );
    void sendAttitudeTarget(uint8_t type_mask, float roll, float pitch, float yaw, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust /*[-1,1]*/);
    void sendAttitudeTarget_attitude(float roll, float pitch, float yaw );
    void sendAttitudeTarget_roll_rate(  float body_roll_rate);
    void sendAttitudeTarget_pitch_rate(  float body_pitch_rate);
    void sendAttitudeTarget_yaw_rate(  float body_yaw_rate);
    void sendAttitudeTarget_thrust(  float thrust);

    //fonction de modification de valeur des sortie
    void setOutputs(float tab);
    void setOutput(int nbO, float val);
    void simulOutputs();
    void sendDirectControlMSG();
};
}/*end namespace ISAE*/
#endif /*OROCOS_ISAE_LDC_COMP_HPP*/
