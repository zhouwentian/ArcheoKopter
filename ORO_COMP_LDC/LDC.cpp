#include "LDC.hpp"
#include <rtt/Component.hpp>

#include <iostream>

const std::string ISAE::component_base::name = "@Pkgname";
const std::string ISAE::component_base::version = "0.1";
const std::string ISAE::component_base::doc = "XXX To be documented\n";

namespace ISAE {

LDC::LDC(std::string const& name) :
    ISAE::component_base(name)
{
    //set the name of the port in the deployer
    this->ports()->addPort("_direct_control_output",_direct_control_output).doc("send direct control motor");
    this->ports()->addPort("_attitude_control_output",_attitude_control_output).doc("send attitude target");
    this->ports()->addPort("_attitude_control_feedback_input",_attitude_control_feedback_input).doc("reception of control feef back");
    this->ports()->addPort("_attitude_input",_attitude_input).doc("reception attitude");
    this->ports()->addPort("_imu_input",_imu_input).doc("reception imut");
    this->ports()->addPort("_battery_input",_battery_input).doc("reception battery");
    this->ports()->addPort("_px4_mode_input",_px4_mode_input).doc("reception px4 mod");
    this->ports()->addPort("_GPI_Control_input",_GPI_Control_input).doc("reception gps pos");
    this->ports()->addPort("_px4_aux_data_input",_px4_aux_data_input).doc("reception px4 data");
    this->ports()->addPort("_local_position_input",_local_position_input).doc("reception local position (alt)");
    this->ports()->addPort("_alt_input",_alt_input).doc("reception alt/sol laser");


    // to access the variable on the deployer:
    addProperty("eulerDispl", _b_elr_dspl).
            doc("boolean display attitude euler");
    addProperty("directCTRLDispl",_b_drt_ctrl_dspl).
            doc("boolean display control feedback");
    addProperty("updatePort", _b_upt_port).
            doc("boolean update port");
    addProperty("gpiDispl", _b_gpi_dspl).
            doc("boolean gps display");
    addProperty("pressDispl", _b_imu_press).
            doc("boolean imu pression display");
    addProperty("laser", _b_laser_dspl).
            doc("boolean laser data display");
    addProperty("looper", _b_simul_Outputs).
            doc("boolean loop");
    addProperty("pow", _pow).
            doc("powermotor loop value");
    addProperty("incre", _incre).
            doc("increment loop value");

    // add fonction access to the deployer
    addOperation("tog", &LDC::toggleDisplay, this).
    doc("reverse the display chosen i:imu  e:euler  c: controle feed back  g: gpi  l:laser");
    addOperation("resDspl", &LDC::resetDisplay, this).
    doc("reset all display to none");

}

bool LDC::configureHook() {
  bool res = ISAE::component_base::configureHook();  
  _b_upt_port = true;
  _b_simul_Outputs=false;
  _b_elr_dspl = false;
  if (!res) return res;

  std::cout << "LDC configured !" <<std::endl;
  return true;
}

bool LDC::startHook() {
  bool res = ISAE::component_base::startHook();
  //std::cout << this->setPeriod(2)<< std::endl;
  //std::cout << this->getPeriod()<< std::endl;

  if (!res) return res;

  std::cout << "LDC started !" <<std::endl;
  return true;
}



void LDC::updateHook() {
  new_period();



  //std::cout << "yoyo"<<std::endl;


  if (display_debug){
        std::cout << component_name() <<"-> updateHook()" << std::endl;
  }
    //std::cout << component_name() <<"-> updateHook()" << std::endl;
  if(_b_simul_Outputs){
    simulOutputs();
    //putVal(_power);
  }
  //data port getter
   updateDataPort();
  //ligne de separation
   splitDisplay();
  //display attitude euler
   displayAttitudeEuler();
   //display attitude control feedback
   displayAttitudeControlFeedBack();
   //display imu pressure
   displayImuPressure();
   //display gpi
   displayGpi();
   //display gpi
   displayLaserData();


  end_period();
}



//var the value of the 8 output, to 0 to 1
void LDC::simulOutputs(){

            if(_pow>=0.99 ){

                _incre=-0.001;
              }
            if(_pow<=0.02){

                  _incre=0.001;
              }

              _pow=_pow+_incre;


              setOutputs(_pow);

}
//set one of the output between 0-7
void LDC::setOutput(int nO, float val){

    outputsValue[nO] = val;

}


// set the 8 output to a value
void LDC::setOutputs(float val){


        for(int i = 0 ; i <= 7 ; i++){

             outputsValue[i] = val;

        }


}

// move off all display value
void LDC::resetDisplay(){

    _b_drt_ctrl_dspl=false;
    _b_elr_dspl=false;
    _b_gpi_dspl=false;
    _b_imu_press=false;
    _b_laser_dspl=false;

}

// active one display value with a specific char
void LDC::toggleDisplay(char i){

    switch (i) {
        case 'c':
        {
            _b_drt_ctrl_dspl=!_b_drt_ctrl_dspl;
        break;
         }
        case 'e':
        {
            _b_elr_dspl=!_b_elr_dspl;
        break;
         }
        case 'g':
        {
            _b_gpi_dspl=!_b_gpi_dspl;
        break;
         }
        case 'i':
        {
             _b_imu_press=!_b_imu_press;
        break;
         }
        case 'l':
        {
            _b_laser_dspl=!_b_laser_dspl;
        break;
         }
         default:

            break;
        };
    }





//just split the display when one of display value are activate
void LDC::splitDisplay(){
     if(_b_elr_dspl || _b_drt_ctrl_dspl || _b_gpi_dspl || _b_imu_press || _b_laser_dspl){
         std::cout << " "<<  std::endl;
         std::cout << "________________________________________________________ "<<  std::endl;
         std::cout << " "<<  std::endl;
     }
}

//function which display attitude euler send by mavlink in the port _attitude_input
void LDC::displayAttitudeEuler(){

     if(_b_elr_dspl){
                   //lign break
                   /*std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;
                   std::cout << " "<<  std::endl;*/
                   //lign break


                   std::cout << "pry"<<  std::endl;
                   std::cout << "   pitch : "<< ((_attitude.pitch)*180)/3.14<< std::endl;
                   std::cout << "   roll : "<< ((_attitude.roll)*180)/3.14<< std::endl;
                   std::cout << "   yaw : "<< ((_attitude.yaw)*180)/3.14<< std::endl;


                   std::cout << "pry_rate:"<<  std::endl;
                   std::cout << "   P_rate : "<< ((_attitude.pitchspeed)*180)/3.14<< std::endl;
                   std::cout << "   R_rate : "<< ((_attitude.rollspeed)*180)/3.14<< std::endl;
                   std::cout << "   Y_rate : "<< ((_attitude.yawspeed)*180)/3.14<< std::endl;

    }

}


//function which display attitude control feedback send by mavlink in the port _attitude_input
void LDC::displayAttitudeControlFeedBack(){

     if(_b_drt_ctrl_dspl){
                     //lign break
                    /* std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;*/
                     //lign break



                   std::cout << "pry_rate_radio co:"<<  std::endl;
                   std::cout << "   P_rate : "<< ((_att_tgt_feedback.body_pitch_rate)*180)/3.14<< std::endl;
                   std::cout << "   R_rate : "<< ((_att_tgt_feedback.body_roll_rate)*180)/3.14<< std::endl;
                   std::cout << "   Y_rate : "<< ((_att_tgt_feedback.body_yaw_rate)*180)/3.14<< std::endl;

    }

}

void LDC::displayImuPressure(){

     if(_b_imu_press){
                     //lign break
                     /*std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;
                     std::cout << " "<<  std::endl;*/
                     //lign break

                   std::cout << "static pressure: "<< _imu.abs_pressure<< std::endl;

                   std::cout << "Altitude  pressure: "<< _imu.pressure_alt<<  std::endl;

                   std::cout << "Vertical speed: " << _local_pos.vz<< std::endl;



    }
}
void LDC::displayGpi(){

    if(_b_gpi_dspl){
                    //lign break
                    /*std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;*/
                    //lign break

                  std::cout << "Latitude: " << _GPI_tmp.POSLLH_Data.latitude<< std::endl;
                  std::cout << "Longitude: " << _GPI_tmp.POSLLH_Data.longitude<< std::endl;



   }

}

void LDC::displayLaserData(){

    if(_b_laser_dspl){
                    //lign break
                    /*std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;
                    std::cout << " "<<  std::endl;*/
                    //lign break

                  std::cout << "D Laser: " << _alt_ground_laser<< std::endl;



   }

}


// function which take a table of float( the actuators) and send this table to mavlink
void LDC::sendDirectControlMSG(){
    ISAE::actuator_control_target m_tmp;

    for(int i = 0 ; i<8 ; i++){
        m_tmp.controls[i]=outputsValue[i];
        //std::cout << m_tmp.controls[i]<<  std::endl;
    }
     //std::cout << m_tmp.controls[0]<<   std::endl;
    _direct_control_output.write(m_tmp);
}


/* function which take the parameters of a attitude Target variable and send then to mavlink

type_mask: mappings: Si un des bits est à 1, l'entrée corespondantes est ignoré
    -->
        bit 1 : body roll rate
        bit 2 : body pitch rate
        bit 3 : body yaw rate
        bit 4->6 : reserved
        bit 7 : thrust
        bit 8 : attitude
        donc pour transmettre roll, pitch, yaw le mask doit être à 7
*/

            //call with attitude_target variabl<< m_tmp.controls[i]<< e
            void LDC::sendAttitudeTarget(ISAE::attitude_target m_tmp ){

                _attitude_control_output.write(m_tmp);

            }

            //call with the variable of an attitude target
            void LDC::sendAttitudeTarget(uint8_t type_mask, float roll, float pitch, float yaw, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust /*[-1,1]*/){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=type_mask;
                        m_tmp.roll=roll;
                        m_tmp.pitch=pitch;
                        m_tmp.yaw=yaw;
                        m_tmp.body_pitch_rate=body_pitch_rate;
                        m_tmp.body_roll_rate=body_roll_rate;
                        m_tmp.body_yaw_rate=body_yaw_rate;
                        m_tmp.thrust=thrust;


                _attitude_control_output.write(m_tmp);

            }
            // modify just the attitude
            void LDC::sendAttitudeTarget_attitude(float roll, float pitch, float yaw ){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0x47;
                        m_tmp.roll=roll;
                        m_tmp.pitch=pitch;
                        m_tmp.yaw=yaw;


                _attitude_control_output.write(m_tmp);

            }
            //modify just the roll rate
            void LDC::sendAttitudeTarget_roll_rate(  float body_roll_rate){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0xc6;
                        m_tmp.body_roll_rate=body_roll_rate;


                _attitude_control_output.write(m_tmp);

            }
            //modify just the pitch rate
            void LDC::sendAttitudeTarget_pitch_rate(  float body_pitch_rate){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0xc5;
                        m_tmp.body_pitch_rate=body_pitch_rate;


                _attitude_control_output.write(m_tmp);

            }
            //modify just the yaw rate
            void LDC::sendAttitudeTarget_yaw_rate(  float body_yaw_rate){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0xc3;
                        m_tmp.body_pitch_rate=body_yaw_rate;

                _attitude_control_output.write(m_tmp);

            }
            //modify just the thrust
            void LDC::sendAttitudeTarget_thrust(  float thrust){

                ISAE::attitude_target m_tmp ;
                        m_tmp.type_mask=0x87;
                        m_tmp.thrust=thrust;

                _attitude_control_output.write(m_tmp);

            }




//function which update the reception variables of the input ports
void LDC::updateDataPort() {
    //std::cout << "update"<<  std::endl;
        if(_b_upt_port){
                if (_imu_input.read(_imu) == RTT::NewData) {

                       // std::cout << "IMU UPDATE"<<  std::endl;
                }
                if (_GPI_Control_input.read(_GPI_tmp)== RTT::NewData) {

                       // std::cout << "GPI UPDATE"<<  std::endl;
                }
                if (_attitude_control_feedback_input.read(_att_tgt_feedback)== RTT::NewData) {

                        //std::cout << "ATTITUDE CONTROL FEEDBACK UPDATE"<<  std::endl;
                }
                if (_attitude_input.read(_attitude)== RTT::NewData) {

                        //std::cout << "ATTITUDE INPUT UPDATE"<<  std::endl;
                }
                if (_battery_input.read(_bat)== RTT::NewData) {

                        //std::cout << "BATTERRY UPDATE"<<  std::endl;
                }
                if (_px4_mode_input.read(_px4_mode)== RTT::NewData) {

                        //std::cout << "PX4 MODE IN UPDATE"<<  std::endl;
                }
                if (_px4_aux_data_input.read(_px4_aux_data)== RTT::NewData) {

                        //std::cout << "PX4 AUX DATA IN UPDATE"<<  std::endl;
                       //std::cout << _px4_aux_data.nb_motors<<  std::endl;
                }
                if (_local_position_input.read(_local_pos)== RTT::NewData) {

                        //std::cout << "local position ned  UPDATE"<<  std::endl;

                }
                if (_alt_input.read(_alt_ground_laser)== RTT::NewData) {

                       // std::cout << "laser Data  UPDATE"<<  std::endl;

                }
        }

}

void LDC::stopHook() {
  ISAE::component_base::stopHook();
  std::cout << "LDC executes stopping !" << std::endl;
}

void LDC::cleanupHook() {
  ISAE::component_base::cleanupHook();
  std::cout << "LDC cleaning up !" << std::endl;
}

} /*end namespace ISAE*/

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(LDC)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ISAE::LDC)
