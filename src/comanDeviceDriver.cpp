#include "coman.h"
#include <boost/archive/text_iarchive.hpp>


using namespace yarp::dev;



bool coman::open(yarp::os::Searchable& config) 
{
    //if there is a .ini file, directly load it avoiding copyng all the data from config
    if( config.check("gazebo_ini_file_path") ) {
        plugin_parameters.fromConfigFile(config.find("gazebo_ini_file_path").asString().c_str());
    }
    
    //I love everything and every interface
    uintptr_t temp;
    std::istringstream iss(config.find("loving_gazebo_pointer").asString().c_str());
    boost::archive::text_iarchive archive(iss);
    try{
        archive>>temp;
    }
    catch (const char *ex)
    {
        //TODO
    }
    _robot=reinterpret_cast<gazebo::physics::Model*>(temp);
    
    gazebo_init();
    return RateThread::start();
}



bool coman::close() //NOT IMPLEMENTED
{
    delete [] control_mode;
    delete [] motion_done;
    return true;
}


//We need a thread to publish some extra information like joint torques and velocities.
bool coman::threadInit()
{
    std::string gazebo_group_name = "GAZEBO";
    std::stringstream property_name;
    property_name<<"name";
    yarp::os::Bottle& name = plugin_parameters.findGroup(gazebo_group_name.c_str()).findGroup(property_name.str().c_str());
    
    
    std::stringstream port_name_torque;
    port_name_torque<<"/wholeBodyDynamics"<<name.get(1).asString().c_str()<<"/Torques:o";
    _joint_torq_port.open(port_name_torque.str().c_str());
    std::stringstream port_name_speed;
    port_name_speed<<"/wholeBodyDynamics"<<name.get(1).asString().c_str()<<"/Speeds:o";
    _joint_speed_port.open(port_name_speed.str().c_str());
    return true;
}

void coman::afterStart(bool s)
{
    if(s)
        printf("TorqueAndSpeedPublisher started successfully\n");
    else
        printf("TorqueAndSpeedPublisher did not start\n");
}

void coman::run()
{
    yarp::os::Bottle bot1;
    yarp::os::Bottle bot2;
    for(unsigned int j = 0; j < _robot_number_of_joints; ++j){
        bot1.addDouble(torque[j]);
        bot2.addDouble(speed[j]);
    }
    _joint_torq_port.write(bot1);
    _joint_speed_port.write(bot2);
    bot1.clear();
    bot2.clear();
}

void coman::threadRelease()
{
    printf("Goodbye from TorqueAndSpeedPublisher\n");
}