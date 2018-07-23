/*
* Copyright (C) 2017 IIT-ADVR
* Author:
* email:
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <pdb_test_plugin.h>

/* Specify that the class XBotPlugin::pdb_test is a XBot RT plugin with name "pdb_test" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::pdb_test)

namespace XBotPlugin {
    
    bool pdb_test::init_control_plugin(XBot::Handle::Ptr handle)
    {
	/* This function is called outside the real time loop, so we can
	 * allocate memory on the heap, print stuff, ...
	 * The RT plugin will be executed only if this init function returns true. */


	/* Save robot to a private member. */
	_robot = handle->getRobotInterface();
	std::string path_to_config_file = XBot::Utils::getXBotConfig();
	_model = XBot::ModelInterface::getModel(path_to_config_file);


	/* Initialize a logger which saves to the specified file. Remember that
	* the current date/time is always appended to the provided filename,
	* so that logs do not overwrite each other. */

	_logger = XBot::MatLogger::getLogger("/tmp/pdb_test_log");

	
	return true;
    }

    void pdb_test::on_start(double time)
    {
	/* This function is called on plugin start, i.e. when the start command
	* is sent over the plugin switch port (e.g. 'rosservice call /pdb_test_switch true').
	* Since this function is called within the real-time loop, you should not perform
	* operations that are not rt-safe. */
	
	/* Save the plugin starting time to a class member */
	_robot->getMotorPosition(_q0);
	
	/* Save the robot starting config to a class member */
	
	_start_time = time;
	
	Eigen::VectorXd qi(7), qf(7);
	_robot->getJointPosition(_q_curr);
	_model->getRobotState("home",_q_home);
	_q_right.emplace("j_arm2_1", _q_curr["j_arm2_1"]);
	_q_right.emplace("j_arm2_2", _q_curr["j_arm2_2"]);
	_q_right.emplace("j_arm2_3", _q_curr["j_arm2_3"]);
	_q_right.emplace("j_arm2_4", _q_curr["j_arm2_4"]);
	_q_right.emplace("j_arm2_5", _q_curr["j_arm2_5"]);
	_q_right.emplace("j_arm2_6", _q_curr["j_arm2_6"]);
	_q_right.emplace("j_arm2_7", _q_curr["j_arm2_7"]);
	
	std::cout << "q_curr[j_arm2_1] "  << _q_curr["j_arm2_1"] << std::endl;
	std::cout << "q_curr[j_arm2_2] "  << _q_curr["j_arm2_2"]<< std::endl;
	std::cout << "q1 index: "  << _model->getDofIndex("j_arm2_1") << std::endl;
	std::cout << "q2 index: "  << _model->getDofIndex("j_arm2_2") << std::endl;
	
	qi[0] = _q_curr["j_arm2_1"];                  qf[0] = _q_home["j_arm2_1"];
	qi[1] = _q_curr["j_arm2_2"];                  qf[1] = _q_home["j_arm2_2"];
	qi[2] = _q_curr["j_arm2_3"];                  qf[2] = _q_home["j_arm2_3"];
	qi[3] = _q_curr["j_arm2_4"];                  qf[3] = _q_home["j_arm2_4"];
	qi[4] = _q_curr["j_arm2_5"];                  qf[4] = _q_home["j_arm2_5"];
	qi[5] = _q_curr["j_arm2_6"];                  qf[5] = _q_home["j_arm2_6"];
	qi[6] = _q_curr["j_arm2_7"];                  qf[6] = _q_home["j_arm2_7"];
	
	std::cout << "joints qi: " << qi.transpose() << std::endl;
	std::cout << "joints qf: " << qf.transpose() << std::endl;
	
	double a[6], t = 0, T = 2.0, pos, vel, acc;
	    
	_N = (int)T/_dt;
	_q.resize(7,_N);
	_dq.resize(7,_N);
	_ddq.resize(7,_N);
	for (int j = 0; j < 7; j++){
	    t=0;
	    minimum_jerk_spline(qi[j], 0, 0, qf[j], 0, 0, T, a);
	    for (int i = 0; i < _N; i++) {
		minimum_jerk(t+=_dt, a, pos, vel, acc);
		_q(j,i) = pos; 
		_dq(j,i) = vel; 
		_ddq(j,i) = acc;
	    }
	}
    }

    void pdb_test::on_stop(double time)
    {
	/* This function is called on plugin stop, i.e. when the stop command
	* is sent over the plugin switch port (e.g. 'rosservice call /pdb_test_switch false').
	* Since this function is called within the real-time loop, you should not perform
	* operations that are not rt-safe. */

	_u.setZero();
	_dq.setZero();
	_model->setJointEffort(_u);       // Make sure there are no torques being sent
	_model->setJointVelocity(_dq);    // Set joint velocities to zero
	_model->update();

	_robot->setReferenceFrom(*_model, XBot::Sync::All);       // if I use XBot::Sync::Position the robot collapses. Why?
	_robot->move();

    }


    void pdb_test::control_loop(double time, double period)
    {
	/* This function is called on every control loop from when the plugin is start until
	* it is stopped.
	* Since this function is called within the real-time loop, you should not perform
	* operations that are not rt-safe. */
	
	/* The following code checks if any command was received from the plugin standard port
	* (e.g. from ROS you can send commands with
	*         rosservice call /pdb_test_cmd "cmd: 'MY_COMMAND_1'"
	* If any command was received, the code inside the if statement is then executed. */
    
    
	static int i = 0;
	if (_current_command == "demo"){
	    std::cout << "i = " << i <<"   q[6][i] "  << _q(5,i) << std::endl;
	    _q_right["j_arm2_1"] = _q(0,i);
	    _q_right["j_arm2_2"] = _q(1,i);
	    _q_right["j_arm2_3"] = _q(2,i);
	    _q_right["j_arm2_4"] = _q(3,i);
	    _q_right["j_arm2_5"] = _q(4,i);
	    _q_right["j_arm2_6"] = _q(5,i);
	    _q_right["j_arm2_7"] = _q(6,i);
	    
	    _robot->setPositionReference(_q_right);
	    _robot->move();
	    
	    if (i < _N-1)	i++;
	}
    
	static double t = 0;
	if (_current_command == "gmm"){
	    _GMRin(0) = t;
	    _gmr.regression(_gmmOut,_GMRin,_in,_out);
	    _Qd = _gmmOut->getMU(0);

	    _q_right["j_arm2_1"] = _Qd(0,i);
	    _q_right["j_arm2_2"] = _Qd(1,i);
	    _q_right["j_arm2_3"] = _Qd(2,i);
	    _q_right["j_arm2_4"] = _Qd(3,i);
	    _q_right["j_arm2_5"] = _Qd(4,i);
	    _q_right["j_arm2_6"] = _Qd(5,i);
	    _q_right["j_arm2_7"] = _Qd(6,i);
	    
	    _robot->setPositionReference(_q_right);
	    _robot->move();
	    
	    
	    if (t < _T)	t+=_dt;;
	}
	    
	//std::cout << "hello" << std::endl;
	if(!current_command.str().empty()){
	
	    if(current_command.str() == "demo"){
		std::cout << "Creat demos and GMM model"  << std::endl;
		_current_command = current_command.str();
	    
		/*for(auto elem : _q_curr)     std::cout << elem.first << " " << elem.second  << "\n";*/
		/*for(auto elem : _q_right)    std::cout << elem.first << " " << elem.second  << "\n";*/
	    }
	
	    if(current_command.str() == "gmm"){
		_current_command = current_command.str();
		
		std::string PriorsName = "/home/fabudakka/work/advr-superbuild/external/pdb_test/data/priors_centauro.txt";
		std::string MuName     = "/home/fabudakka/work/advr-superbuild/external/pdb_test/data/mu_centauro.txt";
		std::string SigmaName  = "/home/fabudakka/work/advr-superbuild/external/pdb_test/data/sigma_centauro.txt";
		
		// Initialize GMM
		_gmm = new pbdlib::GMM_Model(PriorsName, MuName, SigmaName);
		cout << "\nLoaded GMM." << endl;
		/*for(uint i = 0 ; i < _gmm->getNumSTATES() ; i++)
		{
		    cout << "State #" << i << ":"<< endl;
		    _gmm->getCOMPONENTS(i).getMU().print("Mu = ");
		    _gmm->getCOMPONENTS(i).getSIGMA().print("Sigma = ");
		}*/

		// Initialize GMR object and input-output indices
		_gmr.setGMMModel(_gmm);
		_gmmOut = new pbdlib::GMM_Model(1,7);         // pointer to output GMM (it should be 1 Gaussian, we don't consider the multimodal case)
		
		_GMRin = arma::mat(1,1);
		_in.zeros(1,1);
		_out.zeros(1,7);
		_in(0)=0;
		_out(0)=1;        _out(1)=2;        _out(2)=3;		_out(3)=4;
		_out(4)=5;        _out(5)=6;        _out(6)=7;
		
		_Qd.zeros(7);
	    }
	}
    }

    bool pdb_test::close()
    {
	/* This function is called exactly once, at the end of the experiment.
	* It can be used to do some clean-up, or to save logging data to disk. */

	/* Save logged data to disk */
	_logger->flush();

	return true;
    }

    pdb_test::~pdb_test()
    {
	if (_gmm)	delete _gmm;
	if (_gmmOut)	delete _gmmOut;
    }

    void pdb_test::minimum_jerk_spline(double x0, double dx0, double ddx0, double x1, double dx1, double ddx1, double T, double a[6])
    /*
     * Calculates the coefficients of the minimum jerk spline with given value,
     * first, and second derivative at the end points (0 and T)
     */
    {
	a[0] = x0;
	a[1] = dx0;
	a[2] = ddx0 / 2;
	a[3] = (20 * x1 - 20 * x0 - (8 * dx1 + 12 * dx0) * T - (3 * ddx0 - ddx1) * T * T) / (2 * T * T * T);
	a[4] = (30 * x0 - 30 * x1 + (14 * dx1 + 16 * dx0) * T + (3 * ddx0 - 2 * ddx1) * T * T) / (2 * T * T * T * T);
	a[5] = (12 * x1 - 12 * x0 - (6 * dx1 + 6 * dx0) * T - (ddx0 - ddx1) * T * T) / (2 * T * T * T * T * T);
    }
    void pdb_test::minimum_jerk(double t, double a[6], double &pos, double &vel, double &acc)
    /*
     * Returns the value and derivative of the minimum jerk spline with coefficients a
     * at time t
     */
    {
	double t2, t3, t4, t5;
	t2 = t * t;
	t3 = t2 * t;
	t4 = t2 * t2;
	t5 = t3 * t2;
	pos = a[5] * t5 + a[4] * t4 + a[3] * t3 + a[2] * t2 + a[1] * t + a[0];
	vel = 5 * a[5] * t4 + 4 * a[4] * t3 + 3 * a[3] * t2 + 2 * a[2] * t + a[1];
	acc = 20 * a[5] * t3 + 12 * a[4] * t2 + 6 * a[3] * t + 2 * a[2];
    }

}

