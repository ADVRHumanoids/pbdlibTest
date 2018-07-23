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

#ifndef pdb_test_PLUGIN_H_
#define pdb_test_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>
#include <pbdlib/gmm.h>
#include <pbdlib/gmr.h>
#include <armadillo>
//#include "/home/fabudakka/work/advr-superbuild/build/install/include/pbdlib/gmm.h"
//#include "/home/fabudakka/work/advr-superbuild/build/install/include/pbdlib/gmr.h"



namespace XBotPlugin {

/**
 * @brief pdb_test XBot RT Plugin
 *
 **/
class pdb_test : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~pdb_test();
    
    virtual void minimum_jerk_spline(double x0, double dx0, double ddx0, double x1, double dx1, double ddx1, double T, double a[6]);
    virtual void minimum_jerk(double t, double a[6], double &pos, double &vel, double &acc);

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    
    Eigen::VectorXd _dqTmp, _u;
    XBot::JointNameMap _q_right,_q_curr, _q_home;

    double _start_time, _dt = 0.001, _T = 2.0;

    Eigen::VectorXd _q0;
    Eigen::MatrixXd _q, _dq, _ddq;
    int _N;
    
    std::string _current_command;

    XBot::MatLogger::Ptr _logger;
    
    pbdlib::GMM_Model* _gmm, * _gmmOut;
    pbdlib::GMR _gmr;
    arma::urowvec _in,_out;
    arma::mat _GMRin;		// pbdlib expects a mat type as input to GMR
    arma::colvec _Qd;

    
    //pbdlib::GMM_Model* gmmOut;

};

}

#endif // pdb_test_PLUGIN_H_
