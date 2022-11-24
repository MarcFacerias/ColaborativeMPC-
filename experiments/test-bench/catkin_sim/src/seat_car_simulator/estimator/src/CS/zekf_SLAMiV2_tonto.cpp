#define _GLIBCXX_USE_C99 1
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include <iterator>
#include <vector>
#include <sstream>
#include <deque>
#include <string>
#include <sstream>
#include <l4vehicle_msgs/VehicleState.h>
#include <lpv_mpc/ECU.h>
#include <lpv_mpc/pos_info.h>
#include <lpv_mpc/simulatorStates.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <ctime>
#include <math.h>
#include <fstream>
#include <iterator>
#include <vector>
#include <estimator/lmi_data.h>
#include <estimator/ErrorStates.h>
#include <estimator/ErrorsInfo.h>
#include <estimator/LandmarkInfo.h>
#include <estimator/LandmarksInfo.h>
#include <estimator/DataInfo.h>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include<Eigen/Core>
#include<Eigen/SVD>
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
                                // GLOBAL //
///////////////////////////////////////////////////////////////////////////////

float dt;
bool flag = 0;
float x_r;
float y_r;
float x_t = 0;
float y_t = 0;
float yaw_r;

///////////////////////////////////////////////////////////////////////////////
                             // CALLBACK and AUX //
///////////////////////////////////////////////////////////////////////////////
struct PolarPose{

	float range;
	float angle;
	uint id;

};

struct GlobalPose{

	float x;
	float y;
  float max_x;
  float max_y;
  float min_x;
  float min_y;
	uint id;

};

float wrap(float angle){

/*  if (angle < -3.1416){

    angle += 2*3.1416;

  }

  else if (angle > 3.1416){

    angle -= 2*3.1416;
    
  }*/

  return angle;

}


void Polar(float x, float y, PolarPose *pose){

	pose -> range     = sqrt(x*x + y*y);
	pose -> angle     = atan2(y, x);

}

////////////////////////////classes////////////////////////////
class GetMarkers{

  private:

    ros::NodeHandle n;
    ros::Subscriber marker_sub;

  public:

  	std::deque<PolarPose> landmark_list;

    void GetMarkersStart(){

      marker_sub = n.subscribe("/Corrected_Pose", 1, &GetMarkers::GetMarkersCallback, this);

    }

    void GetMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
    {		

    	for (int i = 0; i < msg.markers.size(); i++){

    		PolarPose pose;

    		Polar(msg.markers[i].pose.pose.position.x, msg.markers[i].pose.pose.position.y, &pose);
    		pose.id = msg.markers[i].id;
    		landmark_list.push_back(pose);

        /*std::cout << "msg recieved!: " << i << " " << std::endl;*/

       /*std::cout << msg.markers[i].id << " " << pose.range << " " << pose.angle << " "  << x_t << " " << y_t << " " << yaw_r 
                  << " " << msg.markers[i].pose.pose.orientation.x << " " <<  -msg.markers[i].pose.pose.orientation.y << " " << 
                    -msg.markers[i].pose.pose.orientation.z << " "<< ros::Time::now().toSec() << " " << msg.markers[i].pose.header.stamp.toSec() <<std::endl;*/


    	}
    }

};

class GetSensorData{

  public:
    float x = 0.3;
    float y = 0;
    float yaw = 0;
    float psiDot = 0;
    float vx = 0;
    ros::NodeHandle n;
    ros::Subscriber pose_sub;

    GetSensorData(){

      pose_sub = n.subscribe("/sensorStates", 1, &GetSensorData::GetPoseCallback, this);

    }

    void GetPoseCallback(const lpv_mpc::simulatorStates& msg)
    {

        x       = msg.x;
        y       = msg.y;
        yaw     = wrap(msg.psi);
        vx      = msg.vx;
        psiDot  = msg.psiDot;

    }

};

class GetGroundTruthData{

  public:
    float x = 0;
    float y = 0;
    float yaw = 0;
    float psiDot = 0;
    float vx = 0;
    float vy = 0;

    std::list <float> x_hist;
    std::list <float> y_hist;
    std::list <float> vx_hist;
    std::list <float> vy_hist;
    std::list <float> yaw_hist;
    std::list <float> psiDot_hist;
    ros::NodeHandle n;
    ros::Subscriber groundTruth_sub;

    GetGroundTruthData(){

      groundTruth_sub = n.subscribe("/vehicle_state", 1, &GetGroundTruthData::GetGroundTruthDataCallback, this);

    }

    void GetGroundTruthDataCallback(const l4vehicle_msgs::VehicleState& msg)
    {

        x       = msg.x;
        y       = msg.y;
        yaw     = wrap(msg.heading);
        vx      = msg.longitudinal_velocity;
        vy      = msg.lateral_velocity;
        psiDot  = msg.angular_velocity;

        x_hist.push_back(x);
        y_hist.push_back(y);
        yaw_hist.push_back(yaw);
        vx_hist.push_back(vx);
        vy_hist.push_back(vy);
        psiDot_hist.push_back(psiDot);

    }

};

class GetActuData{

  public:

    float a  = 0.0;
    float steer = 0.0;
    ros::Subscriber actu_sub;
    ros::NodeHandle n;

    GetActuData(){

      actu_sub = n.subscribe("/ecu", 1, &GetActuData::GetActuCallback, this);

    }

    private:

    void GetActuCallback(const lpv_mpc::ECU& msg)
    {

        a     = msg.motor;
        steer = msg.servo;

    }

};

void FlagCallback(const std_msgs::Bool& msg){

  flag = msg.data;

}


/////////////////////////functions//////////////////////////

VectorXf sort_indexes(const VectorXf v) {

  // initialize original index locations
  VectorXf idx = VectorXf::LinSpaced(v.size()+1, 0, v.size());

  std::stable_sort(idx.data(), idx.data() + idx.size()-1,
       [v](int i1, int i2) {return v(i1) < v(i2);});

  return idx;
}

MatrixXf EnvBox(MatrixXf Rxxio){

  int rows = Rxxio.rows();
  MatrixXf limitMatrix = MatrixXf::Zero(rows,rows);

  for (int i=0; i < rows; i++){

      for (int j=0; j < Rxxio.cols(); j++ ){

            limitMatrix(i,i) += fabs(Rxxio(i,j));

                }
  }

  return limitMatrix;
}

MatrixXf reduction(MatrixXf Rxio, int n_dim){

	int n = Rxio.rows();
	int p = Rxio.cols();

	if (n_dim < n){

        ROS_ERROR_STREAM("Unable to perform reduction, actual dimension smaller than desired dimension");
        return Rxio;

	}

	if (p <= n_dim){

        ROS_ERROR_STREAM("Unable to perform reduction, complexity limit not achieved");
        return Rxio;

	}

	VectorXf NCZ = VectorXf::Zero(p);

	for (int i = 0; i < p; i++ ){

		for (int j = 0; j < n; j++){

			NCZ(i) +=  Rxio(j,i) * Rxio(j,i);

		}

	}

	int nvr = p - n_dim + n -1;
	
	VectorXf I      = sort_indexes(NCZ);
	VectorXf idx_s1 = I(seq(0,nvr));
	VectorXf idx_s2 = I(seq(nvr+1,p-1));

	MatrixXf Out;
	

	if (idx_s2.size() != 0){

		Out.resize(n,idx_s2.size());

		for (int i = 0; i < idx_s2.size(); i++){

			Out.col(i) = Rxio.col(idx_s2(i));

		}

	}

	if (idx_s1.size() != 0){

		MatrixXf Ms;
		Ms.resize(n,idx_s1.size());

		for (int i = 0; i < idx_s1.size(); i++){

			Ms.col(i)  = Rxio.col(idx_s1(i));


		}

		Ms = EnvBox(Ms);

		if (Out.cols() > 0){

			Out.conservativeResize(n,Out.cols() + idx_s1.size());

			Out(all, seq(Out.cols() - idx_s2.size(),Out.cols() )) = Ms;

		} 
		else {

			Out.conservativeResize(n, idx_s1.size());
			Out = Ms;

		} 

	}


	return Out;

}
///////////////////////////////////////////////////////////////////////////////
                                // ESTIMATOR //
///////////////////////////////////////////////////////////////////////////////

class ObserverLPV_SLAM{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    ros::Publisher landmark_pub = n.advertise<estimator::LandmarksInfo>("LandmarkData", 1);


    // model dimension variables
    float n_states  = 8;
    float n_outputs = 7;
    float n_control = 2;

    //general usage variables
    MatrixXf eye8 = MatrixXf::Identity(8, 8);


    //vehicle variables

    float lf;
    float lr;
    float m;
    float I;
    float Cf;
    float Cr;
    float mu;
    float et = 0;

    //matrices
    MatrixXf C;
    MatrixXf A;
    MatrixXf B;
    MatrixXf Aln;
    MatrixXf Bln;
    MatrixXf L;
    MatrixXf L_nlm;
    MatrixXf Rxio = eye8;
    MatrixXf Ew_diag;
    MatrixXf Ev_diag;

    //Vectors
    VectorXf Ew;
    VectorXf Ev;
    VectorXf u;

    //DATA from matlab
    std::vector<MatrixXf> Llmi;
    std::vector<MatrixXf> Llmi_nlm;
    std::vector<std::vector<float>> sched_vars;
    std::vector<std::vector<float>> sched_vars_lm;
    std::list <std::vector<float>> upper_limits;
    std::list <std::vector<float>> lower_limits;


    //DATA from landmarks
    std::deque<GlobalPose> lm_data;
    std::deque<MatrixXf>   R_lm_data;

    //estimated states
    float x      = 0.02;
    float y      = 0.0;
    float vx     = 0.75;
    float vy     = 0.0;
    float yaw    = 0.0;
    float psiDot = 0.0;
    float lambda = 1.0012;
    VectorXf states_est;

    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;
    std::list <std::vector<float>> est_error_hist;
    estimator::LandmarksInfo lnd_list_msg;

  void estimateState(GetSensorData sensor, GetActuData ecu, GetMarkers *landmarks){

    VectorXf y_meas;
    u(0)      = ecu.steer;
    u(1)      = ecu.a;
    bool first_est = 1;
    bool lnd_detected = 0;

	// std::cout << states_est;
    //save states
    vx      = states_est(0);
    vy      = states_est(1);
    psiDot  = states_est(2);
    x       = states_est(3);
    y       = states_est(4);
    yaw     = states_est(5);

    states_est(5) = (states_est(5));

  	if (!landmarks->landmark_list.empty())
  	{

  		//std::cout << "lnd to analyse" << std::endl;

      if (lm_data.empty()){

        GlobalPose newPose;
        newPose.id = landmarks->landmark_list.back().id;
        newPose.x  = x + landmarks->landmark_list.back().range*cos(landmarks->landmark_list.back().angle + yaw);
        newPose.y  = y + landmarks->landmark_list.back().range*sin(landmarks->landmark_list.back().angle + yaw);
        //newPose.x  = x_t + landmarks->landmark_list.back().range*cos(landmarks->landmark_list.back().angle + yaw);
        //newPose.y  = y_t + landmarks->landmark_list.back().range*sin(landmarks->landmark_list.back().angle + yaw);

        lm_data.push_back(newPose);
        MatrixXf R_lm  = eye8;
        R_lm_data.push_back(R_lm);

        landmarks->landmark_list.pop_back();
      }

  		while (!landmarks->landmark_list.empty() && (lnd_detected == 0)){

  			int it;
  			for (it = 0; it < lm_data.size(); it++){

  				if (lm_data[it].id == landmarks->landmark_list.back().id){

  					lnd_detected = 1;
  					break;

  				}
  			} 
  		
  		if (!lnd_detected){

  			GlobalPose newPose;
  			newPose.id = landmarks->landmark_list.back().id;
  			newPose.x  = x + landmarks->landmark_list.back().range*cos(landmarks->landmark_list.back().angle + yaw);
  			newPose.y  = y + landmarks->landmark_list.back().range*sin(landmarks->landmark_list.back().angle + yaw);
  			lm_data.push_back(newPose);

	        MatrixXf R_lm  = eye8;
	        R_lm_data.push_back(R_lm);
	        
	        landmarks->landmark_list.pop_back();
	  		break;

  		}

  		if (states_est.size() == 6){

  			states_est.conservativeResize(8);
 
  		}

	    if (Rxio.rows() < 8){

	        Rxio.conservativeResize(8,8);

	    }

  		states_est(6) = lm_data[it].x;
  		states_est(7) = lm_data[it].y;

  		if (y_meas.size() != n_outputs){

  			y_meas.conservativeResize(n_outputs);

  		}

  		y_meas(0) = sensor.vx;
  		y_meas(1) = sensor.psiDot;
  		y_meas(2) = sensor.x;
  		y_meas(3) = sensor.y;
  		y_meas(4) = sensor.yaw;
  		y_meas(5) = landmarks->landmark_list.back().range;
  		y_meas(6) = 0;

  		float est_y = landmarks->landmark_list.back().angle;


      //float est_y = atan2(lm_data[it].y - y, lm_data[it].x - x);

  		//std::cout << landmarks->landmark_list.back().angle << " " << atan2(lm_data[it].y - y, lm_data[it].x - x) << std::endl; 

	    AB_computation(u(0), est_y ,first_est);
		  L_computation(u(0), est_y);

		MatrixXf prior = Aln *states_est + Bln*u;

	    states_est = prior;// + L * (y_meas - C*prior);

        //Store center landmark data
        lm_data[it].x = states_est(6);
        lm_data[it].y = states_est(7);


      Rxio(seq(6,7),all) = R_lm_data[it](seq(6,7),all);
      Rxio(all,seq(6,7)) = R_lm_data[it](all,seq(6,7));
      //std::cout << "drama" << std::endl;
      MatrixXf shape_priori;

      /*std::cout << "Rxio" << std::endl;
      std::cout << Rxio << std::endl;*/

      shape_priori.resize(int(Aln.rows()),int(Rxio.cols()+ 8));

      //std::cout << "Aln*Rxio" << std::endl;

      shape_priori << Aln*Rxio, 2*Ew_diag;

      /*std::cout << "shape_priori" << std::endl;
      std::cout << shape_priori << std::endl;*/

      Rxio.resize(int(n_states), shape_priori.cols() + n_outputs);

      MatrixXf aux_mat;
      aux_mat.resize(int(n_states), shape_priori.cols() + n_outputs);
      aux_mat << (eye8 - L*C)*shape_priori, -L*2*Ev_diag;

      /*std::cout << "aux_mat" << std::endl;
      std::cout << aux_mat << std::endl;*/

      Rxio = reduction(aux_mat,8);
      R_lm_data[it] = Rxio;

      /*std::cout << "Rxio" << std::endl;
      std::cout << Rxio << std::endl;*/

      //Compute interval limits 
      MatrixXf LimitMatrix = EnvBox(Rxio);

      /*std::cout << "LimitMatrix" << std::endl;
      std::cout << LimitMatrix << std::endl;*/


      std::vector<float> aux_vec_max;
      std::vector<float> aux_vec_min;

      for (int i = 0; i < 6; i++){

        aux_vec_max.push_back(states_est(i) + fabs(LimitMatrix(i,i)));
        aux_vec_min.push_back(states_est(i) - fabs(LimitMatrix(i,i)));

      }

      //Store interval limits 
      upper_limits.push_back(aux_vec_max);
      lower_limits.push_back(aux_vec_min);

      int i = 6;

      lm_data[it].max_x = states_est(i) + fabs(LimitMatrix(i,i));
      lm_data[it].min_x = states_est(i) - fabs(LimitMatrix(i,i));
      
      i = 7;

      lm_data[it].max_y = states_est(i) + fabs(LimitMatrix(i,i));
      lm_data[it].min_y = states_est(i) - fabs(LimitMatrix(i,i));

      //std::cout << "max "<< lm_data[it].max_y << " " << lm_data[it].max_x  << std::endl;
      //std::cout << "min "<< lm_data[it].min_y << " " << lm_data[it].min_x  << std::endl;

      //Remove observed landmark 
      landmarks->landmark_list.pop_back(); 
      storeLandmarkData(lm_data[it].x,lm_data[it].y,lm_data[it].id,lm_data[it].max_x,lm_data[it].min_x,lm_data[it].max_y,lm_data[it].min_y);
  	 }
  	}

  	else{

        //std::cout << "no landmark analysis" << std::endl;
        y_meas.conservativeResize(n_outputs - 2);
        y_meas(0) = sensor.vx;
        y_meas(1) = sensor.psiDot;
        y_meas(2) = sensor.x;
        y_meas(3) = sensor.y;
        y_meas(4) = sensor.yaw;


        // update matrices
        AB_computation(u(0), 0, 1);

        L_nlm_computation(u(0));
        //priori estimation

        MatrixXf prior = Aln(seq(0,5),seq(0,5)) *states_est(seq(0,5)) + Bln(seq(0,5),all)*u;

        states_est = prior; //+ L_nlm * (y_meas - C(seq(0,4),seq(0,5))*prior);

        if (Rxio.rows() != 6){

          Rxio.conservativeResize(6,6);

        }

        MatrixXf shape_priori;
        shape_priori.resize(int(Aln.rows()-2),int(Rxio.cols()+ 6));

/*        std::cout << "Rxio" << std::endl;
        std::cout << Rxio << std::endl;*/

        shape_priori << Aln(seq(0,5),seq(0,5))*Rxio, Ew_diag(seq(0,5),seq(0,5));
        /*std::cout << "shape_priori" << std::endl;
        std::cout << shape_priori << std::endl;*/


        Rxio.resize(int(n_states-2), shape_priori.cols() + n_outputs-2);

        MatrixXf aux_mat;
        aux_mat.resize(int(n_states)-2, shape_priori.cols() + n_outputs-2);
        aux_mat << (MatrixXf::Identity(6, 6) - L_nlm*C(seq(0,4),seq(0,5)))*shape_priori, -L(seq(0,5),seq(0,4))*Ev_diag(seq(0,4),seq(0,4));
        /*std::cout << "aux_mat" << std::endl;
        std::cout << aux_mat << std::endl;*/
        Rxio = reduction(aux_mat,6);

        



	    /*ROS_ERROR_STREAM("No LMd Truth:  x " << x <<" y "<< y <<" yaw " << yaw <<" vx " << vx
            <<" vy "<< vy <<" psiDot " << psiDot );*/

  	}

        /*if (!lm_data.empty())
        {
  
          std::cout << lm_data.back().x << " " << lm_data.back().y <<std::endl; 

        }*/

        x_est_hist.push_back(x);
        y_est_hist.push_back(y);
        yaw_est_hist.push_back(yaw);
        vx_est_hist.push_back(vx);
        x_r = x;
        y_r = y;
        vy_est_hist.push_back(vy);
        psiDot_est_hist.push_back(psiDot);


      }

    ObserverLPV_SLAM(){

      A.resize(n_states,n_states);
      B.resize(n_states,n_control);
      Aln.resize(n_states,n_states);
      Bln.resize(n_states,n_control);
      C.resize(n_outputs,n_states);
      L.resize(n_states,n_outputs);
      L_nlm.resize(n_states-2,n_outputs-2);
      Ew.resize(n_states-2);
      Ev.resize(n_outputs-2);
      Ew_diag.resize(n_states,n_states);
      Ev_diag.resize(n_outputs,n_outputs);
      u.resize(n_control);
      states_est.resize(n_states);

      n.getParam("lf",lf);
      n.getParam("lr", lr);
      n.getParam("m", m);
      n.getParam("Iz",I);
      n.getParam("Cf",Cf);
      n.getParam("Cr",Cr);
      n.getParam("mu",mu);

      C << 1, 0, 0, 0, 0, 0, 0, 0, 
           0, 0, 1, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0;

      A = MatrixXf::Zero(n_states,n_states);

      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;


      u << 0,0;
      Ew = VectorXf::Zero(n_states);
      Ev = VectorXf::Zero(n_outputs);
      Ew_diag = MatrixXf::Zero(n_states,n_states);
	  Ev_diag = MatrixXf::Zero(n_outputs,n_outputs);



      for (int i=0; i<n_states; i++){

        Ew_diag(i,i) = Ew(i);

      }

      for (int i=0; i<n_outputs; i++){

        Ev_diag(i,i) = Ev(i);

      }

      std::cout << Ev_diag << std::endl;

      states_est << vx, vy, psiDot, x, y, yaw, 0, 0;
      LoadMatrices();
      LoadMatrices_nlm();
    }

    void storeLandmarkData(float x, float y, float id, float max_x, float max_y, float min_x, float min_y){

      estimator::LandmarkInfo lnd_msg;

      //std::cout << lm_data.size() << std::endl;

      /*for (int i = 0; i < lm_data.size(); i++){

        lnd_msg.x.push_back(lm_data[i].x);
        lnd_msg.y.push_back(lm_data[i].y);
        lnd_msg.id.push_back(lm_data[i].id);
        lnd_msg.max_x.push_back(lm_data[i].max_x);
        lnd_msg.max_y.push_back(lm_data[i].max_y);
        lnd_msg.min_x.push_back(lm_data[i].min_x);
        lnd_msg.min_y.push_back(lm_data[i].min_y);

      }*/

        lnd_msg.x.push_back(x);
        lnd_msg.y.push_back(y);
        lnd_msg.id.push_back(id);
        lnd_msg.max_x.push_back(max_x);
        lnd_msg.max_y.push_back(max_y);
        lnd_msg.min_x.push_back(min_x);
        lnd_msg.min_y.push_back(min_y);

      lnd_list_msg.lnd_list.push_back(lnd_msg);

    }

    private:

    void LoadMatrices(){
      estimator::lmi_data srv;
      srv.request.est_id = "LPV_SLAM";
      client.call(srv);

      ROS_ERROR_STREAM("Loading" << srv.response.L.size() << " matrices "); 
      Llmi.resize(srv.response.L.size());

      // fill LMIs
      for (int i = 0; i < srv.response.L.size(); i++){

        Llmi[i].resize(n_states,n_outputs);

        for (int rows = 0; rows < n_states; rows++){

          for (int cols = 0; cols < n_outputs; cols++){

            Llmi[i](rows,cols) = srv.response.L[i].gains[rows*(n_outputs) + cols];

          }
        }
        std::cout << Llmi[i] << std::endl;
      }
      std::cout << "end Lnd Gain" << std::endl;
      //fill sched_vars

      sched_vars.resize(srv.response.limits.max.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars[i].push_back(srv.response.limits.min[i]);
        sched_vars[i].push_back(srv.response.limits.max[i]);

      }

    }

    void LoadMatrices_nlm(){

      estimator::lmi_data srv;
      srv.request.est_id = "LPV";
      client.call(srv);

      ROS_ERROR_STREAM("Loading" << srv.response.L.size() << " matrices "); 
      Llmi_nlm.resize(srv.response.L.size());

      int n_states_nlm = 6;
      int n_outputs_nlm = 5;

      // fill LMIs
      for (int i = 0; i < srv.response.L.size(); i++){

        Llmi_nlm[i].resize(n_states_nlm,n_outputs_nlm);

        for (int rows = 0; rows < n_states_nlm; rows++){

          for (int cols = 0; cols < n_outputs_nlm; cols++){

            Llmi_nlm[i](rows,cols) = srv.response.L[i].gains[rows*(n_outputs_nlm) + cols];

          }
        } 
        std::cout << Llmi_nlm[i] << std::endl;
      }
      std::cout << "end Gain" << std::endl;

      //fill sched_vars

	  sched_vars_lm.resize(srv.response.limits.max.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars_lm[i].push_back(srv.response.limits.min[i]);
        sched_vars_lm[i].push_back(srv.response.limits.max[i]);

      }

    }
  

    void AB_computation(float steer, float theta_lm , bool first_est){

      //update B
      B(0,0) = -sin(steer) * Cf/m;
      B(1,0) = (cos(steer) * Cf) / m;
      B(2,0) = (lf * Cf * cos(steer)) / I;
      B(1,1) = cos(steer);
      B(2,1) = sin(steer);


      if (first_est) {

  	      //Update A
	      A(0,0) =  -mu;
	      A(0,1) = (sin(steer) * Cf) / (m*vx);
	      A(0,2) = (sin(steer) * Cf * lf) / (m*vx) + vy;

	      A(1,1) = -(Cr + Cf * cos(steer)) / (m*vx);
	      A(1,2) = -(lf * Cf * cos(steer) - lr * Cr) / (m*vx) - vx;

	      A(2,1) = -(lf * Cf * cos(steer) - lr * Cr) / (I*vx);
	      A(2,2) = -(lf * lf * Cf * cos(steer) + lr * lr * Cr) / (I*vx);

	      A(3,0) = cos(yaw);
	      A(3,1) = -sin(yaw);

	      A(4,0) = sin(yaw);
	      A(4,1) = cos(yaw);

	      A(5,2) = 1;

      }

      else{

  	      //Update A
	      A(0,0) =  0;
	      A(0,1) =  0;
	      A(0,2) =  0;

	      A(1,1) =  0;
	      A(1,2) =  0;
	      A(2,1) =  0;
	      A(2,2) =  0;

	      A(3,0) =  0;
	      A(3,1) =  0;

	      A(4,0) =  0;
	      A(4,1) =  0;

	      A(5,2) =  0;

      }

      C(5,3) = -lambda*cos(theta_lm);
      C(5,4) = -lambda*sin(theta_lm);
      C(5,6) = lambda*cos(theta_lm);
      C(5,7) = lambda*sin(theta_lm);

      C(6,3) = -lambda*sin(theta_lm);
      C(6,4) = lambda*cos(theta_lm);
      C(6,6) = lambda*sin(theta_lm);
      C(6,7) = -lambda*cos(theta_lm);



      Aln = eye8 + (A*dt);
      Bln = B*dt;

    }


    // void L_computation(MatrixXf &L, float vx, float vy, float theta, float steer){

    void L_computation(float steer, float theta_lm){

		float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
		float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
		float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
		float M_theta_min       = (sched_vars[5][1] - yaw) / (sched_vars[5][1] - sched_vars[5][0]);
		float M_Stheta_lm       = (sched_vars[7][1] - sin(theta_lm)) / (sched_vars[6][1] - sched_vars[6][0]);
		float M_Ctheta_lm       = (sched_vars[6][1] - cos(theta_lm)) / (sched_vars[7][1] - sched_vars[7][0]);

		VectorXf mu_sch;
		mu_sch.resize(64);

		mu_sch[0]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm                * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
		mu_sch[1]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
		mu_sch[2]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[3]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[4]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
		mu_sch[5]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[6]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[7]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[8]   = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
		mu_sch[9]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
		mu_sch[10]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[11]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[12]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
		mu_sch[13]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[14]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[15]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);  

		mu_sch[16]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
		mu_sch[17]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
		mu_sch[18]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[19]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[20]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
		mu_sch[21]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[22]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[23]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[24]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
		mu_sch[25]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
		mu_sch[26]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[27]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[28]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
		mu_sch[29]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[30]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[31]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);  

		mu_sch[32]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
		mu_sch[33]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
		mu_sch[34]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[35]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[36]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
		mu_sch[37]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[38]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[39]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm                * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[40]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
		mu_sch[41]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
		mu_sch[42]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[43]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[44]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
		mu_sch[45]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[46]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[47]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);  

		mu_sch[48]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
		mu_sch[49]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
		mu_sch[50]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[51]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[52]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
		mu_sch[53]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[54]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[55]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[56]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
		mu_sch[57]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
		mu_sch[58]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[59]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[60]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
		mu_sch[61]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[62]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
		mu_sch[63]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);  

		MatrixXf result = MatrixXf::Zero(n_states,n_outputs);

		for (int i = 0; i < 64; i++){

		  result += mu_sch[i] * Llmi[i];

		}

		L = -result*dt;
    }

    void L_nlm_computation(float steer){


      float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
      float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
      float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
      float M_theta_min       = (sched_vars[5][1] - yaw)   / (sched_vars[5][1] - sched_vars[5][0]);

      VectorXf mu_sch;
      mu_sch.resize(16);

      mu_sch[0]               = M_vx_despl_min         * M_vy_despl_min      * M_steer_min      *  M_theta_min;
      mu_sch[1]               = M_vx_despl_min         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
      mu_sch[2]               = M_vx_despl_min         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
      mu_sch[3]               = M_vx_despl_min         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[4]               = M_vx_despl_min         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
      mu_sch[5]               = M_vx_despl_min         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
      mu_sch[6]               = M_vx_despl_min         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
      mu_sch[7]               = M_vx_despl_min         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

      mu_sch[8]               = (1-M_vx_despl_min)     * M_vy_despl_min      * M_steer_min      *  M_theta_min;
      mu_sch[9]               = (1-M_vx_despl_min)     * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
      mu_sch[10]              = (1-M_vx_despl_min)     * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
      mu_sch[11]              = (1-M_vx_despl_min)     * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[12]              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
      mu_sch[13]              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
      mu_sch[14]              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
      mu_sch[15]              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

      MatrixXf result = MatrixXf::Zero(n_states-2,n_outputs-2);

      for (int i = 0; i < 16; i++){

          result += mu_sch[i] * Llmi_nlm[i];

      }

      L_nlm = -result*dt;
    }
};

class ComputeError{

public:

  ros::Publisher error_pub;
  ros::Publisher data_pub;
  ros::NodeHandle n;
  std::vector<estimator::ErrorStates> errors;
  std::vector<float> rmse;

  ComputeError(){

    error_pub = n.advertise<estimator::ErrorsInfo>("errors", 5);
    data_pub = n.advertise<estimator::DataInfo>("data", 5);

  }

    ///////////////////////////////////////////////////////////////////
   //////////////////////////OVERLOADED FUNCTION//////////////////////
  ///////////////////////////////////////////////////////////////////
  void compute_error(GetGroundTruthData truth,ObserverLPV_SLAM estimator){

    estimator::ErrorStates temp;

    temp.vx = truth.vx - estimator.vx;
    temp.vy = truth.vy - estimator.vy;
    temp.psiDot = truth.psiDot - estimator.psiDot;
    temp.x = truth.x - estimator.x;
    temp.y = truth.y - estimator.y;
    temp.psi = truth.yaw - estimator.yaw;

    // std::cout << temp << '\n';
    errors.push_back(temp);
  }

  void compute_rmse(std::vector<estimator::ErrorStates> errors){


    float error_x = 0;
    float error_y = 0;
    float error_psi = 0;
    float error_vx = 0;
    float error_vy = 0;
    float error_psiDot = 0;
    float numel = 0;

    for (int i = 0; i < errors.size(); i++)
      {
        // Access the object through iterator

        error_x      += errors[i].x*errors[i].x;
        error_y      += errors[i].y*errors[i].y;
        error_psi    += errors[i].psi*errors[i].psi;
        error_vx     += errors[i].vx*errors[i].vx;
        error_vy     += errors[i].vy*errors[i].vy;
        error_psiDot += errors[i].psiDot*errors[i].psiDot;

        numel++;
      }

    rmse = { sqrt(error_x/numel), sqrt(error_y/numel), sqrt(error_psi/numel),
             sqrt(error_vx/numel), sqrt(error_vy/numel), sqrt(error_psiDot/numel)};


  }

  void save_error(GetGroundTruthData truth,ObserverLPV_SLAM estimator){

    //Init com
    estimator::ErrorsInfo msg;
    estimator::DataInfo msg_data;
    compute_rmse(errors);
    msg.rmse = rmse;

    //Parse data auxiliar structs
    estimator::ErrorStates aux;

    for (int i = 0; i < errors.size(); i++ ){

      msg.err.push_back(errors[i]);

    }

    while (!truth.x_hist.empty() && !estimator.x_est_hist.empty()){

      aux.x       = truth.x_hist.front();
      aux.y       = truth.y_hist.front();
      aux.psi     = truth.yaw_hist.front();
      aux.vx      = truth.vx_hist.front();
      aux.vy      = truth.vy_hist.front();
      aux.psiDot  = truth.psiDot_hist.front();

      truth.x_hist.pop_front();
      truth.y_hist.pop_front();
      truth.yaw_hist.pop_front();
      truth.vx_hist.pop_front();
      truth.vy_hist.pop_front();
      truth.psiDot_hist.pop_front();

      msg_data.ground.push_back(aux);

      aux.x       = estimator.x_est_hist.front();
      aux.y       = estimator.y_est_hist.front();
      aux.psi     = estimator.yaw_est_hist.front();
      aux.vx      = estimator.vx_est_hist.front();
      aux.vy      = estimator.vy_est_hist.front();
      aux.psiDot  = estimator.psiDot_est_hist.front();

      estimator.x_est_hist.pop_front();
      estimator.y_est_hist.pop_front();
      estimator.yaw_est_hist.pop_front();
      estimator.vx_est_hist.pop_front();
      estimator.vy_est_hist.pop_front();
      estimator.psiDot_est_hist.pop_front();

      msg_data.data.push_back(aux);

    }

    error_pub.publish(msg);
    data_pub.publish(msg_data);

  }
};


///////////////////////////////////////////////////////////////////////////////
                                // ESTIMATOR //
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

  ros::init(argc, argv, "zekf");
  ros::NodeHandle n;

  ros::Subscriber actu_flag = n.subscribe("flag", 1000, FlagCallback);

  n.getParam("dt", dt);
  dt = 0.05;
  ros::Rate loop_rate(200);

  ros::service::waitForService("LoadMatrices", -1);

  ObserverLPV_SLAM observer;
  GetActuData actuators;
  GetSensorData sensors;
  ComputeError error_manager;
  GetGroundTruthData truth;
  GetMarkers markers;

  int i_it = 0;

  auto start = std::chrono::steady_clock::now();

  while (ros::ok() && i_it < 2000)
  {

    if (flag && (actuators.a != 0) && (actuators.steer != 0) && (truth.vx > 1)){

        if (i_it == 0){

          start = std::chrono::steady_clock::now();

          markers.GetMarkersStart();

          observer.states_est(3)      = truth.x;
          observer.states_est(4)      = truth.y;
          observer.states_est(5)      = truth.yaw;
          observer.states_est(0)      = truth.vx;
          observer.states_est(1)      = truth.vy;
          observer.states_est(2)      = truth.psiDot;

        }

        x_t = truth.x;
		    y_t = truth.y;
        yaw_r = truth.yaw;

        observer.estimateState(sensors, actuators, &markers);
        error_manager.compute_error(truth, observer);
        i_it++;


	     /* ROS_ERROR_STREAM("update finished: x " << observer.x <<" y "<< observer.y <<" yaw " << observer.yaw <<" vx " << observer.vx
	                <<" vy "<< observer.vy <<" psiDot " << observer.psiDot );


        ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                          <<" vy "<< truth.vy <<" psiDot " << truth.psiDot << "iteration " << i_it);*/
    }

    else if(!truth.x_hist.empty()){

        truth.x_hist.pop_front();
        truth.y_hist.pop_front();
        truth.yaw_hist.pop_front();
        truth.vx_hist.pop_front();
        truth.vy_hist.pop_front();
        truth.psiDot_hist.pop_front();

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  auto end = std::chrono::steady_clock::now();

	/*std::cout << "Elapsed time in milliseconds : "
	          << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
	          << " s" << std::endl;*/

  error_manager.save_error(truth, observer);
  observer.landmark_pub.publish(observer.lnd_list_msg);

  return 0;
}
