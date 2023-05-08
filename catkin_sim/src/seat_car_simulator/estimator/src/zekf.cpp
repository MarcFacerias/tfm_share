#include <est_lib/est_lib.h>

/*

Code used as a shell for the estimators, pick an observer type from the library and run it (uncoment the desired ones)

*/


///////////////////////////////////////////////////////////////////////////////
                                // MAIN //
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

  ros::init(argc, argv, "zekf");
  ros::NodeHandle n;

  ros::Subscriber actu_flag = n.subscribe("flag", 1000, FlagCallback);
  //ros::Publisher est_pub = n.advertise<lpv_mpc::pos_info>("/pos_info", 1); //Close Estimation Loop

  n.getParam("dt", dt);
  dt = 0.005;
  ros::Rate loop_rate(200);

  ros::service::waitForService("LoadMatrices", -1);

  //ObserverLPV observer_lpv; //EKF_LPV
  //ObserverLPVi observer_lpvi; //EKF_LPVi

  //UIO works under certain noise conditions, reduce noise to see a proper estimation of the disturbed state
  //ObserverLPV_UIO observer_uio; // EKF_LPV_UIO
  ObserverLPV_UIOi observer_uioi; // EKF_LPVi_UIO

  GetActuData actuators;
  GetSensorData sensors;
  ComputeError error_manager;
  GetGroundTruthData truth;

  lpv_mpc::pos_info pos_msg;

  int i_it = 0;
  while (ros::ok() && i_it < 2250)
  {

    if (flag && (actuators.a != 0) && (actuators.steer != 0) && (truth.vx > 1)){

        if (i_it == 0){

          /*observer_lpv.states_est(3)      = truth.x;
          observer_lpv.states_est(4)      = truth.y;
          observer_lpv.states_est(5)      = truth.yaw;
          observer_lpv.states_est(0)      = truth.vx;
          observer_lpv.states_est(1)      = truth.vy;
          observer_lpv.states_est(2)      = truth.psiDot;*/

          /*observer_lpvi.states_est(3)      = truth.x;
          observer_lpvi.states_est(4)      = truth.y;
          observer_lpvi.states_est(5)      = truth.yaw;
          observer_lpvi.states_est(0)      = truth.vx;
          observer_lpvi.states_est(1)      = truth.vy;
          observer_lpvi.states_est(2)      = truth.psiDot;*/

          /*observer_uio.states_est(3)      = truth.x;
          observer_uio.states_est(4)      = truth.y;
          observer_uio.states_est(5)      = truth.yaw;
          observer_uio.states_est(0)      = truth.vx;
          observer_uio.states_est(1)      = truth.vy;
          observer_uio.states_est(2)      = truth.psiDot;*/

          observer_uioi.states_est(3)      = truth.x;
          observer_uioi.states_est(4)      = truth.y;
          observer_uioi.states_est(5)      = truth.yaw;
          observer_uioi.states_est(0)      = truth.vx;
          observer_uioi.states_est(1)      = truth.vy;
          observer_uioi.states_est(2)      = truth.psiDot;

        }

        auto start = std::chrono::steady_clock::now();

        //observer_lpv.estimateState(sensors,actuators);
        //observer_lpvi.estimateState(sensors,actuators);
        //observer_uio.estimateState(sensors,actuators);
        observer_uioi.estimateState(sensors,actuators);

        // Close Estimation Loop
        /*lpv_mpc::pos_info est_msg;

        est_msg.x                   = observer.x;
        est_msg.y                   = observer.y;
        est_msg.psi                 = observer.yaw;
        est_msg.v                   = sqrt(observer.vx*truth.vx + observer.vy*truth.vy);
        est_msg.v_x                 = observer.vx;
        est_msg.v_y                 = observer.vy;
        est_msg.psiDot              = observer.psiDot;

        est_pub.publish(est_msg);*/

        auto end = std::chrono::steady_clock::now();
        
        i_it++;

        //std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
        
        /*pos_msg.x = observer.x ;
        pos_msg.y = observer.y;
        pos_msg.psi = observer.yaw;
        pos_msg.v_x = observer.vx;
        pos_msg.v_y = observer.vy;
        pos_msg.psiDot = observer.psiDot;
        states_pub.publish(pos_msg);*/

        ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                          <<" vy "<< truth.vy <<" psiDot " << truth.psiDot << "iteration " << i_it);        

        std::cout << "Elapsed time in microseconds : "
                  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
                  << " µs" << std::endl;
    }

    else if(!truth.x_hist.empty()){

      /*
        // Close Estimation Loop


        lpv_mpc::pos_info est_msg;

        est_msg.x                   = truth.x;
        est_msg.y                   = truth.y;
        est_msg.psi                 = truth.yaw;
        est_msg.v                   = sqrt(truth.vx*truth.vx + truth.vy*truth.vy);
        est_msg.v_x                 = truth.vx;
        est_msg.v_y                 = truth.vy;
        est_msg.psiDot              = truth.psiDot;

        est_pub.publish(est_msg);*/


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

  //error_manager.save_error(truth, observer_lpv, "LPV");
  //error_manager.save_error(truth, observer_lpvi, "ZKF");
  //error_manager.save_error(truth, observer_uio, "LPV_UIO");
  error_manager.save_error(truth, observer_uioi, "ZKF_UIO");

  return 0;
}
