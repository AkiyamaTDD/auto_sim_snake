/**
 * @file auto_sim_snake.cpp
 * @brief 阿部さんのシミュレータを自動的に回すプログラム
 * @author Taichi Akiyama
 * @date 2017/11/30
 * @detail
 */

 #ifndef AUTO_SIM_SNAKE_H_
 #define AUTO_SIM_SNAKE_H_

 #include <string>
 #include <cmath>
 #include <vector>
 #include "ros/ros.h"
 #include <std_msgs/Float32.h>
 #include <std_msgs/Float32MultiArray.h>
 #include <std_msgs/Float64.h>
 #include <std_msgs/Float64MultiArray.h>
 extern "C" {
   #include "extApi.h"
 }


static const int NUM_JOINT = 20;
simxInt client_id;
float torque_data[NUM_JOINT];

/** @fn
 * @brief V-REPに接続する
 * @param なし
 * @return なし
 */
void ConnectToVrep() {
  ROS_INFO("Connecting to v-rep");
  client_id = simxStart(/* connectionAddress = */ (simxChar*)"127.0.0.1",
                           /* connectionPort = */ 19997,  // 19997にすればvrep側を手動で開始する必要がない
                           /* waitUntilConnected = */ true,
                           /* doNotReconnectOnceDisconnected = */ true,
                           /* timeOutInMs = */ 5000,
                           /* commThreadCycleInMs = */ 5);

  if (client_id == -1) {
    ROS_ERROR("Could not connect to V-REP remote API server");
  } else {
    ROS_INFO("Connected to V-REP remote API server");
  }
}



/** @fn
 * @brief トルクのコールバック関数
 * @param std_msgs::Float32MultiArray
 * @return なし
 * @detail
 */
void CallBackOfTorque(const std_msgs::Float32MultiArray::ConstPtr& torque_datain) {
  float t_data;
  t_data = torque_datain->data[0];

  for(int i=0; i<NUM_JOINT; i++){
    torque_data[i] = torque_datain->data[i+1];
  }
}

/** @fn
 * @brief 現在時間取得
 * @param なし
 * @return double
 * @detail
 */
double GetTime() {
  double now_time,ts,tn;
  ros::Time t = ros::Time::now();
  ts = t.sec;
  tn = t.nsec;
  now_time = ts + tn/(1000*1000*1000);
  return now_time;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "auto_sim_snake");
  ROS_INFO("Start Auto Simulation");

  //Subscriber
  ros::NodeHandle nh;
  ros::Subscriber sub_torque;
  sub_torque = nh.subscribe<std_msgs::Float32MultiArray>("torque_data", 100, CallBackOfTorque);
  ros::Rate loop_rate(50);

  //Publisher
  ros::Publisher pub_param;
  pub_param = nh.advertise<std_msgs::Float64MultiArray>("auto_change_param", 100);

  ConnectToVrep();

  int count = 0;
  double k = 0;
  simxFloat pos[3];
  char filename[30];
  double start_time,now_time;
  int mode = 0; //0:初期化 1:ループ 2:ループ終了
  //mode = 5; //初期値を与えるとき
  std_msgs::Float64MultiArray param;

  param.data.clear();
  param.data.push_back(k);
  pub_param.publish(param);

  simxInt head_handle;
  simxGetObjectHandle(client_id, (simxChar*) "isnake_hp_robot", &head_handle, simx_opmode_oneshot_wait);
  simxGetObjectPosition(client_id, head_handle, -1, pos, simx_opmode_streaming);
  FILE *fp;

  while (ros::ok())
  {
    ros::spinOnce(); //コールバックに必須
    switch ( mode )
    {
      case 0:
        //初期値設定〜ファイル作成
        simxGetObjectPosition(client_id, head_handle, -1, pos, simx_opmode_buffer);
        simxStartSimulation(client_id, simx_opmode_oneshot_wait);
        if (pos[0] > 2.1) //  位置が戻っていなければ1回待つ
          break;
        param.data.clear();
        param.data.push_back(k);
        pub_param.publish(param);
        start_time = GetTime();
        sprintf(filename, "Auto_k=%1.3f_count=%d.dat",k,count);
        fp=fopen(filename,"w");
        //fprintf(fp, "%s\n", filename);
        mode = 1;
        break;
      case 1:
        //ループ
        simxGetObjectPosition(client_id, head_handle, -1, pos, simx_opmode_buffer);
        now_time = GetTime() - start_time;
        fprintf(fp, "%f", now_time);
        for (int i=0; i<NUM_JOINT; i++){
          ROS_INFO("%f",torque_data[i]);
          fprintf(fp, ", %f" ,torque_data[i]);
        }
        fprintf(fp, "\n");
        if (pos[0] > 2.1)
          mode = 2;
        break;
      case 2:
        //ファイル終了＆次回処理
        simxStopSimulation(client_id, simx_opmode_oneshot_wait);
        k += 0.01;
        fclose(fp);
        mode = 0;
        if (k>0.2){
          k=0.0;
          count++;
        }
        if (count >=10)
          mode = 4;
          ROS_INFO("Mode is 4");
        break;
      case 5:
        //初期値を与えるときの待ちを作る
        mode=0;
        break;
      default:
        ROS_INFO("This mode isn't defined");
        break;
    }
    loop_rate.sleep();
  }


  return 0;
}

#endif /* AUTO_SIM_SNAKE_H_ */
