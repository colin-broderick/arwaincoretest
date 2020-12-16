#include <stdio.h>
#include <time.h>
// #include <uv.h>

#include "bmi2.h"
#include "bmi270.h"
#include "pi_utils.h"

// #include <evnsq/consumer.h>
// #include <evnsq/producer.h>
// #include <evpp/event_loop.h>

#include <chrono>
#include <thread>

#include <getopt.h>


// class BMI270
// {
// public:
//     BMI270(char* host) : client(&loop, evnsq::Option()) 
//     {
// 		topic_i = "imu";
// 		topic_m = "mag";
// 		msg = "0s0s0s0s0s0";
		
// 		client.SetReadyCallback(std::bind(&BMI270::on_ready, this));
		
// 		std::string nsqd_tcp_addr(host);
// 		client.ConnectToNSQDs(nsqd_tcp_addr);
		
// 		/*auto f = [](evpp::EventLoop* l, evnsq::Producer* c, BMI270* b) {
// 			std::this_thread::sleep_for(std::chrono::seconds(2));
// 			for (;;) {
// 				if (l->pending_functor_count() > 10000) {
// 					std::this_thread::sleep_for(std::chrono::milliseconds(20));
// 				} else {
// 					b->imu_reading();
// 				}
// 			}
// 		};
// 		std::thread publish_thread(std::bind(f, &loop, &client, this));*/
    
// 		loop.Run();
//     }
    
//     void on_ready()
//     {
// 		loop.RunEvery(evpp::Duration(0.001), std::bind(&BMI270::imu_reading, this));
// 		//loop.RunEvery(evpp::Duration(0.033333333333), std::bind(&BMI270::mag_reading, this));
// 	}

//     void imu_reading()
//     {
// 		std::stringstream ss;
//         get_bmi270_data(&acc_d, &gyr_d);
//         ss << gyr_d.x << "s" << gyr_d.y << "s" << gyr_d.z << "s";
//         ss << acc_d.x << "s" << acc_d.y << "s" << acc_d.z;
//         msg = ss.str();
//         client.Publish(topic_i, msg);
//     }
    
//     void mag_reading()
//     {
// 		std::stringstream ss;
//         get_bmm150_data(&mag_d);
//         ss << mag_d.x << "s" << mag_d.y << "s" << mag_d.z;
//         msg = ss.str();
//         client.Publish(topic_m, msg);
//     }

// private: 
//     struct vec_scaled_output acc_d, gyr_d, mag_d;
//     std::string topic_i;
//     std::string topic_m;
//     std::string msg;
//     evpp::EventLoop loop;
//     evnsq::Producer client;
//     //ros::Publisher imu_pub;
//     //ros::Publisher mag_pub;
//     //sensor_msgs::MagneticField mag_msg;
//     //sensor_msgs::Imu imu_msg;
// };


int main(int argc, char* argv[])
{
	std::string path = "../calib.txt";
	if (init_bmi270(1, path) != 0)
    {
        printf("Node failed to start\n");
        return 1;
    }
    
    BMI270 bmi(argv[1]);
	    
    
	//evpp::EventLoop loop;
	//evnsq::Producer client(&loop, evnsq::Option());
	
    
    
	
    //ros::Timer timer_imu = n.createTimer(ros::Duration(1./100.), std::bind(&BMI270::imu_reading, bmi));
    //ros::Timer timer_mag = n.createTimer(ros::Duration(1./30.), std::bind(&BMI270::mag_reading, bmi));

    return 0;
}


