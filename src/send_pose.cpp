#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <sstream>
#include <regex>

//libcoap include
#include "send_pose/libcantcoap.h"


/// cantcoap client
#include <sys/types.h>
#include <sys/socket.h>
#define __USE_POSIX 1
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <math.h>

/// cantcoap lib
#include "send_pose/cantcoap.h"
#include "send_pose/nethelper.cc"

// // for linux command
// #include <cstdlib>

// rapidjson 
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"


int messageId = 0;
int error_counter = 0;
int sockfd_global = 0;
int global_ret = 0;

rapidjson::StringBuffer createJson(std::string mac_address, double robot_x, double robot_y); 

void failGracefully(int x) {
	exit(x);
}


int sendCoap(double robot_x, double robot_y, std::string mac_address, int sockfd) {

	ROS_ERROR("%d", sockfd);
	
	// repare resultData for getting json
	rapidjson::StringBuffer resultData;
	//remove /n in the last array
	mac_address.pop_back();
	// create json-styled string based on data
	resultData = createJson(mac_address, robot_x, robot_y);

	char *payload = (char *)resultData.GetString();
	
	std::cout << "payload is " << payload << std::endl;
	std::cout << "payload len: " << strlen(payload) << std::endl;

	// // using an external buffer for memory 
	// // uint8_t *exbuffer[100]; // 
	// // CoapPDU *pdu = new CoapPDU((uint8_t*)exbuffer, 100.0);

	// // construct CoAP packet
	CoapPDU *pdu = new CoapPDU();
	pdu->setVersion(1);
	pdu->setType(CoapPDU::COAP_CONFIRMABLE);
	pdu->setCode(CoapPDU::COAP_POST);
	// pdu->setToken((uint8_t*)"\3\2\1\1",4);
	pdu->setMessageID(messageId++);
	pdu->setURI((char*)"storage", 7);
	pdu->setContentFormat(CoapPDU::COAP_CONTENT_FORMAT_APP_JSON);
	// payload should be the last
	pdu->setPayload((uint8_t*)payload, strlen(payload));  //or pdu->getPayloadLength
	
	// pdu->addOption(CoapPDU::COAP_OPTION_CONTENT_FORMAT,1,(uint8_t*)")");
	// pdu->addOption(CoapPDU::COAP_CONTENT_FORMAT_APP_JSON, 1, (uint8_t*)"");
	std::cout << pdu->getPayloadLength() << std::endl;

	// // send packet
	int ret = send(sockfd, pdu->getPDUPointer(), pdu->getPDULength(),0);
	if(ret!=pdu->getPDULength()) {
		ROS_ERROR("Error sending packet.");
		perror(NULL);
		return -1;
	}
	ROS_INFO("Packet sent");

	// receive packet
	char buffer[128];
	
	ROS_INFO("before recv()");

	ret = recv(sockfd, &buffer, 128, MSG_DONTWAIT);
	// //ret = recvfrom(sockfd, &buffer, 128, 0, (struct sockaddr *)&fromAddr, &fromAddrLen);
	if(ret == -1) {
		ROS_ERROR("Error receiving data");
		return -1;
	}

	ROS_INFO("after recv()");

	// validate packet	
	CoapPDU *recvPDU = new CoapPDU((uint8_t*)buffer,ret);
	if(recvPDU->validate()!=1) {
		INFO("Malformed CoAP packet");
		return -1;
	}
	INFO("Valid CoAP PDU received");
	recvPDU->printHuman();
	
	return 0;
}


int initializeClientCoap() {

	char *listenAddressString = (char *)"192.168.10.20";
	char *listenPortString    = (char *)"5683";
	char *remoteAddressString = (char *)"192.168.10.10"; 
	char *remotePortString    = (char *)"5683";

	// structure for getting address of incoming packets
    // sockaddr_in fromAddr;
    // socklen_t fromAddrLen = sizeof(struct sockaddr_in);
	// memset(&fromAddr,0x00,fromAddrLen);

	// setup bind address
	struct addrinfo *bindAddr;
	INFO("Setting up bind address");
	int ret = setupAddress(listenAddressString, listenPortString, &bindAddr, SOCK_DGRAM,AF_INET);
	if(ret!=0) {
		INFO("Error setting up bind address, exiting.");
		return -1;
	}

	// iterate through returned structure to see what we got
	printAddressStructures(bindAddr);

	int option = 1;
	// setup socket
	int sockfd = socket(bindAddr->ai_family,bindAddr->ai_socktype, bindAddr->ai_protocol);
	// socket설정 SO_REUSEADDR 로 allow reuse of local addresses
	// setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option) );  //SO_REUSEADDR  // SO_RCVTIMEO
	sockfd_global = sockfd; //global variable
	
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option))) {
		ROS_ERROR("setsockopt error");
		error_counter++;
		if(error_counter > 10) {
			ROS_ERROR("It seems that server does not response.. Node will be terminated!");
			ros::shutdown();
		}
		return 1;
	}

	// call bind
	DBG("Binding socket.");
	if(bind(sockfd,bindAddr->ai_addr,bindAddr->ai_addrlen)!=0) {
		DBG("Error binding socket");
		perror(NULL);
		failGracefully(5);
	}
	
	//
	printAddress(bindAddr);


	struct addrinfo *remoteAddress;
	ret = setupAddress(remoteAddressString,remotePortString,&remoteAddress,SOCK_DGRAM,AF_INET);
	if(ret!=0) {
		INFO("Error setting up remote address, exiting.");
		return -1;
	}

	ROS_ERROR("global ret");
	ROS_ERROR("%d", ret);

	// call connect to associate remote address with socket
	ret = connect(sockfd,remoteAddress->ai_addr,remoteAddress->ai_addrlen);
	if(ret!=0) {
		INFO("Error: %s.",gai_strerror(ret));
		INFO("Error connecting to remote host.");
		return -1;
	}
	printAddress(remoteAddress);

	// // create payload data
	// std::string data = "{\"MacAddress\": \"$mac\", \"x\": \"$x\", \"y\": \"$y\"}";

	// //last character remove. (\n removed)
	// mac_address.pop_back();
	// // regex 후 replace 변환, double은 string으로 
	// data = std::regex_replace(data, std::regex("\\$x"), std::to_string(robot_x));
	// data = std::regex_replace(data, std::regex("\\$y"), std::to_string(robot_y));
	// data = std::regex_replace(data, std::regex("\\$mac"), mac_address.c_str());
	
	// std::cout << data << std::endl;
	// std::cout << "data len: " << data.length() << std::endl;
	return 0;
}


class SendPose {
private:
    ros::Subscriber odom_sub;
    ros::Publisher pose_pub;
    geometry_msgs::PoseStamped *pubPosePtr = nullptr;
	//ros::Timer timer = nh->createTimer(ros::Duration(1/10), timerCallback);
	std::string macAddress;

protected:
	int sockfd = 0;

	int getSockfd() {
		return this->sockfd;
	}

	void putSockfd(int sockfd_value) {
		this->sockfd = sockfd_value;
	}

public:
    SendPose(ros::NodeHandle *nh) {  //handler 포인터로 받기 
        odom_sub = nh->subscribe<nav_msgs::Odometry>("/odom", 10, &SendPose::odomCb, this);
        pose_pub = nh->advertise<geometry_msgs::PoseStamped>("/send_pose", 10);
		// get mac address 
		this->macAddress = this->getMacAddress();
		// std::cout << macAddress << std::endl; //for test
		initializeClientCoap();
        this->publishPose();
	}

    void odomCb(const nav_msgs::Odometry::ConstPtr & msg) {
        geometry_msgs::PoseStamped poseData;   
        poseData.header.frame_id = "odom";
        poseData.header.seq++;
        poseData.header.stamp = ros::Time::now();

        poseData.pose.position.x = msg->pose.pose.position.x;
        poseData.pose.position.y = msg->pose.pose.position.y;
        poseData.pose.position.z = msg->pose.pose.position.z;

        poseData.pose.orientation.x = 0.0;
        poseData.pose.orientation.y = 0.0;
        poseData.pose.orientation.z = msg->pose.pose.orientation.z;
        poseData.pose.orientation.w = msg->pose.pose.orientation.w;

        this->pubPosePtr = &poseData;
        
        /// print out
        // std::cout << "header" << std::endl;
        // std::cout << "\tframe_id: " << poseData.header.frame_id << std::endl;
        // std::cout << "\tseq: " << poseData.header.seq << std::endl;
        // std::cout << "\tstamp: " << poseData.header.stamp << std::endl;

        // std::cout << "pose" << std::endl;
        // std::cout << "\tx:" << poseData.pose.position.x << std::endl;
        // std::cout << "\ty:" << poseData.pose.position.y << std::endl;
        // std::cout << "\tz:" << poseData.pose.position.z << std::endl;

        // std::cout << "orientation" << std::endl;
        // std::cout << "\tx:" << poseData.pose.orientation.x << std::endl;
        // std::cout << "\ty:" << poseData.pose.orientation.y << std::endl;
        // std::cout << "\tz:" << poseData.pose.orientation.z << std::endl;
        // std::cout << "\tw:" << poseData.pose.orientation.w << std::endl;
        this->publishPose();
    }

    void publishPose() {
        
        ros::Rate loop_rate(0.5);
		int counti = 0;
        while(ros::ok()) {
            this->pose_pub.publish(*this->pubPosePtr);
            
			// pubPosePtr == nullptr 체크 pubPosePtr에 먼저 접근에서 에러 발생 방지
			if (!pubPosePtr) {
				ROS_INFO("Wait for a second, please. It will be soon continued..");
			} else {
				ROS_INFO("VALID");
				sendCoap(pubPosePtr->pose.position.x, pubPosePtr->pose.position.y, this->macAddress, sockfd_global);
			}
			ros::spinOnce();	
			loop_rate.sleep();
        }
        
    }
	
	/// function that returns a mac address 
	std::string getMacAddress() {
		std::string get_mac_cmd = "cat /sys/class/net/eno1/address";
		// print
		//  std::system(get_mac_cmd.c_str());

		FILE *fp;
		char var[20];

		fp = popen(get_mac_cmd.c_str(), "r");
		while (fgets(var, sizeof(var), fp) != NULL) {
			// printf("%s", var);
		}
		pclose(fp);
		
		std::string result = var;
		return result;
	}

};

/// creation json function ///
rapidjson::StringBuffer createJson(std::string mac_address, double robot_x, double robot_y) {
	rapidjson::StringBuffer strbuf;
	rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);

	writer.StartObject();
		writer.Key("MacAddress");
		writer.String(mac_address.c_str()); //to char
		writer.Key("x");
		writer.String((std::to_string(robot_x).c_str())); //double에서 string 변환 후 다시 char로 변환
		writer.Key("y");
		writer.String(std::to_string(robot_y).c_str());
	writer.EndObject();
	// std::cout << "in fuction: " << strbuf.GetString() << std::endl;
	return strbuf;
}

int main(int argc, char ** argv) {	
    ros::init(argc, argv, "odom_pose_pub");
    ros::NodeHandle nh;
    
	// test
	// clientCoap(30.0, 20.0);
    SendPose spObj(&nh);
    
    ros::MultiThreadedSpinner spinner(4);

    //ros::Timer timerForSub = nh.createTimer(ros::Duration(1.0 / 30.0), 
    //                                                &SendPose::odomCb, spObj);
    // ros::Timer timerForPub = nh.createTimer(ros::Duration(1.0 / 10.0), 
    //                                                 boost::bind(&SendPose::publishPose, spObj));
    
    spinner.spin();  //MultiThreadedSpinner를 사용할 때 (멀티스레드이지만 기능은 ros::spin()처럼 작동)
    //ros::spin();

	ros::shutdown();
	return 0;
}

