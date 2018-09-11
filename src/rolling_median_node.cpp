
/*
 * humanoid_state_estimation - a complete state estimation scheme for humanoid robots
 *
 * Copyright 2017-2018 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *	 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */




#include <ros/ros.h>
#include "rolling_median/mediator.h"
#include <std_msgs/Int32.h>
#include <iostream>

using namespace std;

class rollingMedianFilter
{
  private:
	  int N;
	  std::string it, ot;
	  Mediator* m;
	  ros::NodeHandle n;
	  ros::Publisher pub;
	  ros::Subscriber sub;
	  std_msgs::Int32 pub_msg;
  
  public:
	  void  connect(const ros::NodeHandle nh){
	 	  n = nh;
		  ros::NodeHandle n_p("~");
		  n_p.param<int>("N",N,5);
		  n_p.param<std::string>("input_topic",it,"force_in");
		  n_p.param<std::string>("output_topic",ot,"force_out");
		  m = MediatorNew(N);
		  pub = n.advertise<std_msgs::Int32>(ot,1000);
		  sub = n.subscribe(it,1000,&rollingMedianFilter::filterCb,this);	
	  }
	  void filterCb(const std_msgs::Int32::ConstPtr& msg){

		   pub_msg = *msg;
		   MediatorInsert(m,pub_msg.data);

		   pub_msg.data=MediatorMedian(m);
		   pub.publish(pub_msg);


	  }

	  void run()
	  {
		ros::spin();

	  }

};




int main(int argc, char* argv[])
{



    ros::init(argc, argv, "rolling_median_node");
    ros::NodeHandle n;
    if(!ros::master::check())
    {
        cerr<<"Could not contact master!\nQuitting... "<<endl;
        return -1;
    }



    rollingMedianFilter* rmf = new rollingMedianFilter();
    rmf->connect(n);
    rmf->run();
    delete rmf;
     


}
