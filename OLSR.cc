/***************************************************************************
 *   Copyright (C) 2004 by Francisco J. Ros                                *
 *   fjrm@dif.um.es                                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

///
/// \file	OLSR.cc
/// \brief	Implementation of OLSR agent and related classes.
///
/// This is the main file of this software because %OLSR's behavior is
/// implemented here.
///

#include <olsr/OLSR.h>
#include <olsr/OLSR_pkt.h>
//#include <olsr/OLSR_printer.h>
#include <math.h>
#include <limits.h>
//#include <address.h>
//#include <ip.h>
//#include <cmu-trace.h>
#include <map>


/// header for interact with mac
#include "adhoc/netform_para.h"
#include "adhoc/transmit.h"
#include "adhoc/netform.h"
#include "adhoc/adhoc_timer.h"
#include <adhoc/netform_header.h>

	// Define SNR by xyy
	int SNR [20][20]/////网络初始建立时间
				 =  {   //0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19
							{3,6,0,0,0,0,0,0,0,0, 0,	  0,  0,   0,  0,  3,   3,  3,  3,  3},//0
							{6,3,3,0,0,0,0,0,0,0, 0,   0,  0,   0,  8,  3,   3,  3,  3,  3},//1
							{0,3,3,3,0,0,0,0,0,0, 0,   0,  0,   8,  3,  6,   0,  0,  0,  0},//2
							{0,0,3,3,3,0,0,0,0,0, 0,   0,  8,   3,  6,  0,   0,  0,  0,  0},//3
							{0,0,0,3,3,3,0,0,0,0, 0,   8,  3,   6,  0,  0,   0,  0,  0,  0},//4
							{0,0,0,0,3,3,3,0,0,0, 8,   3,  6,   0,  0,  0,   0,  0,  0,  0},//5
							{0,0,0,0,0,3,3,3,0,8, 3,   6,  0,   0,  0,  0,   0,  0,  0,  0},//6
							{0,0,0,0,0,0,3,3,3,3, 6,   0,  0,   0,  0,  0,   0,  0,  0,  0},//7
							{0,0,0,0,0,0,0,3,3,3, 0,   0,  0,   0,  0,  0,   0,  0,  0,  0},//8
							{0,0,0,0,0,0,8,3,3,3, 3,   0,  0,   0,  0,  0,   0,  0,  0,  0},//9
							{0,0,0,0,0,8,3,6,0,3, 3,   3,  0,   0,  0,  0,   0,  0,  0,  0},//10
							{0,0,0,0,8,3,6,0,0,0, 3,   3,  3,   0,  0,  0,   0,  0,  0,  0},//11
							{0,0,0,8,3,6,0,0,0,0, 0,   3,  3,   3,  0,  0,   0,  0,  0,  0},//12
							{0,0,8,3,6,0,0,0,0,0, 0,   0,  3,   3,  3,  0,   0,  0,  0,  0},//13
							{0,8,3,6,0,0,0,0,0,0, 0,   0,  0,   3,  3,  3,   0,  0,  0,  0},//14
							{3,3,6,0,0,0,0,0,0,0, 0,   0,  0,   0,  3,  3,   0,  0,  0,  0},//15
							{3,3,0,0,0,0,0,0,0,0, 0,   0,  0,   0,  0,  0,   3,  0,  0,  0},//16
							{3,3,0,0,0,0,0,0,0,0, 0,   0,  0,   0,  0,  0,   0,  3,  0,  0},//17
							{3,3,0,0,0,0,0,0,0,0, 0,   0,  0,   0,  0,  0,   0,  0,  3,  0},//18
							{3,3,0,0,0,0,0,0,0,0, 0,   0,  0,   0,  0,  0,   0,  0,  0,  3}//19
									 };
	//			=  {   //1,2, 3,4,5,6,7,8,9,10,13,12,13,14,15,16,17,18,19,20
	//				{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1},//1
	//	            {1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1},//2
	//				{0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0},//3
	//				{0,0,1,1,1,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0},//4
	//				{0,0,0,1,1,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0},//5
	//				{0,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0},//6
	//				{0,0,0,0,0,1,1,1,0,1,1,1,0,0,0,0,0,0,0,0},//7
	//				{0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0},//8
	//				{0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0},//9
	//				{0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0},//10
	//				{0,0,0,0,0,1,1,1,0,1,1,1,0,0,0,0,0,0,0,0},//11
	//				{0,0,0,0,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0},//12
	//				{0,0,0,1,1,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0},//13
	//				{0,0,1,1,1,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0},//14
	//				{0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0},//15
	//				{1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0},//16
	//				{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},//17
	//				{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},//18
	//				{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},//19
	//				{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}//20
	//	                         };

/// Length (in bytes) of UDP header.
#define UDP_HDR_LEN	8


///
/// \brief Function called by MAC layer when cannot deliver a packet.
///
/// \param p Packet which couldn't be delivered.
/// \param arg OLSR agent passed for a callback.
///

///added by li



/// For quick routing , the number of node complete
//static int node_num = 0 ;
/// whether the quick routing is complete
//static bool route_complete;
/// In quick routing phase,
static float Olsr_neighbor_hold_time;
static float Olsr_top_hold_time;

//static int address_number=0;


// if true, routing protocol is in the quick route
//static bool  quick_route;

//static void
//olsr_mac_failed_callback(Packet *p, void *arg) {
//  ((OLSR*)arg)->mac_failed(p);
//}

/********** TCL Hooks **********/

//int OLSR_pkt::offset_;
//static class OLSRHeaderClass : public PacketHeaderClass {
//public:
//	OLSRHeaderClass() : PacketHeaderClass("PacketHeader/OLSR", sizeof(OLSR_pkt)) {
//		bind_offset(&OLSR_pkt::offset_);
//	}
//} class_rtProtoOLSR_hdr;

//static class OLSRClass : public TclClass {
//public:
//	OLSRClass() : TclClass("Agent/OLSR") {}
//	TclObject* create(int argc, const char*const* argv) {
//		// argv has the following structure:
//		// <tcl-object> <tcl-object> Agent/OLSR create-shadow <id>
//		// e.g: _o17 _o17 Agent/OLSR create-shadow 0
//		// argv[4] is the address of the node
//		assert(argc == 5);
//		return new OLSR((nsaddr_t)Address::instance().str2addr(argv[4]));
//	}
//} class_rtProtoOLSR;

///
/// \brief Interface with TCL interpreter.
///
/// From your TCL scripts or shell you can invoke commands on this OLSR
/// routing agent thanks to this function. Currently you can call "start",
/// "print_rtable", "print_linkset", "print_nbset", "print_nb2hopset",
/// "print_mprset", "print_mprselset" and "print_topologyset" commands.
///
/// \param argc Number of arguments.
/// \param argv Arguments.
/// \return TCL_OK or TCL_ERROR.
///
//int
//OLSR::command(int argc, const char*const* argv) {
//	if (argc == 2) {
//		// Starts all timers
//		if (strcasecmp(argv[1], "start") == 0) {
//			hello_timer_.resched(0.0);
//			tc_timer_.resched(0.0);
//	//		mid_timer_.resched(0.0);
//
//			return TCL_OK;
//
//    		}
//		// Prints routing table
//		else if (strcasecmp(argv[1], "print_rtable") == 0) {
//			if (logtarget_ != NULL) {
//				sprintf(logtarget_->pt_->buffer(), "P %f _ Routing Table",
//					CURRENT_TIME);
//				logtarget_->pt_->dump();
//				rtable_.print(logtarget_);
//			}
//			else {
//				fprintf(stdout, "%f __ If you want to print this routing table "
//					"you must create a trace file in your tcl script",
//					CURRENT_TIME);
//			}
//			return TCL_OK;
//		}
//		// Prints link set
//		else if (strcasecmp(argv[1], "print_linkset") == 0) {
//			if (logtarget_ != NULL) {
//				sprintf(logtarget_->pt_->buffer(), "P %f __ Link Set",
//					CURRENT_TIME);
//				logtarget_->pt_->dump();
//				OLSR_printer::print_linkset(logtarget_, linkset());
//			}
//			else {
//				fprintf(stdout, "%f __ If you want to print this link set "
//					"you must create a trace file in your tcl script",
//					CURRENT_TIME);
//			}
//			return TCL_OK;
//		}
//		// Prints neighbor set
//		else if (strcasecmp(argv[1], "print_nbset") == 0) {
//			if (logtarget_ != NULL) {
//				sprintf(logtarget_->pt_->buffer(), "P %f __ Neighbor Set",
//					CURRENT_TIME);
//				logtarget_->pt_->dump();
//				OLSR_printer::print_nbset(logtarget_, nbset());
//			}
//			else {
//				fprintf(stdout, "%f __ If you want to print this neighbor set "
//					"you must create a trace file in your tcl script",
//					CURRENT_TIME);
//			}
//			return TCL_OK;
//		}
//		// Prints 2-hop neighbor set
//		else if (strcasecmp(argv[1], "print_nb2hopset") == 0) {
//			if (logtarget_ != NULL) {
//				sprintf(logtarget_->pt_->buffer(), "P %f __ Neighbor2hop Set",
//					CURRENT_TIME);
//				logtarget_->pt_->dump();
//				OLSR_printer::print_nb2hopset(logtarget_, nb2hopset());
//			}
//			else {
//				fprintf(stdout, "%f __ If you want to print this neighbor2hop set "
//					"you must create a trace file in your tcl script",
//					CURRENT_TIME);
//			}
//			return TCL_OK;
//		}
//		// Prints MPR set
//		else if (strcasecmp(argv[1], "print_mprset") == 0) {
//			if (logtarget_ != NULL) {
//				sprintf(logtarget_->pt_->buffer(), "P %f __ MPR Set",
//					CURRENT_TIME);
//				logtarget_->pt_->dump();
//				OLSR_printer::print_mprset(logtarget_, mprset());
//			}
//			else {
//				fprintf(stdout, "%f __ If you want to print this mpr set "
//					"you must create a trace file in your tcl script",
//					CURRENT_TIME);
//			}
//			return TCL_OK;
//		}
//		// Prints MPR selector set
//		else if (strcasecmp(argv[1], "print_mprselset") == 0) {
//			if (logtarget_ != NULL) {
//				sprintf(logtarget_->pt_->buffer(), "P %f __ MPR Selector Set",
//					CURRENT_TIME);
//				logtarget_->pt_->dump();
//				OLSR_printer::print_mprselset(logtarget_, mprselset());
//			}
//			else {
//				fprintf(stdout, "%f __ If you want to print this mpr selector set "
//					"you must create a trace file in your tcl script",
//					CURRENT_TIME);
//			}
//			return TCL_OK;
//		}
//		// Prints topology set
//		else if (strcasecmp(argv[1], "print_topologyset") == 0) {
//			if (logtarget_ != NULL) {
//				sprintf(logtarget_->pt_->buffer(), "P %f __ Topology Set",
//					CURRENT_TIME);
//				logtarget_->pt_->dump();
//				OLSR_printer::print_topologyset(logtarget_, topologyset());
//			}
//			else {
//				fprintf(stdout, "%f __ If you want to print this topology set "
//					"you must create a trace file in your tcl script",
//					CURRENT_TIME);
//			}
//			return TCL_OK;
//		}
//	}
//	else if (argc == 3) {
//		// Obtains the corresponding dmux to carry packets to upper layers
//		if (strcmp(argv[1], "port-dmux") == 0) {
//    			dmux_ = (PortClassifier*)TclObject::lookup(argv[2]);
//			if (dmux_ == NULL) {
//				fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1], argv[2]);
//				return TCL_ERROR;
//			}
//			return TCL_OK;
//    		}
//		// Obtains the corresponding tracer
//		else if (strcmp(argv[1], "log-target") == 0 || strcmp(argv[1], "tracetarget") == 0) {
//			logtarget_ = (Trace*)TclObject::lookup(argv[2]);
//			if (logtarget_ == NULL)
//				return TCL_ERROR;
//			return TCL_OK;
//		}
//	}
//	// Pass the command up to the base class
//	return Agent::command(argc, argv);
//}


/********** Timers **********///NSp75
void OLSRTimer::start(double time){//定义OLSRTimer类中的一个函数start，它是形参是time，为double类型
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSRTimer::start\n", CURRENT_TIME );
	Scheduler &s = Scheduler::instance();//创建一个调度器类的实例
	assert(busy_ == false);//如果assert()条件中为0，则函数以下的内容都不执行
	busy_ = true;
	paused_ = false;
	stime_ = s.clock();//调度器的虚拟时间
	rtime_ = time; //remaining time
	assert(rtime_ >= 0.0);
	s.schedule(this, &intr_, rtime_);//调度
//	void 	Scheduler::schedule(Handler* h, Event* e, double delay)	{
//		if (!h) {		};		//如果句柄未被设置，错误用例，退出
//		if (e->uid_ > 0) {	}		//事件id出错，退出
//		if (delay < 0) {		}
//		if (uid_ < 0) {		}			//调度队列已满，此调度未能满足
//		//如果以上均通过
//		e->uid_ = uid_++;	//赋给此调度事件一个id号，同事更新调度队列存储的事件数
//		e->handler_ = h;
//		double t = clock_ + delay;	//保存触发事件的时间：当前事件与要延时执行的事件和
//		e->time_ = t;		//指定事件的触发时间
//		insert(e);			//将此调度事件插入调度队列
//	}

}
//Stop Event
void OLSRTimer::stop(){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSRTimer::stop\n", CURRENT_TIME );
	Scheduler &s = Scheduler::instance();
	assert(busy_ == true);
	if (paused_ == false)
		s.cancel(&intr_);
	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
}
//Pause Event
void OLSRTimer::pause(void){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSRTimer::pause\n", CURRENT_TIME );
	Scheduler &s = Scheduler::instance();
	assert(busy_==true && paused_==false);
	busy_ = true;
	paused_ = true;
	rtime_ = rtime_ - (s.clock() - stime_);
	assert(rtime_ >= 0.0);
	s.cancel(&intr_);
}
//Continue Event
void OLSRTimer::resume(){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSRTimer::resume\n", CURRENT_TIME );
	Scheduler &s = Scheduler::instance();
	assert(busy_==true && paused_==true);
	paused_ = false;
	stime_ = s.clock();
	assert(rtime_ >= 0.0);
	s.schedule(this, &intr_, rtime_);
}
//==============olsr定时器=================
void OLSR_MsgTimer::handle(Event *e){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR_MsgTimer::handle\n", CURRENT_TIME );
	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
	//olsr_->expire(olsrmsgtimertimerID);
	olsr_->send_pkt();
}
void OLSR_HelloTimer::handle(Event *e){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR_HelloTimer::handle\n", CURRENT_TIME );
	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
	//olsr_->expire(olsrhellotimertimerID);
	olsr_->send_hello();//调用指针olsr_下的函数send_hello()
    olsr_->set_hello_timer();
}
void OLSR_TcTimer::handle(Event *e){
	fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR_TcTimer::handle\n", CURRENT_TIME );

	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
	//olsr_->expire(olsrtctimertimerID);
	fprintf(stdout, "  mprselect size %d\n",olsr_->mprselset().size());
	if (olsr_->mprselset().size() > 0)//timer_->olsr_tc_timer_->agent_->
		olsr_->send_tc();//timer_->olsr_tc_timer_->agent_->
	olsr_->set_tc_timer();
}
void OLSR_DupTupleTimer::handle(Event *e){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR_DupTupleTimer::handle\n", CURRENT_TIME );
	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
	//olsr_->expire(olsrduptupletimertimerID);
	 if (tuple_->time() < CURRENT_TIME) { //tuple_->time() 指的是什么
		olsr_->rm_dup_tuple(tuple_);/// Time at which this tuple expires and must be removed.
		delete tuple_;
		delete this;
	}
	else
   		fprintf (stdout ,"  DELAY(tuple_->time()) = %f 1111111111111111111111\n",DELAY(tuple_->time()));
		start(DELAY(tuple_->time()));
}
void OLSR_LinkTupleTimer::handle(Event *e){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR_LinkTupleTimer::handle\n", CURRENT_TIME );
	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
	//olsr_->expire(olsrlinktupletimertimerID);
	double now	= CURRENT_TIME;

	if (tuple_->time() < now) {
		olsr_->rm_link_tuple(tuple_);
		delete tuple_;
		delete this;
	}
	else if (tuple_->sym_time() < now) {
		if (first_time_)
			first_time_ = false;
		else
			olsr_->nb_loss(tuple_);
   		   fprintf (stdout ,"  DELAY(tuple_->time()) = %f2222222222222222222222222\n",DELAY(tuple_->time()));
			start(DELAY(tuple_->time()));
	}
	else{
   		fprintf (stdout ,"  DELAY(MIN(tuple_->time(), tuple_->sym_time())) = %f33333333333333333333333333\n",DELAY(MIN(tuple_->time(), tuple_->sym_time())));
		start(DELAY(MIN(tuple_->time(), tuple_->sym_time())));
	}
}
void OLSR_Nb2hopTupleTimer::handle(Event *e){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR_Nb2hopTupleTimer::handle\n", CURRENT_TIME );
	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
//	olsr_->expire(olsrnb2hoptupletimertimerID);
	if (tuple_->time() < CURRENT_TIME) {
			olsr_->rm_nb2hop_tuple(tuple_);
			delete this;
			//delete timer_->olsr_nb2hoptuple_timer_->olsr_;
		}
		else{
	   		 fprintf (stdout ,"  DELAY(tuple_->time()) = %f44444444444444444444444444\n",DELAY(tuple_->time()));
			  start(DELAY(tuple_->time()));
		}
}
void OLSR_MprSelTupleTimer::handle(Event *e){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR_MprSelTupleTimer::handle\n", CURRENT_TIME );
	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
//	olsr_->expire(olsrmprseltupletimertimerID);
	if (tuple_->time() < CURRENT_TIME) {
		olsr_->rm_mprsel_tuple(tuple_);
		delete tuple_;
		delete this;
	}
	else{
   		 fprintf (stdout ,"  DELAY(tuple_->time()) = %f5555555555555555555555555555\n",DELAY(tuple_->time()));
		 start(DELAY(tuple_->time()));
	}
}
void OLSR_TopologyTupleTimer::handle(Event *e){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR_TopologyTupleTimer::handle\n", CURRENT_TIME );
	busy_ = false;
	paused_ = false;
	stime_ = 0.0;
	rtime_ = 0.0;
//	olsr_->expire(olsrtopologytupletimertimerID);
	 if (tuple_->time() < CURRENT_TIME) {
		olsr_->rm_topology_tuple(tuple_);
		delete tuple_;
		delete this;
	}
	else{
   			 fprintf (stdout ,"  DELAY(tuple_->time()) = %f6666666666666666666666666\n",DELAY(tuple_->time()));
			 start(DELAY(tuple_->time()));
		}
}
void OLSR::expire(OLSRTimerID id){
    fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR::expire\n", CURRENT_TIME );
	switch (id)
		{
		   case olsrmsgtimertimerID :
		   {
			   //timer_->olsr_msg_timer_->agent_->send_pkt();
			  send_pkt();
		   }
		   break;
		   case olsrhellotimertimerID :
		   {
				//timer_->olsr_hello_timer_->agent_->send_hello();
			    send_hello();
			    set_hello_timer();
		   }
		   break;
		   case olsrtctimertimerID:
		   {
				if (mprselset().size() > 0)//timer_->olsr_tc_timer_->agent_->
					send_tc();//timer_->olsr_tc_timer_->agent_->
				set_tc_timer();//timer_->olsr_tc_timer_->agent_->
		   }
		   break;
		   case olsrduptupletimertimerID:
		   {
			   if (timer_->olsr_duptuple_timer_->tuple_->time() < CURRENT_TIME) {
					rm_dup_tuple(timer_->olsr_duptuple_timer_->tuple_);
					delete timer_->olsr_duptuple_timer_->tuple_;
					//delete timer_->olsr_duptuple_timer_->olsr_;
				}
				else
			   //		fprintf (stdout ,"%f 1111111111111111111111\n",DELAY(tuple_->time()));
					timer_->olsr_duptuple_timer_->start(DELAY(timer_->olsr_duptuple_timer_->tuple_->time()));
		   }
		   break;
		   case olsrlinktupletimertimerID:
		   {
			   double now	= CURRENT_TIME;

			   	if (timer_->olsr_linktuple_timer_->tuple_->time() < now) {
			   		rm_link_tuple(timer_->olsr_linktuple_timer_->tuple_);
			   		delete timer_->olsr_linktuple_timer_->tuple_;
			   		//delete timer_->olsr_linktuple_timer_->olsr_;
			   	}
			   	else if (timer_->olsr_linktuple_timer_->tuple_->sym_time() < now) {
			   		if (timer_->olsr_linktuple_timer_->first_time_)
			   			timer_->olsr_linktuple_timer_->first_time_ = false;
			   		else
			   			nb_loss(timer_->olsr_linktuple_timer_->tuple_);
			   //		   fprintf (stdout ,"%f2222222222222222222222222\n",DELAY(tuple_->time()));
			   		    timer_->olsr_linktuple_timer_->start(DELAY(timer_->olsr_linktuple_timer_->tuple_->time()));
			   	}
			   	else{
			   //		fprintf (stdout ,"%f33333333333333333333333333\n",DELAY(MIN(tuple_->time(), tuple_->sym_time())));
			   		timer_->olsr_linktuple_timer_->start(DELAY(MIN(timer_->olsr_linktuple_timer_->tuple_->time(), timer_->olsr_linktuple_timer_->tuple_->sym_time())));
			   	}
		   }
		   break;
		   case olsrnb2hoptupletimertimerID:
		   {
			   if (timer_->olsr_nb2hoptuple_timer_->tuple_->time() < CURRENT_TIME) {
			   		rm_nb2hop_tuple(timer_->olsr_nb2hoptuple_timer_->tuple_);
			   		delete timer_->olsr_nb2hoptuple_timer_->tuple_;
			   		//delete timer_->olsr_nb2hoptuple_timer_->olsr_;
			   	}
			   	else{
			   //		 fprintf (stdout ,"%f44444444444444444444444444\n",DELAY(tuple_->time()));
			   		  timer_->olsr_nb2hoptuple_timer_->start(DELAY(timer_->olsr_nb2hoptuple_timer_->tuple_->time()));
			   	}
		   }
		   break;
		   case olsrmprseltupletimertimerID:
		   {
			   if (timer_->olsr_mprseltuple_timer_->tuple_->time() < CURRENT_TIME) {
			   		rm_mprsel_tuple(timer_->olsr_mprseltuple_timer_->tuple_);
			   		delete timer_->olsr_mprseltuple_timer_->tuple_;
			   		//delete timer_->olsr_mprseltuple_timer_->olsr_;
			   	}
			   	else{
			   //		 fprintf (stdout ,"%f5555555555555555555555555555\n",DELAY(tuple_->time()));
			   		 timer_->olsr_mprseltuple_timer_->start(DELAY(timer_->olsr_mprseltuple_timer_->tuple_->time()));
			   	}
		   }
		   break;
		   case olsrtopologytupletimertimerID:
		   {
			   if (timer_->olsr_topologytuple_timer_->tuple_->time() < CURRENT_TIME) {
			   		rm_topology_tuple(timer_->olsr_topologytuple_timer_->tuple_);
			   		delete timer_->olsr_topologytuple_timer_->tuple_;
			   		//delete timer_->olsr_topologytuple_timer_->olsr_;
			   	}
			   	else{
			   //			 fprintf (stdout ,"%f6666666666666666666666666\n",DELAY(tuple_->time()));
			   			 timer_->olsr_topologytuple_timer_->start(DELAY(timer_->olsr_topologytuple_timer_->tuple_->time()));
			   		}
		   }
		}
}

///
/// \brief Sends a HELLO message and reschedules the HELLO timer.
/// \param e The event which has expired.
///
//void
//OLSR_HelloTimer::expire(Event* e) {
//	agent_->send_hello();
//	agent_->set_hello_timer();
//}

///
/// \brief Sends a TC message (if there exists any MPR selector) and reschedules the TC timer.
/// \param e The event which has expired.
///
//void
//OLSR_TcTimer::expire(Event* e) {
////	fprintf (stdout , "mpr select size is %d \n",agent_->mprselset().size() );
//	if (agent_->mprselset().size() > 0)
//		agent_->send_tc();
//	agent_->set_tc_timer();
//}

///
/// \brief Sends a MID message (if the node has more than one interface) and resets the MID timer.
/// \warning Currently it does nothing because there is no support for multiple interfaces.
/// \param e The event which has expired.
///
//void
//OLSR_MidTimer::expire(Event* e) {
//#ifdef MULTIPLE_IFACES_SUPPORT
//	agent_->send_mid();
//	agent_->set_mid_timer();
//#endif
//}

///
/// \brief Removes tuple_ if expired. Else timer is rescheduled to expire at tuple_->time().
///
/// The task of actually removing the tuple is left to the OLSR agent.
///
/// \param e The event which has expired.
///
//void
//OLSR_DupTupleTimer::expire(Event* e) {
//	if (tuple_->time() < CURRENT_TIME) {
//		agent_->rm_dup_tuple(tuple_);
//		delete tuple_;
//		delete this;
//	}
//	else
////		fprintf (stdout ,"%f 1111111111111111111111\n",DELAY(tuple_->time()));
//		resched(DELAY(tuple_->time()));
//}

///
/// \brief Removes tuple_ if expired. Else if symmetric time
/// has expired then it is assumed a neighbor loss and agent_->nb_loss()
/// is called. In this case the timer is rescheduled to expire at
/// tuple_->time(). Otherwise the timer is rescheduled to expire at
/// the minimum between tuple_->time() and tuple_->sym_time().
///
/// The task of actually removing the tuple is left to the OLSR agent.
///
/// \param e The event which has expired.
///
//void
//OLSR_LinkTupleTimer::expire(Event* e) {
//	double now	= CURRENT_TIME;
//
//	if (tuple_->time() < now) {
//		agent_->rm_link_tuple(tuple_);
//		delete tuple_;
//		delete this;
//	}
//	else if (tuple_->sym_time() < now) {
//		if (first_time_)
//			first_time_ = false;
//		else
//			agent_->nb_loss(tuple_);
////		   fprintf (stdout ,"%f2222222222222222222222222\n",DELAY(tuple_->time()));
//		    resched(DELAY(tuple_->time()));
//	}
//	else{
////		fprintf (stdout ,"%f33333333333333333333333333\n",DELAY(MIN(tuple_->time(), tuple_->sym_time())));
//		resched(DELAY(MIN(tuple_->time(), tuple_->sym_time())));
//	}
//
//}

///
/// \brief Removes tuple_ if expired. Else the timer is rescheduled to expire at tuple_->time().
///
/// The task of actually removing the tuple is left to the OLSR agent.
///
/// \param e The event which has expired.
///
//void
//OLSR_Nb2hopTupleTimer::expire(Event* e) {
//	if (tuple_->time() < CURRENT_TIME) {
//		agent_->rm_nb2hop_tuple(tuple_);
//		delete tuple_;
//		delete this;
//	}
//	else{
////		 fprintf (stdout ,"%f44444444444444444444444444\n",DELAY(tuple_->time()));
//		  resched(DELAY(tuple_->time()));
//	}
//
//}

///
/// \brief Removes tuple_ if expired. Else the timer is rescheduled to expire at tuple_->time().
///
/// The task of actually removing the tuple is left to the OLSR agent.
///
/// \param e The event which has expired.
///
//void
//OLSR_MprSelTupleTimer::expire(Event* e) {
//	if (tuple_->time() < CURRENT_TIME) {
//		agent_->rm_mprsel_tuple(tuple_);
//		delete tuple_;
//		delete this;
//	}
//	else{
////		 fprintf (stdout ,"%f5555555555555555555555555555\n",DELAY(tuple_->time()));
//		 resched(DELAY(tuple_->time()));
//	}
//
//}

///
/// \brief Removes tuple_ if expired. Else the timer is rescheduled to expire at tuple_->time().
///
/// The task of actually removing the tuple is left to the OLSR agent.
///
/// \param e The event which has expired.
///
//void
//OLSR_TopologyTupleTimer::expire(Event* e) {
//	if (tuple_->time() < CURRENT_TIME) {
//		agent_->rm_topology_tuple(tuple_);
//		delete tuple_;
//		delete this;
//	}
//	else{
////			 fprintf (stdout ,"%f6666666666666666666666666\n",DELAY(tuple_->time()));
//			 resched(DELAY(tuple_->time()));
//		}
//}

///
/// \brief Removes tuple_ if expired. Else timer is rescheduled to expire at tuple_->time().
/// \warning Actually this is never invoked because there is no support for multiple interfaces.
/// \param e The event which has expired.
///
//void
//OLSR_IfaceAssocTupleTimer::expire(Event* e) {
//	if (tuple_->time() < CURRENT_TIME) {
//		agent_->rm_ifaceassoc_tuple(tuple_);
//		delete tuple_;
//		delete this;
//	}
//	else
//		resched(DELAY(tuple_->time()));
//}

///
/// \brief Sends a control packet which must bear every message in the OLSR agent's buffer.
///
/// The task of actually sending the packet is left to the OLSR agent.
///
/// \param e The event which has expired.
///
//void
//OLSR_MsgTimer::expire(Event* e) {
//	agent_->send_pkt();
//	delete this;
//}


/********** OLSR class **********/

///
/// \brief Creates necessary timers, binds TCL-available variables and do
/// some more initializations.
/// \param id Identifier for the OLSR agent. It will be used as the address
/// of this routing agent.
///Constructor构造函数
OLSR::OLSR(int id,Traf_Queue *route_queue_)
//,NetForm_Timer_Block *timer_block_,OLSR_dup_tuple*	route_tuple_
				//hello_timer_(this),
//				tc_timer_(this)
//			mid_timer_(this)
{
	fprintf (stdout, "CURRENT TIME = %f , STAGE = OLSR\n", CURRENT_TIME );

//	timer_block_ = new NetForm_Timer_Block();

	// Enable usage of some of the configuration variables from Tcl.
	//
	// Note: Do NOT change the values of these variables in the constructor
	// after binding them! The desired default values should be set in
	// ns-X.XX/tcl/lib/ns-default.tcl instead.
//	bind("willingness_", &willingness_);
//	bind("hello_ival_", &hello_ival_);
//	bind("tc_ival_", &tc_ival_);
//	bind("mid_ival_", &mid_ival_);
//	bind_bool("use_mac_", &use_mac_);


	// Do some initializations
	hello_ival_ = 2;
	tc_ival_ =5 ;
	ra_node_id_	= id;
	pkt_seq_	= OLSR_MAX_SEQ_NUM;
	msg_seq_	= OLSR_MAX_SEQ_NUM;
	ansn_		= OLSR_MAX_SEQ_NUM;
	end_quick_route_  =1;
	quick_route_ = false;

	queue_ = route_queue_;
	timer_ = new OLSR_Timer_Block();
	timer_->olsr_hello_timer_ = new OLSR_HelloTimer(this);
	timer_->olsr_tc_timer_ = new OLSR_TcTimer(this);
	fprintf (stdout,"  ");timer_->olsr_hello_timer_->start(0.0);
	fprintf (stdout,"  ");timer_->olsr_tc_timer_->start(0.0);

	temp_add_route.frame_type_ = Traf_Queue_Content::Defualt;

	for(int i = 0; i<BUFFER_SIZE; i++)
	{
		temp_add_route.route_frame_content_pt_.content_[i] = 0 ;
	}
}



//void
//OLSR::recv(RouteMacHeader* cp) {
//
//	   fprintf (stdout, "node[%d], current time %f receive the packet \n", ra_node_id_,CURRENT_TIME);

//	    struct hdr_cmn* ch	= HDR_CMN(p);
//	    struct RouteMacHeader * cp = RouteMac_HEADER(p);
//		struct hdr_ip* ih	= HDR_IP(p);

//		if (ih->saddr() == ra_addr()) {
//			// If there exists a loop, must drop the packet
//			if (ch->num_forwards() > 0) {
//				drop(p, DROP_RTR_ROUTE_LOOP);
//				return;
//			}
//			// else if this is a packet I am originating, must add IP header
//			else if (ch->num_forwards() == 0)
//				ch->size() += IP_HDR_LEN;
//		}

		// If it is an OLSR packet, must process it
//		if (cp->olsr_packet)
//			recv_olsr(p);
//			recv_process(cp);
			///added by li to mapping

		// Otherwise, must forward the packet (unless TTL has reached zero)
//		else {
//			ih->ttl_--;
//			if (ih->ttl_ == 0) {
//				drop(p, DROP_RTR_TTL);
//				return;
//			}
//			forward_data(p);
//		}

//}

// mapping by li
void OLSR::recv(Route_Mac_Block*  cp){
	fprintf (stdout, "CURRENT TIME = %f , STAGE = recv\n", CURRENT_TIME );

 //   struct hdr_cmn* ch	= HDR_CMN(p);
//	struct hdr_ip* ih	= HDR_IP(p);
	OLSR_pkt op;
//	struct RouteMacHeader * cp = RouteMac_HEADER(p);

/// Whether into the quick routing
	if (cp->real_content_flag_ == false ){//判断该包是否有内容，若为false，则只包含标志位
		//根据路由层与mac层之间信号类型来判断是否进入快速路由
			 if (cp->signal_type_ == mr_start_fast_route){
				 Olsr_neighbor_hold_time =20*0.3;
				 Olsr_top_hold_time =20*0.8;
				 hello_ival()  = 0.3;
				 tc_ival() = 0.8;
				 quick_route() = true;
				 online_node_num() =  cp->online_node_num_;
			 }
			 if (cp->signal_type_ == mr_netform_complete){
//				 route_complete = true;
				 Olsr_neighbor_hold_time = 3*2;
				 Olsr_top_hold_time =  3*5;
				 hello_ival()  = 2;
				 tc_ival()  = 5;
				 quick_route() = false;
			 }

		}
	else{//包内有信息，则提取信息
		op.count =1;
		op.pkt_seq_num_    = (cp->buffer_[0] & 0xF8 ) >> 3;
		OLSR_msg  msg ;
		msg.msg_type()    = (cp->buffer_[0] & 0x04 ) >> 2;

//		fprintf (stdout, "  pkt_seq_num = %x,\n  receive message type = %x \n", op.pkt_seq_num_,msg.msg_type());
//		fprintf (stdout, "  pkt_seq_num = %x\n", op.pkt_seq_num_);

		if( msg.msg_type() == 0)
			fprintf (stdout, "  receive message type = HELLO  \n");
		else if( msg.msg_type() == 1)
			fprintf (stdout, "  receive message type = TC \n");
//		else if( msg.msg_type() == 3)
//			fprintf (stdout, "  receive message type = RTC \n");
//		else if( msg.msg_type() == 4)
//			fprintf (stdout, "  receive message type = MID  \n");

		if (quick_route()){
			if (msg.msg_type() == OLSR_HELLO_MSG){
				 msg.vtime()		=  Olsr_neighbor_hold_time;
			}
			else{
				 msg.vtime()		=  Olsr_top_hold_time;
			}
		}

		else{
		msg.vtime()	   = ( (cp->buffer_[0]& 0x03) << 3)  +( (cp->buffer_[1] &  0xE0 ) >> 5);
//			fprintf (stdout, "recv vtime is %f \n", msg.vtime());
		}
//			fprintf (stdout, "  receive vtime = %f \n", msg.msg_type());

		msg.orig_node_id()	   =  cp->buffer_[1] & 0x1F ;
		msg.ttl()     = (cp->buffer_[2] & 0xF8 ) >> 3;
		msg.hop_count()     = ((cp->buffer_[2] & 0x07 ) << 2) + ((cp->buffer_[3] & 0xC0 ) >> 6);
		msg.msg_seq_num()      = (cp->buffer_[3] & 0x3E) >> 1 ;
		fprintf (stdout, "  msg.orig_node_id = %d, op.pkt_seq_num  = %x, msg.msg_seq_num = %x. \n",
							msg.orig_node_id(), op.pkt_seq_num() , msg.msg_seq_num());
//		fprintf (stdout, "  msg.orig_node_id()=%d, msg.ttl()=%x, msg.hop_count()=%x, msg.msg_seq_num()=%x\n",
//							msg.orig_node_id(), msg.ttl() , msg.hop_count(), msg.msg_seq_num());

		if (msg.msg_type() == OLSR_HELLO_MSG){

			  msg.hello().reserved()		=  0;
			  if (quick_route()){
				  msg.hello().htime()     = hello_ival_;
			  }
			  else{
				  msg.hello().htime()     =  ((cp->buffer_[3] & 0x01) << 7) + ((cp->buffer_[4] & 0xFE) >> 1) ;
			  }

			  msg.hello().willingness()	     = ((cp->buffer_[4] & 0x01) << 1) + ( (cp->buffer_[5] & 0x80 )>> 7);
//			  fprintf (stdout, "  HELLO: The willingness of node[%d] is %d \n",msg.orig_node_id(), msg.hello().willingness());
			  char nodeid[20];
			  //这里的逻辑与和移位是什么意图啊？——把buffer[ ]每个字节上存放的信息提取出来
			  nodeid[0]     = ( cp->buffer_[5] & 0x78 ) >> 3;
			  nodeid[1]     =( (cp-> buffer_[5] & 0x07 ) << 1) + ((cp->buffer_[6] & 0x80 )>> 7) ;
			  nodeid[2]     =  (cp-> buffer_[6] & 0x78 ) >> 3;
			  nodeid[3]     =( (cp-> buffer_[6] & 0x07 ) << 1) + ((cp->buffer_[7] & 0x80 )>> 7) ;
			  nodeid[4]     =  (cp-> buffer_[7] & 0x78 ) >> 3;
			  nodeid[5]     = (( cp->buffer_[7] & 0x07 ) << 1) + ((cp->buffer_[8] & 0x80 )>> 7) ;
			  nodeid[6]     =  (cp-> buffer_[8] & 0x78 ) >> 3;
			  nodeid[7]     =( (cp-> buffer_[8] & 0x07 ) << 1) + ((cp->buffer_[9] & 0x80 )>> 7) ;
			  nodeid[8]     =  (cp-> buffer_[9] & 0x78 ) >> 3;
			  nodeid[9]     =( (cp-> buffer_[9] & 0x07 ) << 1) + ((cp->buffer_[10] & 0x80 )>> 7) ;
			  nodeid[10]     =  (cp->buffer_[10] & 0x78 ) >> 3;
			  nodeid[11]     = ((cp-> buffer_[10] & 0x07 ) << 1) + ((cp->buffer_[11] & 0x80 )>> 7) ;
			  nodeid[12]     =  (cp-> buffer_[11] & 0x78 ) >> 3;
			  nodeid[13]     =( (cp-> buffer_[11] & 0x07 ) << 1) + ((cp->buffer_[12] & 0x80 )>> 7) ;
			  nodeid[14]     =  (cp-> buffer_[12] & 0x78 ) >> 3;
			  nodeid[15]     =( ( cp->buffer_[12] & 0x07 ) << 1) + ((cp->buffer_[13] & 0x80 )>> 7) ;
			  nodeid[16]     =  ( cp->buffer_[13] & 0x78 ) >> 3;
			  nodeid[17]     =( ( cp->buffer_[13] & 0x07 ) << 1) + ((cp->buffer_[14] & 0x80 )>> 7) ;
			  nodeid[18]     =  ( cp->buffer_[14] & 0x78 ) >> 3;
			  nodeid[19]     =( ( cp->buffer_[14] & 0x07 ) << 1) + ((cp->buffer_[15] & 0x80 )>> 7) ;

				 //added by xyy
			 msg.hello().saturation() =  ( cp->buffer_[15] & 0x60 ) >> 5;
			 msg.hello().choose_cd_node_id() =  ( cp->buffer_[15] & 0x1f );
			 fprintf (stdout, "  HELLO :orig_ node_id = %d, saturation level = %d, orig_cd_node_id = %d .\n",
					 msg.orig_node_id(), msg.hello().saturation(), msg.hello().choose_cd_node_id());

			 for( int i = 0; i<20; i++){
				 if ( i == msg.orig_node_id() )
					 msg.hello().mine_cd_node() = nodeid[i] & 0x01;
			 }


//				 //test
//				 if (  msg.orig_node_id() == 3 ||
//						 msg.orig_node_id() == 5 ||
//						 msg.orig_node_id() == 7 ||
//						 msg.orig_node_id() == 14 ||
//						 msg.orig_node_id() == 0 ||
//						 msg.orig_node_id() == 2   ){
//					 msg.hello().saturation() = 3;
//				 }
//				 else
//					 msg.hello().saturation() = 1;
//				 fprintf (stdout, "  HELLO :orig_ node_id = %d, saturation level = %d\n", msg.orig_node_id(), msg.hello().saturation());
//
//
//			      for (int i = 0; i < 20 ; i ++ ){
//			    		  if (nodeid[i] != 0){
//			    			  fprintf (stdout,"  HELLO:nodeid [%d]  is  %d \n", i,nodeid[i] );
//			    		  }
//			      }

			  //计算20个节点中nodeid[i] =?= 0x08/09/0A/0B/04/05/06/07/0C/0D/0E/0F的个数，意图是什么？
			  //为什么统计节点地址分别为4\5\6……15的节点个数？
			  int  count0,  count1 , count2 , count3,  count4 ,  count5 ,  count6 ,  count7,
			  count8 ,  count9,  count10 , count11;
			  count0 =  count1 =  count2 =  count3 =  count4 =  count5 =  count6 =  count7
					  =  count8 =  count9 =  count10 =  count11 = 0;
			  int sum = 0;

			  fprintf (stdout,"  HELLO : ");
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x08){//1000 : neighbor type = 2 = SYM_NEIGH; link type = 0 = UNSPEC_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count0)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count0 + 1);
						  count0 ++;
				  }
			  }
				if (count0 > 0)
				{
					msg.hello().hello_msg(sum).count = count0;
					sum ++;
					fprintf (stdout,"count0= %d , ", count0);
				}

			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x09){//1001 : neighbor type = 2 = SYM_NEIGH; link type = 1 = ASYM_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count1)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count1 + 1);
						  count1 ++;
				  }
			  }
				if (count1 > 0)
				{
					msg.hello().hello_msg(sum).count = count1;
					sum ++;
					fprintf (stdout,"count1= %d , ", count1);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x0A){//1010 : neighbor type = 2 = SYM_NEIGH; link type = 2 = SYM_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count2)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count2 + 1);
						  count2 ++;
				  }

			  }
				if (count2 > 0)
				{
					msg.hello().hello_msg(sum).count = count2;
					sum ++;
					fprintf (stdout,"count2= %d , ", count2);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x0B){//1011 : neighbor type = 2 = SYM_NEIGH; link type = 3 = LOST_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count3)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count3 + 1);
						  count3 ++;
				  }

			  }
				if (count3 > 0)
				{
					msg.hello().hello_msg(sum).count = count3;
					sum ++;
					fprintf (stdout,"count3= %d , ", count3);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x04){//0100 : neighbor type = 1 = NOT_NEIGH; link type = 0 = UNSPEC_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count4)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count4 + 1);
						  count4 ++;
				  }

			  }
				if (count4 > 0)
				{
					msg.hello().hello_msg(sum).count = count4;
					sum ++;
					fprintf (stdout,"count4= %d , ", count4);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x05){//0101 : neighbor type = 1 = NOT_NEIGH; link type = 1 = ASYM_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count5)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count5 + 1);
						  count5 ++;
				  }

			  }
				if (count5 > 0)
				{
					msg.hello().hello_msg(sum).count = count5;
					sum ++;
					fprintf (stdout,"count5= %d , ", count5);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x06){//0110 : neighbor type = 1 = NOT_NEIGH; link type = 2 = SYM_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count6)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count6 + 1);
						  count6 ++;
				  }

			  }
				if (count6 > 0)
				{
					msg.hello().hello_msg(sum).count = count6;
					sum ++;
					fprintf (stdout,"count6= %d , ", count6);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x07){//0111 : neighbor type = 1 = NOT_NEIGH; link type = 3 = LOST_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count7)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count7 + 1);
						  count7 ++;
				  }

			  }
				if (count7 > 0)
				{
					msg.hello().hello_msg(sum).count = count7;
					sum ++;
					fprintf (stdout,"count7= %d , ", count7);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x0C){//1100 : neighbor type = 3 = MPR_NEIGH; link type = 0 = UNSPEC_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count8)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count8 + 1);
						  count8 ++;
				  }

			  }
				if (count8 > 0)
				{
					msg.hello().hello_msg(sum).count = count8;
					sum ++;
					fprintf (stdout,"count8= %d , ", count8);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x0D){//1101 : neighbor type = 3 = MPR_NEIGH; link type = 1 = ASYM_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count9)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count9 + 1);
						  count9 ++;
				  }

			  }
				if (count9 > 0)
				{
					msg.hello().hello_msg(sum).count = count9;
					sum ++;
					fprintf (stdout,"count9= %d , ", count9);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x0E){//1110 : neighbor type = 3 = MPR_NEIGH; link type = 2 = SYM_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count10)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count10 + 1);
						  count10 ++;
				  }

			  }
				if (count10 > 0)
				{
					msg.hello().hello_msg(sum).count = count10;
					sum ++;
					fprintf (stdout,"count10= %d , ", count10);
				}
			  for  (int i = 0; i < 20 ; i ++ ){
				  if ( nodeid[i] == 0x0F){//1111 : neighbor type = 3 = MPR_NEIGH; link type = 3 = LOST_LINK
						  msg.hello().hello_msg(sum).link_code()	     =  nodeid[i] & 0x0F ;
						  msg.hello().hello_msg(sum).nb_main_node_id(count11)	     =  i ;
						  msg.hello().hello_msg(sum).link_msg_size()  = 4 + 4*(count11 + 1);
						  count11 ++;
				  }

			  }
			if (count11 > 0)
			{
				msg.hello().hello_msg(sum).count = count11;
				sum ++;
				fprintf (stdout,"count11= %d , ", count11);
			}

		  msg.hello().count= sum;
		  fprintf (stdout,"sum= %d .\n", sum);
//			  //PRINT BY XU
//			  fprintf (stdout,"  HELLO:count0= %d, count1= %d, count2= %d, count3= %d, count4= %d, count5= %d\n"
//						  "  HELLO:count6= %d, count7= %d, count8= %d, count9= %d, count10= %d, count11= %d, sum=%d\n",
//						  count0,  count1 , count2 , count3,  count4 ,  count5 ,  count6 ,  count7,  count8 ,  count9,  count10 , count11 , sum);

		  }

		  else if (msg.msg_type() ==  OLSR_TC_MSG){
		        msg.tc().ansn()  =  cp->buffer_[4] & 0xFF ;
		        msg.tc().reserved()	= 0;
		        char nodeid[20];
		        nodeid[0] =( cp-> buffer_[5] & 0x80) >> 7;
		        nodeid[1] =(  cp->buffer_[5] & 0x40) >> 6;
		        nodeid[2] =(  cp->buffer_[5] & 0x20) >> 5;
		        nodeid[3] =( cp-> buffer_[5] & 0x10) >> 4;
		        nodeid[4] =( cp-> buffer_[5] & 0x08) >> 3;
		        nodeid[5] =(  cp->buffer_[5] & 0x04) >> 2;
		        nodeid[6] =( cp-> buffer_[5] & 0x02) >> 1;
		        nodeid[7] = cp-> buffer_[5] & 0x01;
		        nodeid[8] =(  cp->buffer_[6] & 0x80) >> 7;
		        nodeid[9] =(  cp->buffer_[6] & 0x40) >> 6;
		        nodeid[10] =(  cp->buffer_[6] & 0x20) >> 5;
		        nodeid[11] =(  cp->buffer_[6] & 0x10) >> 4;
				nodeid[12] =(  cp->buffer_[6] & 0x08) >> 3;
				nodeid[13] =(  cp->buffer_[6] & 0x04) >> 2;
				nodeid[14] =(  cp->buffer_[6] & 0x02) >> 1;
				nodeid[15] =  cp->buffer_[6] & 0x01;
				nodeid[16] =(  cp->buffer_[7] & 0x80) >> 7;
				nodeid[17] =(  cp->buffer_[7] & 0x40) >> 6;
				nodeid[18] =( cp-> buffer_[7] & 0x20) >> 5;
				nodeid[19] =(  cp->buffer_[7] & 0x10) >> 4;

				 int sum_tc = 0;
				 fprintf (stdout,"  TC: \n");
				 for  (int i = 0; i < 20 ; i ++ ){
					 if (nodeid[i] != 0){
						 msg.tc().nb_main_node_id(sum_tc)	     =  i ;
						 sum_tc ++;
						 fprintf (stdout,"  msg.tc().nb_main_node_id = %d , nodeid [%d] = %d \n",i,i,nodeid[i] );
					 }
				 }
				 msg.tc().count =  sum_tc;
				 msg.tc().s_node_id() = cp->buffer_[8] ;
//				 //added by xyy
//				 msg.tc().saturation() = cp->buffer_[9];
//				 msg.tc(). size() = 4 + 4*sum_tc;
		  }
//		  else if (msg.msg_type() ==  OLSR_RTC_MSG){
////			   nsaddr_t dst_addr_;
////			  int dst_node_id_;
////			  	dst_node_id_ = 0;
//			  fprintf(stdout,"node %d receive RTC \n",ra_node_id());
//			  	if (ra_node_id()==0 ){
//			  		 node_num = node_num + 1;
//			  		 if (node_num == 19){
//			  			cp->signal_type_  == rm_end_fast_route;
//			  		 }
//			  	}
//			  	else{
//			  		OLSR_rt_entry* entry = rtable_.lookup(0);
//			  			  	if (entry == NULL) {
////			  			  		debug("%f: Node %d can not forward a packet destined to %d\n",
////			  			  			CURRENT_TIME,
////			  			  			OLSR::node_id(ra_addr()),
////									OLSR::node_id(ih->daddr()));
//			  			  			drop(p, DROP_RTR_NO_ROUTE);
//			  			  			return;
//			  			  	}
//			  			  	else {
//			  			  		entry = rtable_.find_send_entry(entry);
//			  			  		assert(entry != NULL);
//			  			  		cp->next_node_id_ = entry->next_node_id();
//			  					fprintf(stdout,"relay node %d send RTC to node %d \n",ra_node_id(),cp->next_node_id_);
////			  			  		if (use_mac()) {
////			  			  			ch->xmit_failure_	= olsr_mac_failed_callback;
////			  			  			ch->xmit_failure_data_	= (void*)this;
////			  			  		}
//			  			  	}
//			  			  Scheduler::instance().schedule(target_, p, 0.0);
//			  	}
//		  }
		  else{
				fprintf (stdout, "  Node  can not process OLSR packet because does not "
							 "implement OLSR type (%x)\n",
//							 CURRENT_TIME,
//								OLSR::node_id(ra_node_id()),
								msg.msg_type());

		    }
//			 op->count = 1;
			 op.pkt_body_[0] =  msg ;
			 fprintf (stdout,"  ");
			 recv_olsr(op);
	}
//	 fprintf (stdout, "node[%d], current time %f let start quick routing\n", ra_addr_,CURRENT_TIME);
//	 fprintf (stdout, "CURRENT TIME = %f , node[%d] ,  let start quick routing\n",CURRENT_TIME, ra_node_id());
}
//complete mapping


/// \brief	This function is called whenever a packet is received. It identifies
///		the type of the received packet and process it accordingly.
///
/// If it is an %OLSR packet then it is processed. In other case, if it is a data packet
/// then it is forwarded.
///
/// \param	p the received packet.
/// \param	h a handler (not used).
///

///
/// \brief Processes an incoming %OLSR packet following RFC 3626 specification.
/// \param p received packet.
///
void OLSR::recv_olsr(OLSR_pkt & op) {
	fprintf (stdout, "CURRENT TIME = %f , STAGE = recv_olsr\n", CURRENT_TIME );

//	struct hdr_ip* ih	= HDR_IP(p);
//	OLSR_pkt* op		= PKT_OLSR(p);

	// All routing messages are sent from and to port RT_PORT,
	// so we check it.
//	assert(ih->sport() == RT_PORT);
//	assert(ih->dport() == RT_PORT);

	// If the packet contains no messages must be silently discarded.
	// There could exist a message with an empty body, so the size of
	// the packet would be pkt-hdr-size + msg-hdr-size.
	//change
//	if (op->pkt_len() < OLSR_PKT_HDR_SIZE + OLSR_MSG_HDR_SIZE) {
//		Packet::free(p);
//		return;
//	}

	///op.count< Number of OLSR_msg contained in pkt_body_.
	/// OLSR_MAX_MSGS<Maximum number of messages per packet.
	assert(op.count >= 0 && op.count <= OLSR_MAX_MSGS);
		OLSR_msg& msg = op.msg(0);

		// If ttl is less than or equal to zero, or
		// the receiver is the same as the originator,
		// the message must be silently dropped
//		fprintf(stdout,"  message.ttl = %d , message orig node id = %d , ra node id = %d\n" ,msg.ttl(),msg.orig_node_id(), ra_node_id());
		fprintf(stdout,"  message orig node id = %d , ra node id = %d\n" ,msg.orig_node_id(), ra_node_id());
		if (msg.ttl() <= 0 || msg.orig_node_id() == ra_node_id())
		{
			fprintf(stdout,"  The packet is not satisfied\n");
//			Packet::free(p);
			//消息从自己这里发出又传回自己这里，则什么也不做
		}
//			continue;
		else{
		// If the message has been processed it must not be processed again
		bool do_forwarding = true;
		//如果duplicate set中不存在记录：D_dest==Originator Address,D_seq_num==Message Sequence Number
		//如果Message type存在，就必须根据消息类型进行处理
		OLSR_dup_tuple* duplicated = state_.find_dup_tuple(msg.orig_node_id(), msg.msg_seq_num());
		if (duplicated == NULL) {
			// Process the message according to its type
			if (msg.msg_type() == OLSR_HELLO_MSG){
				fprintf (stdout,"  ");
				process_hello(msg, ra_node_id(),msg.orig_node_id());
//				if( ra_node_id() == 0)
//					{
//							fprintf(stdout,"  The  neighbor node id of node 0 is %d \n",msg.orig_node_id());
//					}
//				for(int ii=0; ii<20; ii++){
//					if(ra_node_id() == ii){
//						fprintf(stdout,"  recv_olsr : node[%d] -- Neighbor -- node[%d] \n", msg.orig_node_id() , ii );
//					}
//				}

			}

			else if (msg.msg_type() == OLSR_TC_MSG){
				fprintf (stdout,"  ");
				process_tc(msg);
			}
	//		else if (msg.msg_type() == OLSR_MID_MSG)
	//			process_mid(msg, ih->saddr());
			else {
				fprintf(stdout,"  Implement OLSR type (%x)\n",
//					CURRENT_TIME,
//					OLSR::node_id(ra_node_id()),
					msg.msg_type());
			}
		}
		//如果duplicate set中存在记录：D_dest==Originator Address,D_seq_num==Message Sequence Number
		//那么该消息已经被完全处理，需要被丢弃
		else {
			// If the message has been considered for forwarding, it should not be retransmitted again
			for (node_id_list_t::iterator it = duplicated->main_list().begin();
				it != duplicated->main_list().end();
				it++) {
				if(*it == ra_node_id()){
					do_forwarding = false;
					break;
				}
			}
		}


		if (do_forwarding) {
			// HELLO messages are never forwarded.
			// TC and MID messages are forwarded using the default algorithm.
			// Remaining messages are also forwarded using the default algorithm.
			if (msg.msg_type() != OLSR_HELLO_MSG){
				fprintf (stdout,"  ");
				forward_default(op, msg, duplicated, ra_node_id());
			}

		}

	// After processing all OLSR messages, we must recompute routing table
	fprintf (stdout,"  ");
	rtable_computation();

	// Release resources
//	Packet::free(p);
}
}

///
/// \brief Computates MPR set of a node following RFC 3626 hints.
///5.4.2.3 MPR推选
void OLSR::mpr_computation() {
	fprintf (stdout, "CURRENT TIME = %f , STAGE = mpr_computation\n", CURRENT_TIME );

	int main_node = 0;
	for (linkset_t::iterator it = linkset().begin(); it != linkset().end(); it++) {
			OLSR_link_tuple* link_tuple = *it;
			main_node = link_tuple->local_main_node_id();
			break;
	}


	// MPR computation should be done for each interface. See section 8.3.1
	// (RFC 3626) for details.
	state_.clear_mprset();

	nbset_t N; nb2hopset_t N2;
//	bool not_all_connect= false;
	int delete_count = 0;
	// N is the subset of neighbors of the node, which are
	// neighbor "of the interface I"
	for (nbset_t::iterator it = nbset().begin(); it != nbset().end(); it++)
	{
		if ((*it)->status() == OLSR_STATUS_SYM) // I think that we need this check
		{
			N.push_back(*it);//计算N集合
		}
	}


//	for (nbset_t::iterator it = N.begin(); it != N.end(); it++){
//		OLSR_nb_tuple* N_tuple = *it;
//		fprintf (stdout,"  main_node = %d, N_node_id = %d \n",main_node, N_tuple->nb_main_node_id());
//	}

	// N2 is the set of 2-hop neighbors reachable from "the interface
	// I", excluding:
	// (i)   the nodes only reachable by members of N with willingness WILL_NEVER
	// (ii)  the node performing the computation正在进行计算的节点
	// (iii) include all the symmetric neighbors: the nodes for which there exists a symmetric
	//       link to this node on some interface.
	//计算N2集合
	for (nb2hopset_t::iterator it = nb2hopset().begin(); it != nb2hopset().end(); it++) {
		OLSR_nb2hop_tuple* nb2hop_tuple = *it;
		bool ok = true;
//		OLSR_nb_tuple* nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb_main_addr());
		OLSR_nb_tuple* nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb_main_node_id());//(iii)该节点二跳节点的一跳节点是该节点的一跳节点
		if (nb_tuple == NULL)
			ok = false;
		else {
			nb_tuple = state_.find_nb_tuple(nb2hop_tuple->nb_main_node_id(), OLSR_WILL_NEVER);//(i)该节点二跳节点的一跳节点是该节点的一跳节点，willingnessWie否
//			nb_tuple = state_.find_nb_tuple(nb2hop_tuple->nb_main_addr(), OLSR_WILL_NEVER);
			if (nb_tuple != NULL)
				ok = false;
			else {
//				nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb2hop_addr());
				nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb2hop_node_id());//(iii)该二跳节点是本节点的一跳节点
				if (nb_tuple != NULL)
					ok = false;
			}
		}

		if (ok){
			N2.push_back(nb2hop_tuple);
//			bool not_all_connect= true;
//			fprintf(stdout,"2hop is two the list for [node] %d [ time] %f ",ra_node_id(),CURRENT_TIME);
		}
	}

//	for (nb2hopset_t::iterator it = N2.begin(); it != N2.end(); it++){
//		OLSR_nb2hop_tuple* N2_tuple = *it;
//		fprintf (stdout,"  main_node = %d, N2_node_id = %d, N2_nb_node_id = %d \n",main_node, N2_tuple->nb2hop_node_id(), N2_tuple->nb_main_node_id());
//	}
//	fprintf (stdout,"  \n");

//if(not_all_connect){
	// 1. Start with an MPR set made of all members of N with
		// N_willingness equal to WILL_ALWAYS
	// N_willingness == WILL_ALWAYS
		for (nbset_t::iterator it = N.begin(); it != N.end(); it++) {
			OLSR_nb_tuple* nb_tuple = *it;
			if (nb_tuple->willingness() == OLSR_WILL_ALWAYS){
//				   if(mprselset().size() <6){
				state_.insert_mpr_node_id(nb_tuple->nb_main_node_id());
				fprintf (stdout,"  WILL_ALWAYS : main_node = %d, mpr_node_id = %d \n",main_node, nb_tuple->nb_main_node_id());
			}
//				   }

		}

		// 2. Calculate D(y), where y is a member of N, for all nodes in N.
		// We will do this later.

		// 3. Add to the MPR set those nodes in N, which are the *only*
		// nodes to provide reachability to a node in N2. Remove the
		// nodes from N2 which are now covered by a node in the MPR set.
		//************************************************这一段完全看不懂------现在能够看懂了************************************************//
		mprset_t foundset;

		std::set<int> deleted_node_ids;
	//	std::set<nsaddr_t> deleted_addrs;
		for (nb2hopset_t::iterator it = N2.begin(); it != N2.end(); it++) {
			OLSR_nb2hop_tuple* nb2hop_tuple1 = *it;
//			fprintf (stdout,"  N2_nb2hop_node_id = %d .\n", nb2hop_tuple1->nb2hop_node_id());

			mprset_t::iterator pos = foundset.find(nb2hop_tuple1->nb2hop_node_id());
			if (pos != foundset.end()){//如果找到这样的节点，则不执行下面的语句，跳到下一个循环
//				fprintf (stdout,"  pos != foundset.end()\n");//可是这里的foundset是刚开始定义的，所以第一次肯定为空
				continue;//foundset.end??
			}

			bool found = false;
			for (nbset_t::iterator it2 = N.begin(); it2 != N.end(); it2++) {
				if ((*it2)->nb_main_node_id() == nb2hop_tuple1->nb_main_node_id()) {
					//N中存在连接本节点和N2中节点it2的一跳节点
					found = true;
					break;//跳出最近的一个循环
				}
		    }
			if (!found){
//				fprintf (stdout,"  !found111\n");
				continue;
			}

			found = false;
			for (nb2hopset_t::iterator it2 = it + 1; it2 != N2.end(); it2++) {
				OLSR_nb2hop_tuple* nb2hop_tuple2 = *it2;
				if (nb2hop_tuple1->nb2hop_node_id() == nb2hop_tuple2->nb2hop_node_id()) {
					//从本节点到二跳节点存在多条路径，导致二跳节点集中该二跳节点有多条记录
					//但由于它经过不同的一跳节点到达本节点，所以记录不同
					foundset.insert(nb2hop_tuple1->nb2hop_node_id());
					found = true;
					break;
				}
			}
			if (!found) {
				//此时本节点到该二跳节点只存在一条路径，因此找到只通过唯一N中一跳节点到达该二跳节点的一跳节点
//				fprintf (stdout,"  !found222\n");
//				if(mprselset().size() <6){
				state_.insert_mpr_node_id(nb2hop_tuple1->nb_main_node_id());//将该节点加入到mpr集中
				fprintf (stdout,"  ONLY_NODE : main_node = %d, mpr_node_id = %d \n",main_node, nb2hop_tuple1->nb_main_node_id());
//				}

				delete_count = 0;
//				for (nb2hopset_t::iterator it2 = it + 1; it2 != N2.end(); it2++) {
				for (nb2hopset_t::iterator it2 = N2.begin(); it2 != N2.end(); it2++) {
					OLSR_nb2hop_tuple* nb2hop_tuple2 = *it2;
//					if ( nb2hop_tuple1->nb2hop_node_id() != nb2hop_tuple2->nb2hop_node_id() &&
					if (nb2hop_tuple1->nb_main_node_id() == nb2hop_tuple2->nb_main_node_id()) {
						if( it2 <= it )//计算it指针前面删除了几条，之后补回来
							delete_count++;
//						fprintf (stdout,"  delete_count = %d\n",delete_count);

//					    fprintf (stdout,"  deleted_node_ids : N2_node_id = %d, N2_nb_node_id = %d .\n",
//					    		nb2hop_tuple2->nb2hop_node_id(), nb2hop_tuple2->nb_main_node_id());

//						//存在和该二跳节点经过同样一跳节点到达本节点的二跳节点
						//将其加入到deleted_node_ids中，并在N2中将其删除
						//之所以删除是因为与之相连接的一跳节点已经加入到mpr集中了
						deleted_node_ids.insert(nb2hop_tuple2->nb2hop_node_id());
//						fprintf (stdout,"  insert\n");
						it2 = N2.erase(it2);//应该是erase之后指针就指向下一条了
//						fprintf (stdout,"  erase\n");
						it2--;//所以要减一
//						fprintf (stdout,"  --\n");
					}
				}
//				it = N2.erase(it);//由于与之相连接的一跳节点已经加入到mpr集中了，所以删除
//				fprintf (stdout,"  N2.erase(it)\n");
//				it = it - delete_count;
				if(N2.begin() == N2.end())
					break;

				for ( int ii =0;ii<delete_count; ii++ ){
					if( it == N2.begin() )//如果删除了一定数值之后已经到达最前面，则不需要再删除了
						break;
					else
						it--;
				}
//				fprintf (stdout,"  it--\n");
			}
			//下面的重置迭代器不是很懂……----------懂了
			for (std::set<int>::iterator it2 =deleted_node_ids.begin(); it2 != deleted_node_ids.end(); it2++) {
				for (nb2hopset_t::iterator it3 = N2.begin();it3 != N2.end();it3++) {
					if ((*it3)->nb2hop_node_id() == *it2) {
						it3 = N2.erase(it3);//把与已经放到mpr集中的一跳节点相连的二跳节点从N2中删除
						it3--;
						// I have to reset the external iterator because it
						// may have been invalidated by the latter deletion
						it = N2.begin();
						it--;
					}
				}
			}
			deleted_node_ids.clear();//对于这个刚加入mpr节点的deleted_node_ids集清零


//			if(N2.begin() == N2.end())
//				fprintf (stdout," N2 = NULL111 . \n");
//			else{
//				for (nb2hopset_t::iterator it0 = N2.begin(); it0 != N2.end(); it0++){
//					OLSR_nb2hop_tuple* N2_tuple = *it0;
//					fprintf (stdout,"  main_node = %d, N2_node_id = %d, N2_nb_node_id = %d \n",main_node, N2_tuple->nb2hop_node_id(), N2_tuple->nb_main_node_id());
//				}
//				fprintf (stdout,"  \n");
//			}

		}
		//*************************************************************************************************************//

//		if(N2.begin() == N2.end())
//			fprintf (stdout," N2 = NULL222 . \n");

		// 4. While there exist nodes in N2 which are not covered by at
		// least one node in the MPR set:
		while (N2.begin() != N2.end()) {//N2不为空
			// 4.1. For each node in N, calculate the reachability, i.e., the
			// number of nodes in N2 which are not yet covered by at
			// least one node in the MPR set, and which are reachable
			// through this 1-hop neighbor
			map<int, std::vector<OLSR_nb_tuple*> > reachability;
			set<int> rs;
			for (nbset_t::iterator it = N.begin(); it != N.end(); it++) {
				OLSR_nb_tuple* nb_tuple = *it;
				int r = 0;
				for (nb2hopset_t::iterator it2 = N2.begin(); it2 != N2.end(); it2++) {
					OLSR_nb2hop_tuple* nb2hop_tuple = *it2;
					if (nb_tuple->nb_main_node_id()== nb2hop_tuple->nb_main_node_id())
						r++;//计算该节点的reachability

//					fprintf (stdout,"  nb_tuple->nb_main_node_id() = %d , nb2hop_tuple->nb_main_node_id() = %d .\n",
//							nb_tuple->nb_main_node_id(), nb2hop_tuple->nb_main_node_id());
				}
//				fprintf (stdout,"  The reachability of node[%d] is %d.\n",	nb_tuple->nb_main_node_id(), r);
//					fprintf (stdout,"  The reachability of node %d is %d.\n",	nb_tuple->main_node_id_(), r);

				rs.insert(r);
				reachability[r].push_back(nb_tuple);
			}

			// 4.2. Select as a MPR the node with highest N_willingness among
			// the nodes in N with non-zero reachability. In case of
			// multiple choice select the node which provides
			// reachability to the maximum number of nodes in N2. In
			// case of multiple nodes providing the same amount of
			// reachability, select the node as MPR whose D(y) is
			// greater. Remove the nodes from N2 which are now covered
			// by a node in the MPR set.
			OLSR_nb_tuple* max = NULL;
			int max_r = 0;
			for (set<int>::iterator it = rs.begin(); it != rs.end(); it++) {
				int r = *it;
				if (r > 0) {
					for (std::vector<OLSR_nb_tuple*>::iterator it2 = reachability[r].begin();
						it2 != reachability[r].end();
						it2++) {
						OLSR_nb_tuple* nb_tuple = *it2;
						if (max == NULL || nb_tuple->willingness() > max->willingness()) {
							max = nb_tuple;
							max_r = r;
						}
						else if (nb_tuple->willingness() == max->willingness()) {
							if (r > max_r) {
								max = nb_tuple;
								max_r = r;
							}
							else if (r == max_r) {
								fprintf (stdout,"  ");
								int degree_nb_tuple = degree(nb_tuple);
								fprintf (stdout,"  ");
								int degree_max = degree(max);
								if (degree_nb_tuple > degree_max) {
									max = nb_tuple;
									max_r = r;
								}
							}
						}
					}
				}
			}
			if (max != NULL) {
				state_.insert_mpr_node_id(max->nb_main_node_id());
				fprintf (stdout,"  MAX_REACHABILITY : main_node = %d, mpr_node_id = %d \n",main_node ,max->nb_main_node_id());
				std::set<int> nb2hop_node_ids;
				for (nb2hopset_t::iterator it = N2.begin(); it != N2.end(); it++) {
					OLSR_nb2hop_tuple* nb2hop_tuple = *it;
					if (nb2hop_tuple->nb_main_node_id() == max->nb_main_node_id()) {
						 nb2hop_node_ids.insert(nb2hop_tuple->nb2hop_node_id());//插入节点
						it = N2.erase(it);//防止重新计算N2的这个节点，从N2中删除节点
						it--;
					}
				}
				//为什么还要再重新find和erase一次节点？？？？？？
				//防止N2中同一个节点由于有多条路径而存在多条记录的现象
				for (nb2hopset_t::iterator it = N2.begin(); it != N2.end(); it++) {
					OLSR_nb2hop_tuple* nb2hop_tuple = *it;
					std::set<int>::iterator it2 = nb2hop_node_ids.find(nb2hop_tuple->nb2hop_node_id());
					if (it2 != nb2hop_node_ids.end()) {//即找到相应节点
						it = N2.erase(it);
						it--;
					}
				}
			}

//			if(N2.begin() == N2.end())
//				fprintf (stdout," N2 = NULL333 . \n");
//			else{
//				for (nb2hopset_t::iterator it0 = N2.begin(); it0 != N2.end(); it0++){
//					OLSR_nb2hop_tuple* N2_tuple = *it0;
//					fprintf (stdout,"  main_node = %d, N2_node_id = %d, N2_nb_node_id = %d \n",main_node, N2_tuple->nb2hop_node_id(), N2_tuple->nb_main_node_id());
//				}
//				fprintf (stdout,"  \n");
//			}

		}

//       if(ra_node_id() == 0){
////    	   fprintf (stdout, "mpr select size is %d current time is %f \n ",mprselset().size(), CURRENT_TIME);
//       }


//		for (mprset_t::iterator it = mprset_.begin(); it != mpr().end(); it++) {
//			OLSR_mpr_tuple* mpr_tuple = *it;
//			fprintf (stdout, "  FINAL : main_node = %d, mpr_node_id = %d .\n",main_node, mpr_tuple);
//		}

		state_.print_mpr_node_id();

}

///
/// \brief Creates the routing table of the node following RFC 3626 hints.
///
void OLSR::rtable_computation() {
	fprintf (stdout, "CURRENT TIME = %f , STAGE = rtable_computation\n", CURRENT_TIME );

	// 1. All the entries from the routing table are removed.
	rtable_.clear();
	int main_node = 0;
	int addnode[20] = {0};
	int count = 0;
	///whether to complete quick routing, first to find the number of the destination in the routing table

	// 2. The new routing entries are added starting with the
	// symmetric neighbors (h=1) as the destination nodes.
	fprintf (stdout, "  h = 1 .\n");
	for (nbset_t::iterator it0 = nbset().begin(); it0 != nbset().end(); it0++) {
		OLSR_nb_tuple* nb_tuple = *it0;
		if (nb_tuple->status() == OLSR_STATUS_SYM) {
			bool nb_node_id = false;
			OLSR_link_tuple* lt = NULL;
//			R_dest_addr is equal to the main address of the neighbor
//			R_dest_addr = L_neighbor_iface_addr, of the associated link tuple;
//			R_next_addr = L_neighbor_iface_addr, of the associated link tuple;
//			R_dist = 1;
//			R_iface_addr = L_local_iface_addr of the associated link tuple.
			//在一跳邻居节点中找到一条的链路作为主路由
			for (linkset_t::iterator it2 = linkset().begin(); it2 != linkset().end(); it2++) {
				OLSR_link_tuple* link_tuple = *it2;
				if (link_tuple->nb_main_node_id() == nb_tuple->nb_main_node_id()
						&& link_tuple->time() >= CURRENT_TIME ) {
					main_node = link_tuple->local_main_node_id();
					addnode[ link_tuple->nb_main_node_id() ] = 1;
					lt = link_tuple;

					rtable_.add_entry(link_tuple->nb_main_node_id(),
							link_tuple->nb_main_node_id(),//next = dest
							link_tuple->local_main_node_id(),
							1,
							SNR[ link_tuple->local_main_node_id() ] [ link_tuple->nb_main_node_id() ]
							);
					fprintf (stdout, "    rtable1: main = %d, dest = %d, next = %d, dist = 1, snr = %d.\n",
							link_tuple->local_main_node_id() ,
							link_tuple->nb_main_node_id(),
							link_tuple->nb_main_node_id(),
							SNR[ link_tuple->local_main_node_id() ] [ link_tuple->nb_main_node_id() ]
							);

						nb_node_id  = true;
				}
			}

			//************************************************ 这一段不是很懂 ************************************************//
//			no R_dest_addr is equal to the main address of the neighbor
//			R_dest_addr = main address of the neighbor;
//			R_next_addr = L_neighbor_iface_addr, of the associated link tuple;
//			R_dist = 1;
//			R_iface_addr = L_local_iface_addr of the associated link tuple.
			//在没有R_dest_addr == the main address of the neighbor时
			//nb_tuple->nb_main_node_id()和link_tuple->nb_main_node_id()有什么不同？
			if (!nb_node_id&& lt != NULL) {
				addnode[ nb_tuple->nb_main_node_id() ] = 1;

				rtable_.add_entry(nb_tuple->nb_main_node_id(),
						lt->nb_main_node_id(),
						lt->local_main_node_id(),
						1,
						SNR[ lt->local_main_node_id() ] [ lt->nb_main_node_id() ]);
				fprintf (stdout, "    rtable1: main = %d, dest = %d, next = %d, dist = 1, snr = %d.\n",
						lt->local_main_node_id(),
						nb_tuple->nb_main_node_id(),
						lt->nb_main_node_id(),
						SNR[ lt->local_main_node_id() ] [ lt->nb_main_node_id() ]);
			}
			//*************************************************************************************************************//

		}
	}

//		fprintf(stdout, " \n");


//				//在一跳邻居节点中找到二跳的链路作为备用路由
//				//或者在snr较小的一跳邻居节点里找备用路由
//	for (nbset_t::iterator it0 = nbset().begin(); it0 != nbset().end(); it0++) {
//		OLSR_nb_tuple* nb_tuple = *it0;
//		if (nb_tuple->status() == OLSR_STATUS_SYM) {
//			for (linkset_t::iterator it2 = linkset().begin(); it2 != linkset().end(); it2++) {
//				OLSR_link_tuple* link_tuple = *it2;
//				if (link_tuple->nb_main_node_id() == nb_tuple->nb_main_node_id()	&& link_tuple->time() >= CURRENT_TIME ) {
//					count = 1;
//					bool addentry2 = false;
//					OLSR_rt_entry* entrysmin2 = new OLSR_rt_entry();
//
//					if ( addnode[ nb_tuple->nb_main_node_id() ] < 2  ){
//						for (nb2hopset_t::iterator it3 = nb2hopset().begin(); it3 != nb2hopset().end(); it3++){
//							OLSR_nb2hop_tuple* nb2hop_tuple = *it3;
//							if ( nb_tuple->nb_main_node_id() == nb2hop_tuple->nb2hop_node_id() ){
//								//跳数为1的第一条路由表项
//								OLSR_rt_entry* entry = rtable_.lookupall(nb2hop_tuple->nb_main_node_id(), 1, count);
//								if(entry == NULL )
//									continue;
//
//								count = count+1;
//
//								if (count==2){
//									entrysmin2->dest_node_id() = nb2hop_tuple->nb2hop_node_id();
//									entrysmin2->next_node_id() = entry->next_node_id();
//									entrysmin2->main_node_id() = entry->main_node_id();
//									entrysmin2->dist() = 2;
//									entrysmin2->snr() = MIN( entry->snr(),
//																						SNR[ entrysmin2->dest_node_id() ][ entrysmin2->next_node_id() ] );
//									addentry2 = true;
//								}
//								else{
//									if( entry->snr() > entrysmin2->snr()){
//										entrysmin2->dest_node_id() = nb2hop_tuple->nb2hop_node_id();
//										entrysmin2->next_node_id() = entry->next_node_id();
//										entrysmin2->main_node_id() = entry->main_node_id();
//										entrysmin2->dist() = 2;
//										entrysmin2->snr() = MIN( entry->snr(),
//																							SNR[ entrysmin2->dest_node_id() ][ entrysmin2->next_node_id() ] );
//									}
//								}
//							}
//						}
//					}
//
//					if(addentry2){
//						addnode[ entrysmin2->dest_node_id() ] ++;
//						rtable_.add_entry(entrysmin2->dest_node_id() ,
//									entrysmin2->next_node_id() ,
//									entrysmin2->main_node_id() ,
//									entrysmin2->dist() ,
//									entrysmin2->snr() );
//						fprintf (stdout, "    rtable2: main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n",
//									entrysmin2->main_node_id() ,	entrysmin2->dest_node_id() ,
//									entrysmin2->next_node_id() ,	entrysmin2->dist() ,	entrysmin2->snr() );
//					}
//
//				}
//			}
//		}
//	}

	// N2 is the set of 2-hop neighbors reachable from this node, excluding:
	// (i)   the nodes only reachable by members of N with willingness WILL_NEVER
	// (ii)  the node performing the computation
	// (iii) all the symmetric neighbors: the nodes for which there exists a symmetric
	//       link to this node on some interface.
	fprintf (stdout, "  h = 2 .\n");

	int itdest = 0;
	bool addentry1 = false;
	bool addentry2 = false;
	OLSR_rt_entry* entry = new OLSR_rt_entry();
	OLSR_rt_entry* entrysmin1 = new OLSR_rt_entry();
	OLSR_rt_entry* entrysmin2 = new OLSR_rt_entry();
	for (nb2hopset_t::iterator it0 = nb2hopset().begin(); it0 != nb2hopset().end(); it0++) {
		//初始化
		count = 0;
		addentry1 = false;
		addentry2 = false;
		entrysmin1 = new OLSR_rt_entry();
		entrysmin2 = new OLSR_rt_entry();

		OLSR_nb2hop_tuple* nb2hop_tuple0 = *it0;
		itdest  = nb2hop_tuple0->nb2hop_node_id();
		if ( addnode[ nb2hop_tuple0->nb2hop_node_id() ] < 2 ){
			for (nb2hopset_t::iterator it1 = nb2hopset().begin(); it1 != nb2hopset().end(); it1++){
				OLSR_nb2hop_tuple* nb2hop_tuple = *it1;
				if( nb2hop_tuple ->nb2hop_node_id() == itdest ){
					bool ok = true;
					OLSR_nb_tuple* nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb_main_node_id());
					if (nb_tuple == NULL)
						ok = false;
					else {
						nb_tuple = state_.find_nb_tuple(nb2hop_tuple->nb_main_node_id(), OLSR_WILL_NEVER);//(i)
						if (nb_tuple != NULL)
							ok = false;
//						else {//在二跳邻居节点集中找一跳邻居节点集
//							nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb2hop_node_id());//(iii)
//							if (nb_tuple != NULL)
//								ok = false;
//						}
					}

					// 3. For each node in N2 create a new entry in the routing table
					if (ok ) {
						entry = rtable_.lookup(nb2hop_tuple->nb_main_node_id());
						if ( entry == NULL )
							continue;

						count++;
						if( count == 1 ){
							entrysmin1->dest_node_id() = nb2hop_tuple->nb2hop_node_id();
							entrysmin1->next_node_id() = entry->next_node_id();
							entrysmin1->main_node_id() = entry->main_node_id();
							entrysmin1->dist() = entry->dist()+1;
							entrysmin1->snr() = SNR[ entrysmin1->dest_node_id() ][ entry->dest_node_id() ] ;
							addentry1 = true;
						}

						else{//以当前的拓扑结构是进不了这个循环的，即count最大也只有2
							if( count == 2 ){
								entrysmin2->dest_node_id() = nb2hop_tuple->nb2hop_node_id();
								entrysmin2->next_node_id() = entry->next_node_id();
								entrysmin2->main_node_id() = entry->main_node_id();
								entrysmin2->dist() = entry->dist()+1;
								entrysmin2->snr() = SNR[ entrysmin2->dest_node_id() ][ entry->dest_node_id() ] ;
								addentry2 = true;
							}

							else{
								if( entry->snr() > entrysmin1->snr() || entry->snr() > entrysmin2->snr() ){
									if( entrysmin1->snr() < entrysmin2->snr() ){
										entrysmin1->dest_node_id() = nb2hop_tuple->nb2hop_node_id();
										entrysmin1->next_node_id() = entry->next_node_id();
										entrysmin1->main_node_id() = entry->main_node_id();
										entrysmin1->dist() = entry->dist()+1;
										entrysmin1->snr() = SNR[ entrysmin1->dest_node_id() ][ entry->dest_node_id() ] ;
									}
									else{
										entrysmin2->dest_node_id() = nb2hop_tuple->nb2hop_node_id();
										entrysmin2->next_node_id() = entry->next_node_id();
										entrysmin2->main_node_id() = entry->main_node_id();
										entrysmin2->dist() = entry->dist()+1;
										entrysmin2->snr() = SNR[ entrysmin2->dest_node_id() ][ entry->dest_node_id() ] ;
									}
								}
							}
						}
					}
				}
			}
		}

		if ( addnode[ itdest ] == 1 && addentry1 ){
			if (       addentry2 == false
					||entrysmin1->dist()  < entrysmin2->dist()
					|| ( entrysmin1->dist()  == entrysmin2->dist() && entrysmin1->snr() >= entrysmin2->snr()) ){
				entrysmin2 = entrysmin1;
			}
			addentry1 = false;
			addentry2  = true;
		}

		if(addentry1){
			addnode[ entrysmin1->dest_node_id() ]++ ;
			rtable_.add_entry(entrysmin1->dest_node_id() ,
						entrysmin1->next_node_id() ,
						entrysmin1->main_node_id() ,
						entrysmin1->dist() ,
						entrysmin1->snr() );
			fprintf (stdout, "    rtable1: main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n",
			entrysmin1->main_node_id() , entrysmin1->dest_node_id() ,
			entrysmin1->next_node_id() ,	entrysmin1->dist() ,	entrysmin1->snr() );
			}
		if(addentry2){
			addnode[ entrysmin2->dest_node_id() ]++ ;
			rtable_.add_entry(entrysmin2->dest_node_id() ,
						entrysmin2->next_node_id() ,
						entrysmin2->main_node_id() ,
						entrysmin2->dist() ,
						entrysmin2->snr() );
			fprintf (stdout, "    rtable2: main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n",
						entrysmin2->main_node_id() ,	entrysmin2->dest_node_id() ,
						entrysmin2->next_node_id() , entrysmin2->dist() , entrysmin2->snr() );
		}


	}



	fprintf (stdout, "  h = 3 .\n");

//	for (topologyset_t::iterator it = topologyset().begin(); it != topologyset().end(); it++) {
//		OLSR_topology_tuple* topology_tuple = *it;
//		fprintf(stdout, "    topology->dest_node_id = %d, topology->last_node_id = %d . \n",
//				topology_tuple->dest_node_id(),  topology_tuple->last_node_id());
//	}
//	fprintf(stdout, " \n");

//	int can = true;
	int number = 0;
	OLSR_rt_entry* entry0   = new OLSR_rt_entry();
//	OLSR_rt_entry* entry1 = new OLSR_rt_entry();
	OLSR_rt_entry* entry2 = new OLSR_rt_entry();

//	for (int index = 1; ; index++){
//	for (int index = 1; index < 10 ; index++){
	for (int index = 1; index < 4 ; index++){
//		fprintf (stdout, "  index = %d .\n", index);
		for (topologyset_t::iterator it0 = topologyset().begin(); it0 != topologyset().end(); it0++) {
			//初始化
			count = 0;
			number = 1;
//			can = true;
			addentry1 = false;
			addentry2 = false;
			entrysmin1 = new OLSR_rt_entry();
			entrysmin2 = new OLSR_rt_entry();

			OLSR_topology_tuple* topology_tuple0 = *it0;
			itdest  = topology_tuple0->dest_node_id();
//			fprintf(stdout, "    topology->dest_node_id = %d, topology->last_node_id = %d . \n",
//					topology_tuple0->dest_node_id(),  topology_tuple0->last_node_id());

			if (  main_node == itdest
				||  addnode[ itdest ] >= 2 )
				continue;

			for (u_int32_t h = 2; h<10; h++) {
				for (topologyset_t::iterator it1 = topologyset().begin(); it1 != topologyset().end(); it1++) {
					OLSR_topology_tuple* topology_tuple = *it1;
//					fprintf(stdout, "    topology->dest_node_id = %d, topology->last_node_id = %d . \n",
//							topology_tuple->dest_node_id(),  topology_tuple->last_node_id());

					if( topology_tuple ->dest_node_id() == itdest ){
						number = 1;
//						entry1 = rtable_.lookup(topology_tuple->dest_node_id());
//						if(entry1 != NULL)
//							continue;

						entry2 = rtable_.lookupall(topology_tuple->last_node_id(), h, number);

						if ( entry2 == NULL )
							continue;

//						//if (main_node == 1 && ( itdest == 6 || itdest == 5 ) )
//						if (main_node == 1 )
//							fprintf (stdout, "    rtable-1-entry2: number = %d, main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n",
//									number, entry2->main_node_id() , entry2->dest_node_id() ,
//									entry2->next_node_id() , 	entry2->dist() ,entry2->snr() );

						while( entry2->dist() == h ){

							count++;
//							fprintf (stdout, "  count = %d .\n", count);

							//路由表中两条都没有存在
							if( count == 1 ){
//									fprintf(stdout, "     count = 0%d .\n",count );
								entrysmin1->dest_node_id() = topology_tuple->dest_node_id();
								entrysmin1->next_node_id() = entry2->next_node_id();
								entrysmin1->main_node_id() = entry2->main_node_id();
								entrysmin1->dist() = h+1;
								entrysmin1->snr() = SNR[ main_node ][ entrysmin1->next_node_id() ] ;
								addentry1 = true;
							}

							else if( count == 2 && entry2->next_node_id() != entrysmin1->next_node_id()  ){
									entrysmin2->dest_node_id() = topology_tuple->dest_node_id();
									entrysmin2->next_node_id() = entry2->next_node_id();
									entrysmin2->main_node_id() = entry2->main_node_id();
									entrysmin2->dist() = h+1;
									entrysmin2->snr() = SNR[ main_node ][ entrysmin2->next_node_id() ] ;
									addentry2 = true;
								}
							else if( entry2->next_node_id() != entrysmin1->next_node_id() &&
											entry2->next_node_id() != entrysmin2->next_node_id() &&
										 ( entry2->snr() > entrysmin1->snr() || entry2->snr() > entrysmin2->snr() ) ){
								if( entrysmin1->snr() < entrysmin2->snr() ){
//											fprintf(stdout, "     count = %d .\n",count );
									entrysmin1->dest_node_id() = topology_tuple->dest_node_id();
									entrysmin1->next_node_id() = entry2->next_node_id();
									entrysmin1->main_node_id() = entry2->main_node_id();
									entrysmin1->dist() = h+1;
									entrysmin1->snr() = SNR[ main_node ][ entrysmin1->next_node_id() ] ;
									addentry1 = true;
								}
								else{
//											fprintf(stdout, "     count = %d .\n",count );
									entrysmin2->dest_node_id() = topology_tuple->dest_node_id();
									entrysmin2->next_node_id() = entry2->next_node_id();
									entrysmin2->main_node_id() = entry2->main_node_id();
									entrysmin2->dist() = h+1;
									entrysmin2->snr() = SNR[ main_node ][ entrysmin2->next_node_id() ] ;
									addentry2 = true;
								}
							}

							number++;
							entry2 = rtable_.lookupall(topology_tuple->last_node_id(), h, number);
							if (entry2 == NULL){
								break;
							}

////							//if (main_node == 1 && itdest == 6 )
//							if (main_node == 1 )
//								fprintf (stdout, "    rtable-2-entry2: number = %d, main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n",
//									number, entry2->main_node_id() , entry2->dest_node_id() ,
//									entry2->next_node_id() , 	entry2->dist() ,entry2->snr() );
						}
					}
				}
				if(addentry2)
					break;
			}

			if ( addnode[ itdest ] == 1 && addentry1 ){
				if (       addentry2 == false
						|| ( entrysmin1->dist()  < entrysmin2->dist() )
						|| ( entrysmin1->dist()  == entrysmin2->dist() && entrysmin1->snr() >= entrysmin2->snr()) ){
					entrysmin2 = entrysmin1;
				}
				addentry1 = false;
				addentry2  = true;
			}

			if(addentry1){
				addnode[ entrysmin1->dest_node_id() ]++ ;
				rtable_.add_entry(entrysmin1->dest_node_id() ,
							entrysmin1->next_node_id() ,
							entrysmin1->main_node_id() ,
							entrysmin1->dist() ,
							entrysmin1->snr() );
				fprintf (stdout, "    rtable1: main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n",
							entrysmin1->main_node_id() ,
							entrysmin1->dest_node_id() ,
							entrysmin1->next_node_id() ,
							entrysmin1->dist() ,
							entrysmin1->snr() );
			}
			if(addentry2){
				addnode[ entrysmin2->dest_node_id() ]++ ;
				rtable_.add_entry(entrysmin2->dest_node_id() ,
							entrysmin2->next_node_id() ,
							entrysmin2->main_node_id() ,
							entrysmin2->dist() ,
							entrysmin2->snr() );
				fprintf (stdout, "    rtable2: main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n",
							entrysmin2->main_node_id() ,
							entrysmin2->dest_node_id() ,
							entrysmin2->next_node_id() ,
							entrysmin2->dist() ,
							entrysmin2->snr() );
				continue;
			}

		}

		//没有必要，因为在实际中网络中的节点是会离开网络的
		//由于3跳以上的路由依赖于拓扑表，因此要多次查询拓扑表防止有节点未加入其中
		for(int ii=0;ii<20;ii++){
			if ( ii != main_node ){
				entry0 = rtable_.lookup( ii );
				if ( entry0 == NULL )
					break;
			}
		}
		if (  entry0 != NULL)
			break;

//		if ( entry0 == NULL )//存在节点还没有加入路由表
//			continue;
//		else//所有节点都加入路由表
//			break;

	}



	/// Judge if the current node completes the quick routing

		//??
       for (int i=0;i<20;i++){
    	   if (ra_node_id()==i && end_quick_route_==1){
    		   int q_route[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    		   for(int j=0;j<20;j++){
    			   OLSR_rt_entry* entry1 = rtable_.lookup(j);
    			   if(entry1 != NULL){
    					q_route[j] =1;
    			   }
    		   }

    		 int num_com_node = 0;
              for(int a = 0;a<20;a++){
            	  if(ra_node_id()!= a){
            		  if(q_route[a]==1){
						  num_com_node = num_com_node +1;
					  }
            	  }

              }
//              fprintf(stdout,"[online node]%d  \n",online_node_num());
              if (num_com_node ==online_node_num()-1){
            	  fprintf(stdout,"  node[%d] complete quick route\n",ra_node_id());
            	  end_quick_route() =2;
              }
    	   }
       }
   	if (ra_node_id() ==1 ){
   		OLSR_rt_entry* entry1 = rtable_.lookup(7);
//   		if (entry1 != NULL) {
//   			fprintf(stdout,"[time] %f [node] %d [connect to node 7] \n", CURRENT_TIME, ra_node_id()  );
//   		}
   		if (entry1 == NULL) {
//   			fprintf(stdout,"[time] %f [node] %d [disconnect to node 7] \n", CURRENT_TIME, ra_node_id()  );
   		}
   	}

}

///
/// \brief Processes a HELLO message following RFC 3626 specification.
///
/// Link sensing and population of the Neighbor Set, 2-hop Neighbor Set and MPR
/// Selector Set are performed.
///
/// \param msg the %OLSR message which contains the HELLO message.
/// \param receiver_iface the address of the interface where the message was received from.
/// \param sender_iface the address of the interface where the message was sent from.
///
void OLSR::process_hello(OLSR_msg& msg, int receiver_main_node_id, int sender_main_node_id) {
	// 	process_hello(msg, ra_node_id(),msg.orig_node_id());
	fprintf (stdout, "CURRENT TIME = %f , STAGE = process_hello\n", CURRENT_TIME );

	assert(msg.msg_type() == OLSR_HELLO_MSG);

//	fprintf(stdout,"OLSR HELLLLLLLLLLLLLLLLLLO\n");

	 fprintf (stdout,"  ");
     link_sensing(msg,  receiver_main_node_id, sender_main_node_id);   //链路检测
     fprintf (stdout,"  ");
	populate_nbset(msg);   //计算1-hop邻居
    fprintf (stdout,"  ");
	populate_nb2hopset(msg);   //计算2-hop邻居

	//added by xyy
    fprintf (stdout,"  ");
	UCDS_CD_computation();


    fprintf (stdout,"  ");
//	mpr_computation();   //mpr推选
	UCDS_mpr_computation();


    fprintf (stdout,"  ");
	populate_mprselset(msg);   //计算mpr selector
}

///
/// \brief Processes a TC message following RFC 3626 specification.
///
/// The Topology Set is updated (if needed) with the information of
/// the received TC message.
///
/// \param msg the %OLSR message which contains the TC message.
/// \param sender_iface the address of the interface where the message was sent from.
///5.5 拓扑消息扩散的目的在于为网络中的节点提供足够的链路状态信息来进行路由计算
void OLSR::process_tc(OLSR_msg& msg) {
	fprintf (stdout, "CURRENT TIME = %f , STAGE = process_tc\n", CURRENT_TIME );

//	for (topologyset_t::iterator it = topologyset().begin(); it != topologyset().end(); it++) {
//		OLSR_topology_tuple* topology_tuple = *it;
//		fprintf(stdout, "    main_node_id = %d, topology->dest_node_id = %d, topology->last_node_id = %d . \n",
//				msg.orig_node_id() , topology_tuple->dest_node_id(),  topology_tuple->last_node_id());
//	}
//	fprintf(stdout, " \n");

	assert(msg.msg_type() == OLSR_TC_MSG);
	double now	= CURRENT_TIME;
	OLSR_tc& tc	= msg.tc();

	// 1. If the sender interface of this message is not in the symmetric
	// 1-hop neighborhood of this node, the message MUST be discarded.
	OLSR_link_tuple* link_tuple = state_.find_sym_link_tuple(tc.s_node_id(), now);
	//tc的发送节点是接收节点的对称一跳邻居节点时才进行处理，否则return
	if (link_tuple == NULL)
		return;

	// 2. If there exist some tuple in the topology set where:
	// 	T_last_addr == originator address AND
	// 	T_seq       >  ANSN,
	// then further processing of this TC message MUST NOT be
	// performed.
	OLSR_topology_tuple* topology_tuple =	state_.find_newer_topology_tuple(msg.orig_node_id(), tc.ansn());
	if (topology_tuple != NULL)
		return;

	// 3. All tuples in the topology set where:
	//	T_last_addr == originator address AND
	//	T_seq       <  ANSN
	// MUST be removed from the topology set.
	state_.erase_older_topology_tuples(msg.orig_node_id(), tc.ansn());

	// 4. For each of the advertised neighbor main address received in
	// the TC message:
	fprintf (stdout, "  tc.count = %d\n", tc.count );
	for (int i = 0; i < tc.count; i++) {
		assert(i >= 0 && i < OLSR_MAX_ADDRS);
		int node_id = tc.nb_main_node_id(i);
		// 4.1. If there exist some tuple in the topology set where:
		// 	T_dest_addr == advertised neighbor main address, AND
		// 	T_last_addr == originator address,
		// then the holding time of that tuple MUST be set to:
		// 	T_time      =  current time + validity time.
		OLSR_topology_tuple* topology_tuple =
			state_.find_topology_tuple(node_id, msg.orig_node_id());
		if (topology_tuple != NULL){
//			fprintf (stdout, "  There exists the tuple in the topology set, then update.\n" );
	//		topology_tuple->time() = now + OLSR::emf_to_seconds(msg.vtime());
			//added by li
			topology_tuple->time() = now + msg.vtime();

//			fprintf (stdout, "  msg.orig_node_id() = %d , T_d = %d , T_l = %d , T_seq = %d , T_time = %d .\n" ,
//					msg.orig_node_id() , topology_tuple->dest_node_id() , topology_tuple->last_node_id() , topology_tuple->seq() , topology_tuple->time() );

		}
		// 4.2. Otherwise, a new tuple MUST be recorded in the topology
		// set where:
		//	T_dest_addr = advertised neighbor main address,
		//	T_last_addr = originator address,
		//	T_seq       = ANSN,
		//	T_time      = current time + validity time.
		else {
			fprintf (stdout, "  There dos't exist the tuple in the topology set, then create a new one.\n" );
			OLSR_topology_tuple* topology_tuple = new OLSR_topology_tuple;
			topology_tuple->dest_node_id()	= node_id;
			topology_tuple->last_node_id()	= msg.orig_node_id();
			topology_tuple->seq()		= tc.ansn();
	//		topology_tuple->time()		= now + OLSR::emf_to_seconds(msg.vtime());
			///added by li
			topology_tuple->time()		= now + msg.vtime();

			fprintf (stdout, "  msg.orig_node_id() = %d , T_d = %d , T_l = %d , T_seq = %d , T_time = %d .\n" ,
					msg.orig_node_id() , topology_tuple->dest_node_id() , topology_tuple->last_node_id() , topology_tuple->seq() , topology_tuple->time() );

			fprintf (stdout, "  " );
			add_topology_tuple(topology_tuple);
			// Schedules topology tuple deletion
			timer_->olsr_topologytuple_timer_ = new OLSR_TopologyTupleTimer(this, topology_tuple);

//		   fprintf (stdout ,"%f77777777777777777777777777777\n",DELAY(topology_tuple->time()));
			fprintf (stdout, "  " );
			timer_->olsr_topologytuple_timer_->start(DELAY(topology_tuple->time()));
		}
	}
}

///
/// \brief Processes a MID message following RFC 3626 specification.
///
/// The Interface Association Set is updated (if needed) with the information
/// of the received MID message.
///
/// \param msg the %OLSR message which contains the MID message.
/// \param sender_iface the address of the interface where the message was sent from.
///
//void
//OLSR::process_mid(OLSR_msg& msg, nsaddr_t sender_iface) {
//	assert(msg.msg_type() == OLSR_MID_MSG);
//	double now	= CURRENT_TIME;
//	OLSR_mid& mid	= msg.mid();
//
//	// 1. If the sender interface of this message is not in the symmetric
//	// 1-hop neighborhood of this node, the message MUST be discarded.
//	OLSR_link_tuple* link_tuple = state_.find_sym_link_tuple(sender_iface, now);
//	if (link_tuple == NULL)
//		return;
//
//	// 2. For each interface address listed in the MID message
//	for (int i = 0; i < mid.count; i++) {
//		bool updated = false;
//		for (ifaceassocset_t::iterator it = ifaceassocset().begin();
//			it != ifaceassocset().end();
//			it++) {
//			OLSR_iface_assoc_tuple* tuple = *it;
//			if (tuple->iface_addr() == mid.iface_addr(i)
//				&& tuple->main_addr() == msg.orig_addr()) {
//				///tuple->time()	= now + OLSR::emf_to_seconds(msg.vtime());
//				///added by li
//				tuple->time()	= now + msg.vtime();
//				updated		= true;
//			}
//		}
//		if (!updated) {
//			OLSR_iface_assoc_tuple* tuple	= new OLSR_iface_assoc_tuple;
//			tuple->iface_addr()		= msg.mid().iface_addr(i);
//			tuple->main_addr()		= msg.orig_addr();
//			///tuple->time()			= now + OLSR::emf_to_seconds(msg.vtime());
//			///added by li
//			tuple->time()			= now + msg.vtime();
//			add_ifaceassoc_tuple(tuple);
//			// Schedules iface association tuple deletion
//			OLSR_IfaceAssocTupleTimer* ifaceassoc_timer =
//				new OLSR_IfaceAssocTupleTimer(this, tuple);
//			ifaceassoc_timer->resched(DELAY(tuple->time()));
//		}
//	}
//}

///
/// \brief OLSR's default forwarding algorithm.
///
/// See RFC 3626 for details.
///
/// \param p the %OLSR packet which has been received.
/// \param msg the %OLSR message which must be forwarded.
/// \param dup_tuple NULL if the message has never been considered for forwarding,
/// or a duplicate tuple in other case.
/// \param local_iface the address of the interface where the message was received from.
///
void OLSR::forward_default(OLSR_pkt & op, OLSR_msg& msg, OLSR_dup_tuple* dup_tuple, int local_main_node_id) {
	fprintf (stdout, "CURRENT TIME = %f , STAGE = forward_default\n", CURRENT_TIME );

double now		= CURRENT_TIME;
//	struct hdr_ip* ih	= HDR_IP(p);
//	struct RouteMacHeader * cp = RouteMac_HEADER(p);
	OLSR_tc& tc	= msg.tc();
//	fprintf (stdout ,"Forward default  YYYYYYYYYYYYYYYYYY");

	// If the sender interface address is not in the symmetric
	// 1-hop neighborhood the message must not be forwarded
	OLSR_link_tuple* link_tuple = state_.find_sym_link_tuple(tc.s_node_id(), now);
	if (link_tuple == NULL)
		return;

	// If the message has already been considered for forwarding,
	// it must not be retransmitted again
	if (dup_tuple != NULL && dup_tuple->retransmitted()) {
//		debug("%f: Node %d does not forward a message received"
//			" from %d because it is duplicated\n",
//			CURRENT_TIME,
//			OLSR::node_id(ra_node_id()),
//			OLSR::node_id(dup_tuple->node_id()));
		return;
	}

	// If the sender interface address is an interface address
	// of a MPR selector of this node and ttl is greater than 1,
	// the message must be retransmitted
	bool retransmitted = false;
	if (msg.ttl() > 1) {
		fprintf (stdout, "  msg.ttl > 1.\n" );
		OLSR_mprsel_tuple* mprsel_tuple = state_.find_mprsel_tuple(tc.s_node_id());
		if (mprsel_tuple != NULL) {
			fprintf (stdout, "  mprsel_tuple != NULL.\n" );
			OLSR_msg& new_msg = msg;
			new_msg.ttl()--;
			new_msg.hop_count()++;
			// We have to introduce a random delay to avoid
			// synchronization with neighbors.

			if (quick_route())
				{
			     fprintf (stdout,"  ");
				enque_msg(msg, jitter);
				}
			else
				{
				     fprintf (stdout,"  ");
					enque_msg(msg, JITTER);
				}
			retransmitted = true;
		}
	}

	// Update duplicate tuple...
	if (dup_tuple != NULL) {
		fprintf (stdout, "  Update duplicate tuple.\n" );
		dup_tuple->time()		= now + OLSR_DUP_HOLD_TIME;
		dup_tuple->retransmitted()	= retransmitted;
		dup_tuple->main_list().push_back(local_main_node_id);
	}
	// ...or create a new one
	else {
		fprintf (stdout, "  Create a new duplicate tuple.\n" );
		OLSR_dup_tuple* new_dup = new OLSR_dup_tuple;
		new_dup->node_id()			= msg.orig_node_id();
		new_dup->seq_num()		= msg.msg_seq_num();
		new_dup->time()			= now + OLSR_DUP_HOLD_TIME;
		new_dup->retransmitted()	= retransmitted;
		new_dup->main_list().push_back(local_main_node_id);
	     fprintf (stdout,"  ");
	     add_dup_tuple(new_dup);
		// Schedules dup tuple deletion
		timer_->olsr_duptuple_timer_ =	 new OLSR_DupTupleTimer(this, new_dup);


//		fprintf (stdout ,"%f8888888888888888888888888\n",DELAY(new_dup->time()));
	     fprintf (stdout,"  ");
		timer_->olsr_duptuple_timer_->start(DELAY(new_dup->time()));
	}
}

///
/// \brief Forwards a data packet to the appropriate next hop indicated by the routing table.
///
/// \param p the packet which must be forwarded.
///该函数是在组网的时候才会有使用到
int OLSR::forward_data(int sender_node_id) {
	fprintf (stdout, "CURRENT TIME = %f , STAGE = forward_data\n", CURRENT_TIME );

	OLSR_rt_entry* entry = new OLSR_rt_entry();
	int saturation = 0;
	int saturation1 = 0;
	int saturation2 = 0;

//	struct RouteMacHeader * cp = RouteMac_HEADER(p);
//
//	struct hdr_cmn* ch	= HDR_CMN(p);
//	struct hdr_ip* ih	= HDR_IP(p);
//
//	if (ch->direction() == hdr_cmn::UP &&
//		((u_int32_t)ih->daddr() == IP_BROADCAST || ih->daddr() == ra_addr())) {
//		dmux_->recv(p, 0);
//		return;
//	}
//	else {
//		ch->direction()	= hdr_cmn::DOWN;
//		ch->addr_type()	= NS_AF_INET;
//		if ((u_int32_t)ih->daddr() == IP_BROADCAST)
//			ch->next_hop()	= IP_BROADCAST;
//		else {

	OLSR_rt_entry* entry01 = rtable_.lookupall(sender_node_id, 1);
	OLSR_rt_entry* entry02 = rtable_.lookupall(sender_node_id, 2);

	if (entry01 == NULL) {
//				debug("%f: Node %d can not forward a packet destined to %d\n",
//					CURRENT_TIME,
//					OLSR::node_id(ra_node_id()),
//					OLSR::node_id(cp->d_node_id_));
//				drop(p, DROP_RTR_NO_ROUTE);
		fprintf(stdout, "  send node id is%d,  such an entry does not exist.\n",sender_node_id);
		return NULL;
	}
	else {//若能找到以sender_node_id为目标节点的路由表项
		fprintf (stdout, "    entry1: main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n",
														entry01->main_node_id() , entry01->dest_node_id() ,
														entry01->next_node_id() ,	entry01->dist() ,	entry01->snr()  );
		if(entry02 != NULL)
			fprintf (stdout, "    entry2: main = %d, dest = %d, next = %d,  dist = %d, snr = %d.\n\n",
														entry02->main_node_id() , entry02->dest_node_id() ,
														entry02->next_node_id() ,	entry02->dist() ,	entry02->snr()  );

		OLSR_rt_entry* entry1 = rtable_.find_send_entry(entry01);
		OLSR_rt_entry* entry2 = rtable_.find_send_entry(entry02);

		if( entry01->dist() == 1 || entry2 == NULL){
			entry = entry1;
			fprintf (stdout, "    entry : main = %d, next = %d, next_saturation = %d.\n",
							entry->main_node_id() , entry->next_node_id() ,saturation );
//					fprintf (stdout, "     entry01->dist() == 1 || entry2 == NULL \n");
//					fprintf (stdout, "    entry : main = %d, next = %d, next_saturation don't care .\n",
//									entry->main_node_id() , entry->next_node_id()  );
		}

		else{
			for (nbset_t::iterator it1 = nbset().begin(); it1 != nbset().end(); it1++) {
				OLSR_nb_tuple* nb_tuple1 = *it1;
				if( nb_tuple1->nb_main_node_id() == entry1->next_node_id() ){
					for (nbset_t::iterator it2 = nbset().begin(); it2 != nbset().end(); it2++) {
						OLSR_nb_tuple* nb_tuple2 = *it2;
						if( nb_tuple2->nb_main_node_id() == entry2->next_node_id() ){
							saturation1 = nb_tuple1->saturation();
							saturation2 = nb_tuple2->saturation();

							fprintf (stdout, "    entry1: main = %d, next = %d, next_saturation = %d.\n",
										entry1->main_node_id() , entry1->next_node_id() , saturation1 );
							fprintf (stdout, "    entry2: main = %d, next = %d, next_saturation = %d.\n\n",
										entry2->main_node_id() , entry2->next_node_id() , saturation2 );

							if (  ( entry01->dist() == entry02->dist() && saturation1 > saturation2 ) ||
									( saturation1 == 3 && saturation2 < 3 ) ||
									( entry01->dist() == entry02->dist() && saturation1 == saturation2 && entry01->snr() < entry02->snr() )  ){
								entry = entry2;
								saturation = saturation2;
							}
							else{
								entry = entry1;
								saturation = saturation1;
							}

							fprintf (stdout, "    entry : main = %d, next = %d, next_saturation = %d.\n",
											entry->main_node_id() , entry->next_node_id() ,saturation );
						}
					}
				}
			}
		}


//				assert(entry != NULL);
//				fprintf(stdout, "[time] %f send node id is%d, next node id is  %d\n",CURRENT_TIME,sender_node_id, entry->next_node_id());
//				fprintf(stdout, "  send node id = %d, dest node id = %d, next node id = %d, main node id = %d.\n",
//						sender_node_id, entry->dest_node_id(), entry->next_node_id(), entry->main_node_id());
		return entry->next_node_id();
		//为从当前节点到达目的节点，找到下一跳的节点号
		//这里如果能够找到两条路由表项，即下一跳有两种可能，则可以根据下一跳节点的饱和度来择优选择

//				if (use_mac()) {
//					ch->xmit_failure_	= olsr_mac_failed_callback;
//					ch->xmit_failure_data_	= (void*)this;
//				}
	}

//		Scheduler::instance().schedule(target_, p, 0.0);

}

///
/// \brief Enques an %OLSR message which will be sent with a delay of (0, delay].
///
/// This buffering system is used in order to piggyback several %OLSR messages in
/// a same %OLSR packet.
///
/// \param msg the %OLSR message which must be sent.
/// \param delay maximum delay the %OLSR message is going to be buffered.
///
//void OLSR::enque_msg(OLSR_msg& msg, double delay) {
//由于要返回队列中等待的数据包数量，因此将原先的void函数修改为int函数
int OLSR::enque_msg(OLSR_msg& msg, double delay) {
	fprintf (stdout, "CURRENT TIME = %f , STAGE = enque_msg\n", CURRENT_TIME );

	assert(delay >= 0);

	msgs_.push_back(msg);
	//这里的msgs_和rtable_的定义和使用应该类似，因此可以写一个函数来统计当前等待的包的数目
	//这里直接调用size()函数来得到队列中等待的数据包的数量
	//added by xyy
	int msg_count = 0;
	msg_count = msgs_.size();

//	OLSR_MsgTimer* timer = new OLSR_MsgTimer(this);
	timer_->olsr_msg_timer_= new OLSR_MsgTimer(this);
//	fprintf (stdout ,"%f99999999999999999999999999\n",delay);
	//timer->resched(delay);
    fprintf (stdout,"  ");
	timer_->olsr_msg_timer_->start(delay);

	return msg_count;
}
///
/// \brief Creates as many %OLSR packets as needed in order to send all buffered
/// %OLSR messages.
///
/// Maximum number of messages which can be contained in an %OLSR packet is
/// dictated by OLSR_MAX_MSGS constant.
///
//void
//OLSR::send_pkt() {
//	//struct RouteMacHeader * rmheader = RouteMac_HEADER(p);
//	int num_msgs = msgs_.size();
//	if (num_msgs == 0)
//		return;
//
//	// Calculates the number of needed packets
//	int num_pkts = (num_msgs%OLSR_MAX_MSGS == 0) ? num_msgs/OLSR_MAX_MSGS :
//		(num_msgs/OLSR_MAX_MSGS + 1);
//
//	for (int i = 0; i < num_pkts; i++) {
//		Packet* p		= allocpkt();
//		struct hdr_cmn* ch	= HDR_CMN(p);
//		struct hdr_ip* ih	= HDR_IP(p);
//		OLSR_pkt* op		= PKT_OLSR(p);
//
//		op->pkt_len()		= OLSR_PKT_HDR_SIZE;
//		op->pkt_seq_num()	= pkt_seq();
//
//		int j = 0;
//		for (std::vector<OLSR_msg>::iterator it = msgs_.begin(); it != msgs_.end(); it++) {
//			if (j == OLSR_MAX_MSGS)
//				break;
//
//			op->pkt_body_[j++]	= *it;
//			op->count		= j;
//			op->pkt_len()		+= (*it).size();
//
//			it = msgs_.erase(it);
//			it--;
//		}
//
//		ch->ptype()		= PT_OLSR;
//		ch->direction()		= hdr_cmn::DOWN;
//		ch->size()		= IP_HDR_LEN + UDP_HDR_LEN + op->pkt_len();
//		ch->error()		= 0;
//		ch->next_hop()		= IP_BROADCAST;
//		ch->addr_type()		= NS_AF_INET;
//		if (use_mac()) {
//			ch->xmit_failure_	= olsr_mac_failed_callback;
//			ch->xmit_failure_data_	= (void*)this;
//		}
//
//		ih->saddr()	= ra_addr();
//		ih->daddr()	= IP_BROADCAST;
//		ih->sport()	= RT_PORT;
//		ih->dport()	= RT_PORT;
//		ih->ttl()	= IP_DEF_TTL;
//
//
//
//		Scheduler::instance().schedule(target_, p, 0.0);
//	}
//}

//added by li

/// send packet to tell the complete of quick routing
//void
//OLSR::send_RTcomplete_pkt() {
//	Packet* p		= allocpkt();
//	struct hdr_cmn* ch	= HDR_CMN(p);
////	struct hdr_ip* ih	= HDR_IP(p);
//    RouteMacHeader* cp	= RouteMac_HEADER(p);
//	cp->real_content_flag_ = true;
//	cp->signal_type_ = no_signal;
//
//	cp->buffer_[0]		= cp->buffer_[0] | (char(pkt_seq())& 0x1f)<<3;
//	cp->buffer_[0]		= cp->buffer_[0] | (char(OLSR_RTC_MSG)& 0x03)<<1;
//
//	/// to find the next hop destination
//	int dst_addr_;
//	dst_addr_ = 0 ;
//
//	OLSR_rt_entry* entry = rtable_.lookup(dst_addr_);
//	if (entry == NULL) {
////		debug("%f: Node %d can not forward a packet destined to %d\n",
////			CURRENT_TIME,
////			OLSR::node_id(ra_addr()),
//	//		OLSR::node_id(ih->daddr()));
//		fprintf(stdout,"orignal node %d can not find next mode id \n",ra_node_id());
//			drop(p, DROP_RTR_NO_ROUTE);
//			return;
//	}
//	else {
//		entry = rtable_.find_send_entry(entry);
//		assert(entry != NULL);
//		cp->next_node_id_ = entry->next_node_id();
//		fprintf(stdout,"orignal node %d send RTC to node %d \n",ra_node_id(),cp->next_node_id_);
//	}
//	cp->d_node_id_ = 0;
//	cp->olsr_packet = true;
//
////    ch->ptype()		= PT_OLSR;
//	ch->direction()		= hdr_cmn::DOWN;
////	ch->size()		= IP_HDR_LEN + UDP_HDR_LEN + 1;
////	ch->error()		= 0;
////	ch->next_hop()		= IP_BROADCAST;
////	ch->addr_type()		= NS_AF_INET;
////	if (use_mac()) {
////		ch->xmit_failure_	= olsr_mac_failed_callback;
////		ch->xmit_failure_data_	= (void*)this;
////	}
//
////	ih->saddr()	= ra_addr();
////	ih->daddr()	= dst_addr_;
////	ih->sport()	= RT_PORT;
////	ih->dport()	= RT_PORT;
////	ih->ttl()	= IP_DEF_TTL;
//
//	Scheduler::instance().schedule(target_, p, 0.0);
//}

void OLSR::send_pkt() {
	fprintf (stdout, "CURRENT TIME = %f , STAGE = send_pkt\n", CURRENT_TIME );

// fprintf (stdout, "CURRENT TIME = %f , send the packet \n", CURRENT_TIME);

	int num_msgs = msgs_.size();
	//fprintf(stdout, " %d message size ",num_msgs );
	if (num_msgs == 0)
		return;

	// Calculates the number of needed packets
	int num_pkts = (num_msgs%OLSR_MAX_MSGS == 0) ? num_msgs/OLSR_MAX_MSGS :
		(num_msgs/OLSR_MAX_MSGS + 1);//一个pkt的大小为OLSR_MAX_MSGS，计算这些msg需要多少的pkt

	for (int i = 0; i < num_pkts; i++) {

		int j = 0;
		int node_id_;
		int size_;
		for (std::vector<OLSR_msg>::iterator it = msgs_.begin(); it != msgs_.end(); it++) {
			if (j == OLSR_MAX_MSGS)
				break;
//			fprintf (stdout, "send message type is %d \n", (*it).msg_type());
			if ((*it).msg_type() == OLSR_HELLO_MSG)
			{
				   fprintf (stdout,"  node[%d] send HELLO. \n",  (*it).orig_node_id());
//					Packet* p		= allocpkt();
//					struct hdr_cmn* ch	= HDR_CMN(p);
		//			struct hdr_ip* ih	= HDR_IP(p);
					Route_Mac_Block cp;
	//		OLSR_pkt* op		= PKT_OLSR(p);
	//		op->pkt_len()		= OLSR_PKT_HDR_SIZE;
	//		op->pkt_seq_num()	= pkt_seq();
					cp.real_content_flag_ = true;
					cp.signal_type_ = no_signal;
					cp.olsr_packet = true;
					cp.d_node_id_ = -1;
					cp.next_node_id_ = -1;
					cp.buffer_[0] 	= cp.buffer_[0] | ((pkt_seq())& 0x1f)<<3;
					cp.buffer_[0] = cp.buffer_[0] |(char((*it).msg_type())& 0x01)<<2;
				if (quick_route())
					{
						int vtime_ = 1;
						cp.buffer_[0]	=cp.buffer_[0] | (char(vtime_)& 0x18)>>3;
						cp.buffer_[1]	= cp.buffer_[1] |(char(vtime_)& 0x07)<<5;
					}
					else
					{
						cp.buffer_[0]	=cp.buffer_[0] | (char((*it).vtime())& 0x18)>>3;
						cp.buffer_[1]	= cp.buffer_[1] |(char((*it).vtime())& 0x07)<<5;
					}
					node_id_ = (*it).orig_node_id();

					cp.buffer_[1]	= cp.buffer_[1] |(char(node_id_)& 0x1f);
					cp.buffer_[2]	= cp.buffer_[2] |(char((*it).ttl())& 0x1f)<<3;
					cp.buffer_[2]	= cp.buffer_[2] |(char((*it).hop_count())& 0x1c)>>2;
					cp.buffer_[3]	= cp.buffer_[3] |(char((*it).hop_count())& 0x03)<<6;
					cp.buffer_[3]	= cp.buffer_[3] |(char((*it).msg_seq_num())& 0x1f)<<1;
					cp.buffer_[3]	= cp.buffer_[3] |(char((*it).hello().htime())& 0x80)>>7;
					cp.buffer_[4]	= cp.buffer_[4] |(char((*it).hello().htime())& 0x7f)<<1;
					cp.buffer_[4]  = cp.buffer_[4] |(char((*it).hello().willingness())& 0x02)>>1;
					cp.buffer_[5]  = cp.buffer_[5] |(char((*it).hello().willingness())& 0x01)<<7;


					u_int8_t link_code_;
					int k=0;
					int c=0;
					c=(*it).hello().count;
//					fprintf(stdout,"  This hello message has %d kinds of link_code\n",c);

					u_int8_t nb_type;

					for(int y=0;y<c;y++)
						//			std::vector<OLSR_hello_msg>  hello_msgs_;
						//		    for(std::vector<OLSR_hello_msg>::iterator it2 = hello_msgs_.begin(); it2 != hello_msgs_.end(); it2++)
						//			for(std::vector<OLSR_hello_msg>::iterator it2 = (*it).hello().hello_msg_.begin(); it2 != (*it).hello().hello_msg_.end(); it2++)
					{

							link_code_ = (*it).hello().hello_msg(y).link_code();
							k = (*it).hello().hello_msg(y).count;

							for (int m = 0; m != k; m++)
							{
								node_id_ = (*it).hello().hello_msg(y).nb_main_node_id(m);

								if( node_id_ == (*it).orig_node_id() )
									continue;

//								if(ra_node_id() == 1)
//								{
//									fprintf(stdout,"the nb node id of node 1 is %d \n", node_id_);
//								}
								int n = 0;
								if(node_id_%2)
								{
									n = (node_id_- 1)/2;//奇数
									cp.buffer_[5+n] = cp.buffer_[5+n]  |(char(link_code_)& 0x0E)>>1;
									cp.buffer_[6+n] = cp.buffer_[6+n] | (char(link_code_)& 0x01)<<7;
								}
								else
								{
									n = node_id_/2;//偶数
									cp.buffer_[5+n] =  cp.buffer_[5+n] |(char(link_code_)& 0x0F)<<3;
								}
							}
					}



					int n = 0;
					for(int i = 0; i < 20; i++){
						if( i == (*it).orig_node_id() ){
							if( i%2 ){//奇数
								n = ( i -1 ) / 2 ;
//								cp.buffer_[5+n] = cp.buffer_[5+n]  |(char((*it).hello().mine_cd_node())& 0x0E)>>1;
								cp.buffer_[6+n] = cp.buffer_[6+n] | (char((*it).hello().mine_cd_node())& 0x01)<<7;
							}
							else{//偶数
								n = i / 2;
								cp.buffer_[5+n] =  cp.buffer_[5+n] |(char((*it).hello().mine_cd_node())& 0x01)<<3;//0F
							}
							break;
						}
					}


					//added by xyy
					cp.buffer_[15]  =cp.buffer_[15] | (char((*it).hello().saturation()) & 0x03)<<5;
					cp.buffer_[15]  =cp.buffer_[15] | (char(choose_cd_node_id()) & 0x1f);


				   fprintf (stdout,"  node[%d] is CD node chosen by node[%d]. \n", choose_cd_node_id() , (*it).orig_node_id());


					   size_ = 16;
//					   ch.ptype()		= PT_OLSR;
//					   ch.direction()		= hdr_cmn::DOWN;
//				       ch.size()		= IP_HDR_LEN + UDP_HDR_LEN + size_;
//					   ch.error()		= 0;
//					   ch.next_hop()		= IP_BROADCAST;
//					   ch->addr_type()		= NS_AF_INET;

//				   fprintf (stdout,"   (*it).orig_node_id() = %d. \n", (*it).orig_node_id());
//				   fprintf (stdout,"  node[%d]  send HELLO packet. \n", (*it).orig_node_id());
					cp.packet_type_ = Route_Mac_Block::HELLO;
					send_to_mac(&cp);
//					   Scheduler::instance().schedule(target_, p, 0.0);
				}


			else if ((*it).msg_type() == OLSR_TC_MSG)
			{
//				fprintf (stdout , "TCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCc\n ") ;
//				Packet* p		= allocpkt();
//				struct hdr_cmn* ch	= HDR_CMN(p);
	//			struct hdr_ip* ih	= HDR_IP(p);
//		OLSR_pkt* op		= PKT_OLSR(p);
				Route_Mac_Block  cp;

//		op->pkt_len()		= OLSR_PKT_HDR_SIZE;
//		op->pkt_seq_num()	= pkt_seq();
				cp.real_content_flag_ = true;
				cp.signal_type_ = no_signal;
				cp.olsr_packet = true;
				cp.d_node_id_ = -1;
				cp.next_node_id_ = -1;
				cp.buffer_[0] 	= cp.buffer_[0] | ((pkt_seq())& 0x1f)<<3;
				cp.buffer_[0]	= cp.buffer_[0] |(char((*it).msg_type())& 0x01)<<2;
				if (quick_route())
				{
					int vtime_ = 1;
					cp.buffer_[0]	=cp.buffer_[0] | (char(vtime_)& 0x18)>>3;//>>4
					cp.buffer_[1]	= cp.buffer_[1] |(char(vtime_)& 0x07)<<5;
				}
				else
				{
					cp.buffer_[0]	=cp.buffer_[0] | (char((*it).vtime())& 0x18)>>3;//>>4
					cp.buffer_[1]	= cp.buffer_[1] |(char((*it).vtime())& 0x07)<<5;
				}
				node_id_ = (*it).orig_node_id();
				cp.buffer_[1]	= cp.buffer_[1] |(char(node_id_)& 0x1f);
				cp.buffer_[2]	= cp.buffer_[2] |(char((*it).ttl())& 0x1f)<<3;
				cp.buffer_[2]	= cp.buffer_[2] |(char((*it).hop_count())& 0x1c)>>2;
				cp.buffer_[3]	= cp.buffer_[3] |(char((*it).hop_count())& 0x03)<<6;
				cp.buffer_[3]	= cp.buffer_[3] |(char((*it).msg_seq_num())& 0x1f)<<1;

				cp.buffer_[4]	= cp.buffer_[4] |(char((*it).tc().ansn())& 0xff);
				int l;
				l = (*it).tc().count;
				for (int x = 0; x != l; x++)
				{
					node_id_ = (*it).tc().nb_main_node_id(x);
					int y;
					y = node_id_%8;
					if(node_id_<8)
					{//这里的封装，每个节点都是1--------不对，只会把是node_id的节点置为1，而这里面存放的是mprselset
						if(y==0) { cp.buffer_[5]  =cp.buffer_[5] | 0x80;}
						if(y==1){ cp.buffer_[5]  =cp.buffer_[5]  |0x40;}
						if(y==2){ cp.buffer_[5]  =cp.buffer_[5]  |0x20;}
						if(y==3){ cp.buffer_[5]  =cp.buffer_[5]  |0x10;}
						if(y==4){ cp.buffer_[5]  =cp.buffer_[5]  |0x08;}
						if(y==5){ cp.buffer_[5]  =cp.buffer_[5]  |0x04;}
						if(y==6){ cp.buffer_[5]  =cp.buffer_[5]  |0x02;}
						if(y==7){ cp.buffer_[5]  =cp.buffer_[5]  |0x01;}
					}
					else if(node_id_>15)
					{
						if(y==0){ cp.buffer_[7]  =cp.buffer_[7] |0x80;}
						if(y==1){ cp.buffer_[7]  =cp.buffer_[7] |0x40;}
						if(y==2){ cp.buffer_[7]  =cp.buffer_[7] |0x20;}
						if(y==3){ cp.buffer_[7]  =cp.buffer_[7] |0x10;}
					}
					else
					{
						if(y==0){ cp.buffer_[6]  =cp.buffer_[6]   |0x80;}
						if(y==1){ cp.buffer_[6]    =cp.buffer_[6]   |0x40;}
						if(y==2){ cp.buffer_[6]    =cp.buffer_[6]   |0x20;}
						if(y==3){ cp.buffer_[6]    =cp.buffer_[6]   |0x10;}
						if(y==4){ cp.buffer_[6]    =cp.buffer_[6]   |0x08;}
						if(y==5){ cp.buffer_[6]   = cp.buffer_[6]  |0x04;}
						if(y==6){ cp.buffer_[6]   =cp.buffer_[6]  |0x02;}
						if(y==7){ cp.buffer_[6]   =cp.buffer_[6]  |0x01;}
					}
				}
				//sender address of the tc message
				cp.buffer_[8]  =cp.buffer_[8] | (char(ra_node_id())& 0x1f);

//				//added by xyy
//				cp.buffer_[9]  =cp.buffer_[9] | (char((*it).tc().saturation()) & 0x03);

				size_ = 8;
//				ch.ptype()		= PT_OLSR;
//				ch.direction()		= hdr_cmn::DOWN;
//				ch.size()		= IP_HDR_LEN + UDP_HDR_LEN + size_;
//				ch.error()		= 0;
//				ch.next_hop()		= IP_BROADCAST;
//				ch.addr_type()		= NS_AF_INET;
//				if (use_mac()) {
//					ch->xmit_failure_	= olsr_mac_failed_callback;
//					ch->xmit_failure_data_	= (void*)this;
//				}
//
//				ih->saddr()	= ra_addr();
//				ih->daddr()	= IP_BROADCAST;
//				ih->sport()	= RT_PORT;
//				ih->dport()	= RT_PORT;
//				ih->ttl()	= IP_DEF_TTL;

//				Scheduler::instance().schedule(target_, p, 0.0);
//				fprintf (stdout,"node %d  send tc packet\n",ra_node_id());
				fprintf (stdout,"  node[%d] send TC packet\n",(*it).orig_node_id());
				cp.packet_type_ = Route_Mac_Block::TC;
				fprintf (stdout,"  ");
				send_to_mac(&cp);
			}
//			else if ((*it).msg_type() == OLSR_MID_MSG)
//			{
//				Packet* p		= allocpkt();
//				struct hdr_cmn* ch	= HDR_CMN(p);
//				struct hdr_ip* ih	= HDR_IP(p);
//				OLSR_pkt* op		= PKT_OLSR(p);
//
//				op->pkt_len()		= OLSR_PKT_HDR_SIZE;
//				op->pkt_seq_num()	= pkt_seq();
//
//				op->pkt_body_[j++]	= *it;
//
//
//				op->count		= j;
//				op->pkt_len()		+= (*it).size();
//				size_ 			= op->pkt_len();
//
//				ch->ptype()		= PT_OLSR;
//				ch->direction()		= hdr_cmn::DOWN;
//				ch->size()		= IP_HDR_LEN + UDP_HDR_LEN + size_;
//				ch->error()		= 0;
//				ch->next_hop()		= IP_BROADCAST;
//				ch->addr_type()		= NS_AF_INET;
//				if (use_mac()) {
//					ch->xmit_failure_	= olsr_mac_failed_callback;
//					ch->xmit_failure_data_	= (void*)this;
//				}
//
//				ih->saddr()	= ra_addr();
//				ih->daddr()	= IP_BROADCAST;
//				ih->sport()	= RT_PORT;
//				ih->dport()	= RT_PORT;
//				ih->ttl()	= IP_DEF_TTL;
//
//				Scheduler::instance().schedule(target_, p, 0.0);
//
//			}

			it = msgs_.erase(it);
			it--;
			j=j+1;
		}

//		ch->ptype()		= PT_OLSR;
//		ch->direction()		= hdr_cmn::DOWN;
//		ch->size()		= IP_HDR_LEN + UDP_HDR_LEN + size_;
//		ch->error()		= 0;
//		ch->next_hop()		= IP_BROADCAST;
//		ch->addr_type()		= NS_AF_INET;
//		if (use_mac()) {
//			ch->xmit_failure_	= olsr_mac_failed_callback;
//			ch->xmit_failure_data_	= (void*)this;
//		}
//
//		ih->saddr()	= ra_addr();
//		ih->daddr()	= IP_BROADCAST;
//		ih->sport()	= RT_PORT;
//		ih->dport()	= RT_PORT;
//		ih->ttl()	= IP_DEF_TTL;

//		Scheduler::instance().schedule(target_, p, 0.0);
	}
}

/// complete


void OLSR::send_to_mac(Route_Mac_Block *cp)
{
	fprintf (stdout, "CURRENT TIME = %f , STAGE = send_to_mac\n", CURRENT_TIME );

	if(cp->real_content_flag_ == true)
	{
		if(cp->packet_type_ == Route_Mac_Block::HELLO)
		{
			temp_add_route.frame_type_ = Traf_Queue_Content::HELLO;
		}
		else if(cp->packet_type_ == Route_Mac_Block::TC)
		{
			temp_add_route.frame_type_ = Traf_Queue_Content::TC;
		}
		for(int i=0;i<BUFFER_SIZE;i++)
		{
			temp_add_route.route_frame_content_pt_.content_[i] = cp->buffer_[i];
		}
		traf_queue_add(queue_,&temp_add_route);
		//fprintf(stdout,"[time] %f [node] n(%d) recv route to send [buffer_[0]] %d\n",CURRENT_TIME,node_addr_[2],temp_recv_route_to_send.route_pt_.content_[0]);
		}
		else
		{

		}
}
///
/// \brief Creates a new %OLSR HELLO message which is buffer_ed to be sent later on.
///
void OLSR::send_hello() {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = send_hello\n", CURRENT_TIME );

	OLSR_msg msg;

    fprintf (stdout, "  node[%d] send_hello. \n", msg.orig_node_id() );

	double now		= CURRENT_TIME;
	msg.msg_type()		= OLSR_HELLO_MSG;
//	msg.vtime()		= OLSR::seconds_to_emf(OLSR_NEIGHB_HOLD_TIME);
//	msg.vtime()		= OLSR_NEIGHB_HOLD_TIME;
	msg.vtime()		= Olsr_neighbor_hold_time;
//    fprintf (stdout, "  send vtime is %f \n", msg.vtime());

//	added by li
//	msg.vtime()		= Olsr_neighbor_hold_time;

	msg.orig_node_id()		= ra_node_id();

	//added by li
//	bool updated = false;
//	for (nodeidassocset_t::iterator it = nodeidassocset().begin();
//		it != nodeidassocset().end();
//		it++) {
//		OLSR_nodeid_assoc_tuple* tuple = *it;
//		if (tuple->main_addr() == ra_addr()) {
//			updated		= true;
//		}
//	}
//	if (!updated) {
//		OLSR_nodeid_assoc_tuple* tuple	= new OLSR_nodeid_assoc_tuple;
//		fprintf(stdout, "%d_XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n",
//								  address_number);
//		tuple->node_id()		= address_number;
//		tuple->main_addr()		= ra_addr();
//		add_nodeidassoc_tuple(tuple);
//		fprintf(stdout , "node_id is  %d  , main_addr is  %d , originator addr is %d \n",tuple->node_id(),tuple->main_addr(), ra_addr());
//
//		address_number=address_number+1;
//
//
//	}
  //complete


	msg.ttl()		= 1;
	msg.hop_count()		= 0;
//	fprintf(stdout,"message sequence number is %d\n" ,msg_seq());
	msg.msg_seq_num()	= msg_seq();

	msg.hello().reserved()		= 0;
//	msg.hello().htime()		= OLSR::seconds_to_emf(hello_ival());
//	added by li
//	if (quick_route)
//	{
//		msg.hello().htime()	= OLSR::seconds_to_emf(1);
//	}
//	else
//	{
	//		msg.hello().htime()		=OLSR::seconds_to_emf(hello_ival());
	msg.hello().htime()		=hello_ival();
//	}

//	fprintf (stdout, "%f hello interval\n",hello_ival() );
	msg.hello().willingness()	= OLSR_WILL_DEFAULT;
	msg.hello().count		= 0;

	map<u_int8_t, int> linkcodes_count;



	//added by xyy
	//如果有节点选择本节点为骨干节点，或者本节点选择自己为骨干节点，则标记HELLO消息
//	if ( local_cd == 1 || choose_cd_node_id() == ra_node_id() )
	if( state_.find_cd_node_id( ra_node_id() ) )
		msg.hello().mine_cd_node() = 1;
	else
		msg.hello().mine_cd_node() = 0;



//	int local_cd = 0;
	for (linkset_t::iterator it = linkset().begin(); it != linkset().end(); it++) {
		OLSR_link_tuple* link_tuple = *it;


//		//added by xyy
//		//如果本节点推选自己为骨干接点，或者被其他节点选为骨干节点
//		if ( it == linkset().begin() ){
//			if ( link_tuple->local_main_node_cd() == 1)// ||
////					 state_.find_cd_node_id( link_tuple->local_main_node_id() ) == 1)
//				msg.hello().cd_node() = 1;
//			else
//				msg.hello().cd_node() = 0;
//			fprintf(stdout, "  msg.orig_node_id = [%d], msg.hello().cd_node = %d .\n",msg.orig_node_id(), msg.hello().cd_node() );
//		}
//		//如果有节点选择本节点为骨干节点，则标记local_cd标志位
//		if( link_tuple->local_main_node_cd() ==1 )
//			local_cd = 1;


		if (link_tuple->local_main_node_id() == ra_node_id() && link_tuple->time() >= now) {
			u_int8_t link_type, nb_type, link_code;
//			fprintf (stdout, "time TTTTTTTTTTTTTTtttt is %f \n", link_tuple->asym_time() - now);

			// Establishes link type
			if ( link_tuple->lost_time() >= now)
				link_type = OLSR_LOST_LINK;
			else if (link_tuple->sym_time() >= now)
				link_type = OLSR_SYM_LINK;
			else if (link_tuple->asym_time() >= now)
				link_type = OLSR_ASYM_LINK;
			else
				link_type = OLSR_LOST_LINK;

			// Establishes neighbor type.
			if (state_.find_mpr_node_id( link_tuple->nb_main_node_id()) ){
				nb_type = OLSR_MPR_NEIGH;//是MPR的骨干节点
				fprintf (stdout,"  node[%d] is MPR node chosen by node[%d].\n",  link_tuple->nb_main_node_id(),  link_tuple->local_main_node_id() );
			}

//			//added by xyy
//			//HELLO消息中只会标记该节点是不是骨干节点，以及该节点推选哪个邻居节点为骨干节点
////			else if ( msg.hello().cd_node() == 0 &&	state_.find_cd_node_id(link_tuple->nb_main_node_id()) )
////			else if ( link_tuple->nb_main_node_cd() == 1 ){
//			//如果本节点选择的骨干节点被选为MPR节点，则在这之前就会被标记为MPR_NEIGH
//			//被标记为CD的是在这次本节点推选骨干节点过程中没有被选为MPR的骨干节点
//			else if ( link_tuple->nb_main_node_id() == choose_cd_node_id() ){
//				nb_type = OLSR_CD_NEIGH;//不是MPR的骨干节点
//				fprintf (stdout,"  node[%d] is CD node chosen by node[%d].\n",  link_tuple->nb_main_node_id(),  link_tuple->local_main_node_id() );
//			}

			else {
				bool ok = false;
				for (nbset_t::iterator nb_it = nbset().begin();
					nb_it != nbset().end();
					nb_it++) {
					OLSR_nb_tuple* nb_tuple = *nb_it;
					if (nb_tuple->nb_main_node_id() == link_tuple->nb_main_node_id()) {
						if (nb_tuple->status() == OLSR_STATUS_SYM)
							nb_type = OLSR_SYM_NEIGH;
						else if (nb_tuple->status() == OLSR_STATUS_NOT_SYM)
							nb_type = OLSR_NOT_NEIGH;
						else {
							fprintf(stderr, "  There is a neighbor tuple"
								" with an unknown status!\n");
							exit(1);
						}
						ok = true;
						break;
					}
				}
				if (!ok) {
					fprintf(stderr, "  Link tuple has no corresponding"
						" Neighbor tuple\n");
					exit(1);
				}
			}

			int count = msg.hello().count;
			link_code = (link_type & 0x03) | ((nb_type << 2) & 0x0f);
//			fprintf(stdout,"send link_code is %d \n",link_code);
			map<u_int8_t, int>::iterator pos = linkcodes_count.find(link_code);
			if (pos == linkcodes_count.end()) {
				linkcodes_count[link_code] = count;
				assert(count >= 0 && count < OLSR_MAX_HELLOS);
				msg.hello().hello_msg(count).count = 0;
				msg.hello().hello_msg(count).link_code() = link_code;
				msg.hello().hello_msg(count).reserved() = 0;
				msg.hello().count++;
			}
			else
				count = (*pos).second;

			int i = msg.hello().hello_msg(count).count;
			assert(count >= 0 && count < OLSR_MAX_HELLOS);
			assert(i >= 0 && i < OLSR_MAX_ADDRS);

			msg.hello().hello_msg(count).nb_main_node_id(i) =
				link_tuple->nb_main_node_id();
			msg.hello().hello_msg(count).count++;
			msg.hello().hello_msg(count).link_msg_size() =
				msg.hello().hello_msg(count).size();

		}
	}

	msg.msg_size() = msg.size();


	//added by xyy
	int saturation = 0;
	if (quick_route())
		{
		    fprintf (stdout,"  ");
			saturation = enque_msg(msg, jitter);
		    fprintf (stdout, "  quick_route : saturation = %d\n", saturation);
		}
		else
		{

			fprintf (stdout,"  ");
			saturation = enque_msg(msg, JITTER);
		    fprintf (stdout, "  normal : saturation = %d\n", saturation);
		}

	if (saturation < 5)
		msg.hello().saturation() = 1;//空闲
	else if (saturation > 5 && saturation < 20)
		msg.hello().saturation() = 2;//一般
	else
			msg.hello().saturation() = 3;//繁忙


//	if (saturation < 2)
//		msg.hello().saturation() = 1;//空闲
//	else if (saturation > 1 && saturation < 3)
//		msg.hello().saturation() = 2;//一般
//	else
//			msg.hello().saturation() = 3;//繁忙


    fprintf (stdout, "  saturation level = %d\n", msg.hello().saturation());

}

///
/// \brief Creates a new %OLSR TC message which is buffer_ed to be sent later on.
///
void OLSR::send_tc() {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = send_tc\n", CURRENT_TIME );

//	fprintf(stdout,"TCCCCCCCCCCCCCCCCCCCCCCCCCCCCCc\n");
	OLSR_msg msg;
	msg.msg_type()		= OLSR_TC_MSG;
//	msg.vtime()		= OLSR::seconds_to_emf(OLSR_TOP_HOLD_TIME);
//	msg.vtime()		= OLSR_TOP_HOLD_TIME;
	msg.vtime()		= Olsr_top_hold_time;

//    fprintf (stdout, "  send vtime is %f \n",msg.vtime());
//	added by i
//	msg.vtime()		= Olsr_top_hold_time;
	msg.orig_node_id()		= ra_node_id();
	msg.ttl()		= 19;
	msg.hop_count()		= 0;
	msg.msg_seq_num()	= msg_seq();

	msg.tc().ansn()		= ansn_;
	msg.tc().reserved()	= 0;
	msg.tc().count		= 0;

	for (mprselset_t::iterator it = mprselset().begin(); it != mprselset().end(); it++) {
		OLSR_mprsel_tuple* mprsel_tuple = *it;
		int count = msg.tc().count;

		assert(count >= 0 && count < OLSR_MAX_ADDRS);
		msg.tc().nb_main_node_id(count) = mprsel_tuple->main_node_id();
		msg.tc().count++;
	}

	msg.msg_size()		= msg.size();
//		fprintf(stdout,"TCCCCCCCCCCCCCCCCCCCCCCCCCCCCCc\n");

	//added by xyy
	int saturation = 0;
	if (quick_route())
		{
			fprintf (stdout, "  " );
			saturation = enque_msg(msg, jitter);
		    fprintf (stdout, "  quick_route : saturation = %d\n", saturation);
		}
		else
		{
			fprintf (stdout,"  ");
			saturation = enque_msg(msg, JITTER);
		    fprintf (stdout, "  normal : saturation = %d\n", saturation);
		}


//		if (saturation < 5)
//			msg.tc().saturation() = 1;//空闲
//		else if (saturation > 5 && saturation < 20)
//			msg.tc().saturation() = 2;//一般
//		else
//				msg.tc().saturation() = 3;//繁忙
//
//	    fprintf (stdout, "  saturation level = %d\n", msg.tc().saturation());

}

///
/// \brief Creates a new %OLSR MID message which is buffered to be sent later on.
/// \warning This message is never invoked because there is no support for multiple interfaces.
///
//void
//OLSR::send_mid() {
//	OLSR_msg msg;
//	msg.msg_type()		= OLSR_MID_MSG;
//	msg.vtime()		= OLSR::seconds_to_emf(OLSR_MID_HOLD_TIME);
//	msg.orig_addr()		= ra_addr();
//	msg.ttl()		= 255;
//	msg.hop_count()		= 0;
//	msg.msg_seq_num()	= msg_seq();
//
//	msg.mid().count		= 0;
//	//foreach iface in this_node do
//	//	msg.mid().iface_addr(i) = iface
//	//	msg.mid().count++
//	//done
//
//	msg.msg_size()		= msg.size();
//
//	enque_msg(msg, JITTER);
//}

///
/// \brief	Updates Link Set according to a new received HELLO message (following RFC 3626
///		specification). Neighbor Set is also updated if needed.
///
/// \param msg the OLSR message which contains the HELLO message.
/// \param receiver_iface the address of the interface where the message was received from.
/// \param sender_iface the address of the interface where the message was sent from.
///5.4.1 链路检测
void OLSR::link_sensing(OLSR_msg& msg, int receiver_main, int sender_main) {
	//    link_sensing(msg,  receiver_main_node_id, sender_main_node_id);   //链路检测
	// 	process_hello(msg, ra_node_id(),msg.orig_node_id());
    fprintf (stdout, "CURRENT TIME = %f , STAGE = link_sensing\n", CURRENT_TIME );
    fprintf (stdout, "\n  sender_node_id = [%d], receicer_node_id = [%d] \n", sender_main, receiver_main );

	OLSR_hello& hello	= msg.hello();
	double now		= CURRENT_TIME;
	bool updated		= false;
	bool created		= false;

	//如果在链路信息表中找不到这样的信息：L_neighbor_iface_addr==Source Address
	OLSR_link_tuple* link_tuple = state_.find_link_tuple(sender_main);
	if (link_tuple == NULL) {
		fprintf(stdout,"  Can't find an old link_tuple, now create a new one!\n");
		// We have to create a new tuple
		link_tuple = new OLSR_link_tuple;
		link_tuple->nb_main_node_id()	= sender_main;
		link_tuple->nb_main_node_cd() = 0;
		link_tuple->nb_main_node_nbnum() = 0;//初始化
		link_tuple->local_main_node_id()	= receiver_main;
		link_tuple->local_main_node_cd() = 0;//初始化
		link_tuple->local_main_node_nbnum() = 0;//初始化
		link_tuple->sym_time()		= now - 1;//expire
		link_tuple->saturation() = hello.saturation();
		link_tuple->lost_time()		= 0.0;
//		fprintf (stdout, " receive  vtime is %f \n", msg.vtime());
//		link_tuple->time()		= now + OLSR::emf_to_seconds(msg.vtime());
//		added by li
		link_tuple->time()		= now + msg.vtime();
//		fprintf (stdout, " receive  vtime is %f \n", link_tuple->time());
	    fprintf (stdout,"  ");
		add_link_tuple(link_tuple, hello.willingness());
		created = true;

	}
	else{//如果能够找到，那么就更新列表
		fprintf(stdout,"  There is an old link_tuple, now update it!\n");
		updated = true;
	}
	//1、现进行这样的更新
//	link_tuple->asym_time() = now + OLSR::emf_to_seconds(msg.vtime());
//	    added by li
	link_tuple->asym_time() = now + msg.vtime();

//	fprintf(stdout,"  hello.count = %d\n",hello.count);
	assert(hello.count >= 0 && hello.count <= OLSR_MAX_HELLOS);


//	// added by xyy
//	if ( hello.cd_node() == 1 && state_.find_cd_node_id( sender_main ) == 0 )
//		state_.insert_cd_node_id( sender_main );//如果CD表中没有该节点，则加入
//	else if ( hello.cd_node() == 0 && state_.find_cd_node_id( sender_main ) == 1 )
//		state_.erase_cd_node_id( sender_main );// 如果该节点已经不是骨干节点，则从表中删除
	//这里只是标志发送该HELLO消息的节点是否自己推选自己为骨干节点
	//接收到HELLO消息只会记录发送该节点的信息，以及在它那边自己的信息
	if ( hello.mine_cd_node() == 1 ){
		link_tuple->nb_main_node_cd() = 1;
		fprintf (stdout,"  node[%d] received HELLO : node[%d] is CD node told by node[%d].\n",
				receiver_main, link_tuple->nb_main_node_id(), sender_main);
	}
	else if ( hello.mine_cd_node() == 0 ){
		link_tuple->nb_main_node_cd() = 0;
		fprintf (stdout,"  node[%d] received HELLO : node[%d] isn't CD node told by node[%d].\n",
				receiver_main, link_tuple->nb_main_node_id(), sender_main);
	}

	//这里只是标志发送该HELLO消息的节点是否将接收该HELLO消息的节点选为骨干节点
	//只要接收的HELLO消息中有一个推选自己为骨干节点就标记自己是骨干节点
	//由于初始化的时候是0，所以如果中间没有节点将其赋值为1，就会一直保持为0
	fprintf (stdout,"  node[%d] received HELLO : node[%d] is  CD node chosen by node[%d] .\n",
			receiver_main,hello.choose_cd_node_id(), sender_main);

	if ( hello.choose_cd_node_id() == receiver_main ){
		fprintf (stdout,"  node[%d] received HELLO : node[%d] is CD node told by node[%d].\n",
				receiver_main, receiver_main, sender_main);
		link_tuple->local_main_node_cd() = 1;
	}
	else{
		fprintf (stdout,"  node[%d] received HELLO : node[%d] isn't CD node told by node[%d].\n",
				receiver_main, receiver_main, sender_main);
		link_tuple->local_main_node_cd() = 0;
	}



	for (int i = 0; i < hello.count; i++) {
		OLSR_hello_msg& hello_msg = hello.hello_msg(i);
		fprintf (stdout, "  nb_main_node_id = [%d], ",hello_msg.nb_main_node_id(i));

		int nt = hello_msg.link_code() >> 2;//Neighbor Type

		if ( nt ==OLSR_NOT_NEIGH )
			fprintf (stdout, "Neighbor Type = NOT_NEIGH, ");
		else if ( nt ==OLSR_SYM_NEIGH )
			fprintf (stdout, "Neighbor Type = SYM_NEIGH, ");
		else if ( nt ==OLSR_MPR_NEIGH )
			fprintf (stdout, "Neighbor Type = MPR_NEIGH, ");
//		else if ( nt ==OLSR_CD_NEIGH )
//			fprintf (stdout, "Neighbor Type = CD_NEIGH, ");
//		fprintf (stdout, "  Neighbor Type = %d \n", nt);
//		fprintf (stdout, " YYYYYYYYYYYYYYYYYYYYYY%d \n", nt);


		int lt = hello_msg.link_code() & 0x03;//Link Type

		if ( lt == OLSR_UNSPEC_LINK )
			fprintf (stdout, "Link Type = UNSPEC_LINK. \n");
		else if ( lt == OLSR_ASYM_LINK )
			fprintf (stdout, "Link Type = ASYM_LINK. \n");
		else if ( lt == OLSR_SYM_LINK )
			fprintf (stdout, "Link Type = SYM_LINK. \n");
		else if ( lt == OLSR_LOST_LINK )
			fprintf (stdout, "Link Type = LOST_LINK. \n");
		//		fprintf (stdout, "  Link Type = %d \n", lt);


		// We must not process invalid advertised links
		if ((lt == OLSR_SYM_LINK && nt == OLSR_NOT_NEIGH) ||//链路对称，但没有对称邻居
			  ( nt != OLSR_NOT_NEIGH && nt != OLSR_SYM_NEIGH && nt != OLSR_MPR_NEIGH )){//} &&  nt != OLSR_CD_NEIGH )){//Neighbor Type不是三种类型之一
			continue;
		}

//		fprintf (stdout, " message count is  %d \n",hello_msg.count);
		assert(hello_msg.count >= 0 && hello_msg.count <= OLSR_MAX_ADDRS);

		//如果发现HELLO消息中包含自己的地址，则进行如下更新
		for (int j = 0; j < hello_msg.count; j++) {


//			fprintf (stdout, "  HELLO_orig_node_id = [%d], nb_main_node_id = [%d]. \n",sender_main, hello_msg.nb_main_node_id(j));
//
//			if ( nt ==OLSR_NOT_NEIGH )
//				fprintf (stdout, "  Neighbor Type = NOT_NEIGH, ");
//			else if ( nt ==OLSR_SYM_NEIGH )
//				fprintf (stdout, "  Neighbor Type = SYM_NEIGH, ");
//			else if ( nt ==OLSR_MPR_NEIGH )
//				fprintf (stdout, "  Neighbor Type = MPR_NEIGH, ");
//			else if ( nt ==OLSR_CD_NEIGH )
//				fprintf (stdout, "  Neighbor Type = CD_NEIGH, ");
//
//			if ( lt == OLSR_UNSPEC_LINK )
//				fprintf (stdout, "Link Type = UNSPEC_LINK. \n");
//			else if ( lt == OLSR_ASYM_LINK )
//				fprintf (stdout, "Link Type = ASYM_LINK. \n");
//			else if ( lt == OLSR_SYM_LINK )
//				fprintf (stdout, "Link Type = SYM_LINK. \n");
//			else if ( lt == OLSR_LOST_LINK )
//				fprintf (stdout, "Link Type = LOST_LINK. \n");
//

			if (hello_msg.nb_main_node_id(j) == receiver_main) {

				if (lt == OLSR_LOST_LINK) {
					link_tuple->sym_time() = now - 1;//expire
					updated = true;
				}
				else if (lt == OLSR_SYM_LINK || lt == OLSR_ASYM_LINK) {
//					link_tuple->sym_time()	=
//						now + OLSR::emf_to_seconds(msg.vtime());
//					added by li

					link_tuple->sym_time()	=
						now + msg.vtime();
					link_tuple->time()	=
//						link_tuple->sym_time() + OLSR_NEIGHB_HOLD_TIME;
						link_tuple->sym_time() + Olsr_neighbor_hold_time;
					link_tuple->lost_time()	= 0.0;
					updated = true;
				}
				break;
			}
		}

	}
	link_tuple->time() = MAX(link_tuple->time(), link_tuple->asym_time());

	if (updated){
	    fprintf (stdout,"  ");
		updated_link_tuple(link_tuple);
	}
//	for(int ii=0; ii<20; ii++){
//		if(receiver_main == ii){
//			fprintf(stdout,"  link_sensing : node[%d] -- Neighbor -- node[%d] \n", sender_main, ii );
//		}
//	}

	// Schedules link tuple deletion
	if (created && link_tuple != NULL) {
//		fprintf (stdout ,"%f10101010101010101010101010\n",DELAY(MIN(link_tuple->time(), link_tuple->sym_time())));
		timer_->olsr_linktuple_timer_ = new OLSR_LinkTupleTimer(this, link_tuple);

//       timer_->olsr_linktuple_timer_->start(0.0);

	    fprintf (stdout,"  ");
		timer_->olsr_linktuple_timer_->start(DELAY(MIN(link_tuple->time(), link_tuple->sym_time())));
	}
//		if (link_tuple->local_main_node_id() == 6){
//			fprintf (stdout, "nbset is %d \n" ,link_tuple->nb_main_node_id()	);
//		}
}

///
/// \brief	Updates the Neighbor Set according to the information contained in a new received
///		HELLO message (following RFC 3626).
///
/// \param msg the %OLSR message which contains the HELLO message.
///5.4.2.1 邻居检测——一跳邻居计算
//每个节点基于本地链路信息表计算邻居信息表
//本地信息表和邻居信息表要同时更新
//当且仅当两个节点之间至少存在一跳链路时，两个节点才能成为邻居
void OLSR::populate_nbset(OLSR_msg& msg) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = populate_nbset\n", CURRENT_TIME );

	OLSR_hello& hello = msg.hello();

	OLSR_nb_tuple* nb_tuple = state_.find_nb_tuple(msg.orig_node_id());
//	if (ra_node_id() == 1){
//		fprintf (stdout, "nbset is %d \n" ,nb_tuple->nb__main_node_id_);
//	}
	if (nb_tuple != NULL){
//		fprintf (stdout,"nb_tuple is not NULLL\n");
		nb_tuple->willingness() = hello.willingness();
		nb_tuple->saturation() = hello.saturation();
	}
}

///
/// \brief	Updates the 2-hop Neighbor Set according to the information contained in a new
///		received HELLO message (following RFC 3626).
///
/// \param msg the %OLSR message which contains the HELLO message.
///5.4.2.2 邻居检测——二跳邻居计算
void OLSR::populate_nb2hopset(OLSR_msg& msg) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = populate_nb2hopset\n", CURRENT_TIME );

	double now		= CURRENT_TIME;
	OLSR_hello& hello	= msg.hello();
	
	for (linkset_t::iterator it_lt = linkset().begin(); it_lt != linkset().end(); it_lt++) {
		OLSR_link_tuple* link_tuple = *it_lt;
		if (link_tuple->nb_main_node_id() == msg.orig_node_id()) {
			if (link_tuple->sym_time() >= now) {
				assert(hello.count >= 0 && hello.count <= OLSR_MAX_HELLOS);
				for (int i = 0; i < hello.count; i++) {

					OLSR_hello_msg& hello_msg = hello.hello_msg(i);
					int nt = hello_msg.link_code() >> 2;//neighbor type
					assert(hello_msg.count >= 0 && hello_msg.count <= OLSR_MAX_ADDRS);

					for (int j = 0; j < hello_msg.count; j++) {
						int nb2hop_node_id = hello_msg.nb_main_node_id(j);
						if (nt == OLSR_SYM_NEIGH || nt == OLSR_MPR_NEIGH ){
//						if (nt == OLSR_SYM_NEIGH || nt == OLSR_MPR_NEIGH || nt == OLSR_CD_NEIGH) {
							// if the main address of the 2-hop	neighbor address != main address of the receiving node: create a nb2hop_tuple;
							// else,  silently discard the 2-hop neighbor address
							if (nb2hop_node_id != ra_node_id()) {
								// Otherwise, a 2-hop tuple is created
								OLSR_nb2hop_tuple* nb2hop_tuple =	state_.find_nb2hop_tuple(msg.orig_node_id(), nb2hop_node_id);
								if (nb2hop_tuple == NULL) {
//									if(ra_node_id() == 0){
////										fprintf(stdout," node originator address of node 0 is %d \n",msg.orig_node_id());
//										for (int j = 0; j < hello_msg.count; j++) {
//											fprintf(stdout,"  address of node 0 is %d \n",hello_msg.nb_main_node_id(j));
//										}
//									}
									nb2hop_tuple =
										new OLSR_nb2hop_tuple;
									nb2hop_tuple->nb_main_node_id() =  msg.orig_node_id();
									nb2hop_tuple->nb2hop_node_id() =	nb2hop_node_id;
								    fprintf (stdout,"  ");
									add_nb2hop_tuple(nb2hop_tuple);

//									if(ra_node_id() == 0)
//									{
//										fprintf(stdout,"nb2 node id of node 1 is %d \n",nb2hop_tuple->nb2hop_node_id());
//									}
//									nb2hop_tuple->time() =
//										now + OLSR::emf_to_seconds(msg.vtime());
//									added by li
								   nb2hop_tuple->time() = now + msg.vtime();
									// Schedules nb2hop tuple
									// deletion
									timer_->olsr_nb2hoptuple_timer_ = new OLSR_Nb2hopTupleTimer(this, nb2hop_tuple);

//									fprintf (stdout ,"%f11 11 11 11 11 11 11 11 11 11 11 11 11\n",DELAY(nb2hop_tuple->time()));
								    fprintf (stdout,"  ");
									timer_->olsr_nb2hoptuple_timer_ ->start(DELAY(nb2hop_tuple->time()));
								}
								else {
//									nb2hop_tuple->time() =
	//									now + OLSR::emf_to_seconds(msg.vtime());
//									added by li
									nb2hop_tuple->time() = now + msg.vtime();
								}

							}
						}
						//该记录被删除
						else if (nt == OLSR_NOT_NEIGH) {
							// For each 2-hop node listed in the HELLO
							// message with Neighbor Type equal to
							// NOT_NEIGH all 2-hop tuples where:
							// N_neighbor_main_addr == Originator
							// Address AND N_2hop_addr  == main address
							// of the 2-hop neighbor are deleted.
							state_.erase_nb2hop_tuples(msg.orig_node_id(),
								nb2hop_node_id);
						}
					}
				}
			}
		}
	}
//	if (msg.orig_node_id() == 1){
//		fprintf (stdout, "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEe\n");
//	        for (nb2hopset_t::iterator it = nb2hopset().begin(); it != nb2hopset().end(); it++) {
//		        	OLSR_nb2hop_tuple* nb2hop_tuple = *it;
//
//
//
//				fprintf (stdout, "neibor node id is  %d \n", nb2hop_tuple->nb2hop_node_id());
//			}
//	}
}

///
/// \brief	Updates the MPR Selector Set according to the information contained in a new
///		received HELLO message (following RFC 3626).
///
/// \param msg the %OLSR message which contains the HELLO message.
///5.4.2.4 邻居检测——MPR Selector计算
void OLSR::populate_mprselset(OLSR_msg& msg) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = populate_mprselset\n", CURRENT_TIME );

	for (mprselset_t::iterator it = mprselset().begin(); it != mprselset().end(); it++) {
		OLSR_mprsel_tuple* mprsel_tuple = *it;
		fprintf (stdout, "  mpr_node_id = %d, mprsel_node_id = %d .\n", ra_node_id(), mprsel_tuple->main_node_id());
	}

	double now		= CURRENT_TIME;
	OLSR_hello& hello	= msg.hello();

	assert(hello.count >= 0 && hello.count <= OLSR_MAX_HELLOS);
	for (int i = 0; i < hello.count; i++) {
		OLSR_hello_msg& hello_msg = hello.hello_msg(i);
		int nt = hello_msg.link_code() >> 2;//neighbor type
		if (nt == OLSR_MPR_NEIGH) {//如果neighbor type是MPR_NEIGH，那么就加入MPRselector表中
			assert(hello_msg.count >= 0 && hello_msg.count <= OLSR_MAX_ADDRS);
			for (int j = 0; j < hello_msg.count; j++) {
				if (hello_msg.nb_main_node_id(j) == ra_node_id()) {
					//ra_node_id指的是收到hello消息的节点号，它在hello消息中的neighbor_type是MPR_NEIGH
					// We must create a new entry into the mpr selector set
					OLSR_mprsel_tuple* mprsel_tuple =	state_.find_mprsel_tuple(msg.orig_node_id());
					if (mprsel_tuple == NULL) {
						mprsel_tuple = new OLSR_mprsel_tuple;
						mprsel_tuple->main_node_id() = msg.orig_node_id();
						fprintf (stdout,"  mprsel_tuple->main_node_id() = %d\n", mprsel_tuple->main_node_id());
	//					mprsel_tuple->time() =
	//						now + OLSR::emf_to_seconds(msg.vtime());

//					added by li
						mprsel_tuple->time() = now + msg.vtime();
					    fprintf (stdout,"  ");
						add_mprsel_tuple(mprsel_tuple);
						// Schedules mpr selector tuple deletion
						timer_->olsr_mprseltuple_timer_ =	new OLSR_MprSelTupleTimer(this, mprsel_tuple);

//						fprintf (stdout ,"%f13 13 13 13 13 13 13 13 13 13 13 13 13\n",DELAY(mprsel_tuple->time()));
					    fprintf (stdout,"  ");
						timer_->olsr_mprseltuple_timer_ ->start(DELAY(mprsel_tuple->time()));
					}
					else
//						mprsel_tuple->time() =
//							now + OLSR::emf_to_seconds(msg.vtime());
//						added by li
						mprsel_tuple->time() =
							 now + msg.vtime();
				}
			}
		}
	}

}

///
/// \brief	Drops a given packet because it couldn't be delivered to the corresponding
///		destination by the MAC layer. This may cause a neighbor loss, and appropiate
///		actions are then taken.
///
/// \param p the packet which couldn't be delivered by the MAC layer.
///
//void
//OLSR::mac_failed(Packet* p) {
//	double now		= CURRENT_TIME;
//	struct hdr_ip* ih	= HDR_IP(p);
//	struct hdr_cmn* ch	= HDR_CMN(p);

//	debug("%f: Node %d MAC Layer detects a breakage on link to %d\n",
//		now,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(ch->next_hop()));

//	if ((u_int32_t)ih->daddr() == IP_BROADCAST) {
//		drop(p, DROP_RTR_MAC_CALLBACK);
//		return;
//	}
//
//	OLSR_link_tuple* link_tuple = state_.find_link_tuple(ch->next_hop());
//	if (link_tuple != NULL) {
////		link_tuple->lost_time()	= now + OLSR_NEIGHB_HOLD_TIME;
////		link_tuple->time()	= now + OLSR_NEIGHB_HOLD_TIME;
//		link_tuple->lost_time()	= now + Olsr_neighbor_hold_time;
//		link_tuple->time()	= now + Olsr_neighbor_hold_time;
//		nb_loss(link_tuple);
//	}
//	drop(p, DROP_RTR_MAC_CALLBACK);
//}

///
/// \brief Schedule the timer used for sending HELLO messages.
///
void OLSR::set_hello_timer() {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = set_hello_timer\n", CURRENT_TIME );

	if (quick_route()){
////		fprintf (stdout ,"%hello interval quick route\n",hello_ival());
		fprintf (stdout,"  ");
		timer_->olsr_hello_timer_->start((double)(hello_ival() - jitter));
	}
	else{
//		fprintf (stdout ,"%fhello interval  complete the quick route\n",hello_ival());
		fprintf (stdout,"  ");
		timer_->olsr_hello_timer_->start((double)(hello_ival() - JITTER));
	}
}

///
/// \brief Schedule the timer used for sending TC messages.
///
void OLSR::set_tc_timer() {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = set_tc_timer\n", CURRENT_TIME );

	if (quick_route()){
////		fprintf (stdout ,"%f tc interval quick route\n",tc_ival());
//		tc_timer_.resched((double)(tc_ival() - jitter));
		fprintf (stdout,"  ");
		timer_->olsr_tc_timer_->start((double)(tc_ival() - jitter));
	}
	else{
//		fprintf (stdout ,"%f tc interval complete the quick route\n",tc_ival());
//		tc_timer_.resched((double)(tc_ival() - JITTER));
		fprintf (stdout,"  ");
		timer_->olsr_tc_timer_->start((double)(tc_ival() - JITTER));
	}

}

///
/// \brief Schedule the timer used for sending MID messages.
///
//void
//OLSR::set_mid_timer() {
//	mid_timer_.resched((double)(mid_ival() - JITTER));
//}

///
/// \brief Performs all actions needed when a neighbor loss occurs.
///
/// Neighbor Set, 2-hop Neighbor Set, MPR Set and MPR Selector Set are updated.
///
/// \param tuple link tuple with the information of the link to the neighbor which has been lost.
///
void OLSR::nb_loss(OLSR_link_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = nb_loss\n", CURRENT_TIME );

//	debug("%f: Node %d detects neighbor %d loss\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->nb_main_node_id()));

    fprintf (stdout,"  ");
	updated_link_tuple(tuple);
	state_.erase_nb2hop_tuples(tuple->nb_main_node_id());
	state_.erase_mprsel_tuples(tuple->nb_main_node_id());

	//added by xyy
    fprintf (stdout,"  ");
	UCDS_CD_computation();

    fprintf (stdout,"  ");
//	mpr_computation();
	UCDS_mpr_computation();
    fprintf (stdout,"  ");
 	rtable_computation();
}

///
/// \brief Adds a duplicate tuple to the Duplicate Set.
///
/// \param tuple the duplicate tuple to be added.
///
void OLSR::add_dup_tuple(OLSR_dup_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = add_dup_tuple\n", CURRENT_TIME );

	/*debug("%f: Node %d adds dup tuple: addr = %d seq_num = %d\n",
		CURRENT_TIME,
		OLSR::node_id(ra_addr()),
		OLSR::node_id(tuple->addr()),
		tuple->seq_num());*/

	state_.insert_dup_tuple(tuple);
}

///
/// \brief Removes a duplicate tuple from the Duplicate Set.
///
/// \param tuple the duplicate tuple to be removed.
///
void OLSR::rm_dup_tuple(OLSR_dup_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = rm_dup_tuple\n", CURRENT_TIME );

	/*debug("%f: Node %d removes dup tuple: addr = %d seq_num = %d\n",
		CURRENT_TIME,
		OLSR::node_id(ra_addr()),
		OLSR::node_id(tuple->addr()),
		tuple->seq_num());*/

	state_.erase_dup_tuple(tuple);
}

///
/// \brief Adds a link tuple to the Link Set (and an associated neighbor tuple to the Neighbor Set).
///
/// \param tuple the link tuple to be added.
/// \param willingness willingness of the node which is going to be inserted in the Neighbor Set.
///
void OLSR::add_link_tuple(OLSR_link_tuple* tuple, u_int8_t  willingness) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = add_link_tuple\n", CURRENT_TIME );

	double now = CURRENT_TIME;

//	debug("%f: Node %d adds link tuple: nb_addr = %d\n",
//		now,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->nb_main_node_id()));

	state_.insert_link_tuple(tuple);
	// Creates associated neighbor tuple
	OLSR_nb_tuple* nb_tuple		= new OLSR_nb_tuple;
	nb_tuple->nb_main_node_id()	= tuple->nb_main_node_id();
	nb_tuple->willingness()		= willingness;
	if (tuple->sym_time() >= now)
		nb_tuple->status() = OLSR_STATUS_SYM;
	else
		nb_tuple->status() = OLSR_STATUS_NOT_SYM;

    fprintf (stdout,"  ");
    add_nb_tuple(nb_tuple);
}

///
/// \brief Removes a link tuple from the Link Set.
///
/// \param tuple the link tuple to be removed.
///
void OLSR::rm_link_tuple(OLSR_link_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = rm_link_tuple\n", CURRENT_TIME );

	int nb_node_id= tuple->nb_main_node_id();
	double now		= CURRENT_TIME;

//	debug("%f: Node %d removes link tuple: nb_addr = %d\n",
//		now,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->nb_main_node_id()));
	// Prints this here cause we are not actually calling rm_nb_tuple() (efficiency stuff)
//	debug("%f: Node %d removes neighbor tuple: nb_addr = %d\n",
//		now,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(nb_node_id));

	state_.erase_link_tuple(tuple);

	OLSR_nb_tuple* nb_tuple = state_.find_nb_tuple(nb_node_id);
	state_.erase_nb_tuple(nb_tuple);
	delete nb_tuple;
}

///
/// \brief	This function is invoked when a link tuple is updated. Its aim is to
///		also update the corresponding neighbor tuple if it is needed.
///
/// \param tuple the link tuple which has been updated.
///
void OLSR::updated_link_tuple(OLSR_link_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = updated_link_tuple\n", CURRENT_TIME );

	double now = CURRENT_TIME;

	// Each time a link tuple changes, the associated neighbor tuple must be recomputed
	OLSR_nb_tuple* nb_tuple =
		state_.find_nb_tuple(tuple->nb_main_node_id());
//	if (nb_tuple != NULL) {
//		if (use_mac() && tuple->lost_time() >= now)
//			nb_tuple->status() = OLSR_STATUS_NOT_SYM;
//		else if (tuple->sym_time() >= now)
//			nb_tuple->status() = OLSR_STATUS_SYM;
//		else
//			nb_tuple->status() = OLSR_STATUS_NOT_SYM;
	if (nb_tuple != NULL) {
		if ( tuple->lost_time() >= now)
			nb_tuple->status() = OLSR_STATUS_NOT_SYM;
		else if (tuple->sym_time() >= now)
			nb_tuple->status() = OLSR_STATUS_SYM;
		else
			nb_tuple->status() = OLSR_STATUS_NOT_SYM;

//	fprintf (stdout,"%f:  has updated link tuple, status = %s\n",
//		now,
////		OLSR::node_id(ra_node_id()),
////		OLSR::node_id(tuple->nb_main_node_id()),
//		((nb_tuple->status() == OLSR_STATUS_SYM) ? "sym" : "not_sym"));
	}
}

///
/// \brief Adds a neighbor tuple to the Neighbor Set.
///
/// \param tuple the neighbor tuple to be added.
///
void OLSR::add_nb_tuple(OLSR_nb_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = add_nb_tuple\n", CURRENT_TIME );

//	fprintf(stdout,"%f: Node  adds neighbor tuple, status = %s\n",
//		CURRENT_TIME,
////		OLSR::node_id(ra_node_id()),
////		OLSR::node_id(tuple->nb_main_node_id()),
//		((tuple->status() == OLSR_STATUS_SYM) ? "sym" : "not_sym"));
	
	state_.insert_nb_tuple(tuple);
}

///
/// \brief Removes a neighbor tuple from the Neighbor Set.
///
/// \param tuple the neighbor tuple to be removed.
///
void OLSR::rm_nb_tuple(OLSR_nb_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = rm_nb_tuple\n", CURRENT_TIME );

	fprintf(stdout,"  %f: Node removes neighbor tuple, status = %s\n",
		CURRENT_TIME,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->nb_main_node_id()),
		((tuple->status() == OLSR_STATUS_SYM) ? "sym" : "not_sym"));
	
	state_.erase_nb_tuple(tuple);
}

///
/// \brief Adds a 2-hop neighbor tuple to the 2-hop Neighbor Set.
///
/// \param tuple the 2-hop neighbor tuple to be added.
///
void OLSR::add_nb2hop_tuple(OLSR_nb2hop_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = add_nb2hop_tuple\n", CURRENT_TIME );

//	debug("%f: Node %d adds 2-hop neighbor tuple: nb_addr = %d nb2hop_addr = %d\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->nb_main_node_id()),
//		OLSR::node_id(tuple->nb2hop_node_id()));

	state_.insert_nb2hop_tuple(tuple);
}

///
/// \brief Removes a 2-hop neighbor tuple from the 2-hop Neighbor Set.
///
/// \param tuple the 2-hop neighbor tuple to be removed.
///
void OLSR::rm_nb2hop_tuple(OLSR_nb2hop_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = rm_nb2hop_tuple\n", CURRENT_TIME );

//	debug("%f: Node %d removes 2-hop neighbor tuple: nb_addr = %d nb2hop_addr = %d\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->nb_main_node_id()),
//		OLSR::node_id(tuple->nb2hop_node_id()));

	state_.erase_nb2hop_tuple(tuple);
}

///
/// \brief Adds an MPR selector tuple to the MPR Selector Set.
///
/// Advertised Neighbor Sequence Number (ANSN) is also updated.
///
/// \param tuple the MPR selector tuple to be added.
///
void OLSR::add_mprsel_tuple(OLSR_mprsel_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = add_mprsel_tuple\n", CURRENT_TIME );

//	debug("%f: Node %d adds MPR selector tuple: nb_addr = %d\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->main_node_id()));

	state_.insert_mprsel_tuple(tuple);
	ansn_ = (ansn_ + 1)%(OLSR_MAX_SEQ_NUM + 1);
}

///
/// \brief Removes an MPR selector tuple from the MPR Selector Set.
///
/// Advertised Neighbor Sequence Number (ANSN) is also updated.
///
/// \param tuple the MPR selector tuple to be removed.
///
void OLSR::rm_mprsel_tuple(OLSR_mprsel_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = rm_mprsel_tuple\n", CURRENT_TIME );

//	debug("%f: Node %d removes MPR selector tuple: nb_addr = %d\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->main_node_id()));

	state_.erase_mprsel_tuple(tuple);
	ansn_ = (ansn_ + 1)%(OLSR_MAX_SEQ_NUM + 1);
}

///
/// \brief Adds a topology tuple to the Topology Set.
///
/// \param tuple the topology tuple to be added.
///
void OLSR::add_topology_tuple(OLSR_topology_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = add_topology_tuple\n", CURRENT_TIME );

//	debug("%f: Node %d adds topology tuple: dest_addr = %d last_addr = %d seq = %d\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->dest_node_id()),
//		OLSR::node_id(tuple->last_node_id()),
//		tuple->seq());

	state_.insert_topology_tuple(tuple);
}

///
/// \brief Removes a topology tuple from the Topology Set.
///
/// \param tuple the topology tuple to be removed.
///
void OLSR::rm_topology_tuple(OLSR_topology_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = rm_topology_tuple\n", CURRENT_TIME );

//	debug("%f: Node %d removes topology tuple: dest_addr = %d last_addr = %d seq = %d\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_node_id()),
//		OLSR::node_id(tuple->dest_node_id()),
//		OLSR::node_id(tuple->last_node_id()),
//		tuple->seq());

	state_.erase_topology_tuple(tuple);
}

///
/// \brief Adds an interface association tuple to the Interface Association Set.
///
/// \param tuple the interface association tuple to be added.
///
//void
//OLSR::add_ifaceassoc_tuple(OLSR_iface_assoc_tuple* tuple) {
//	debug("%f: Node %d adds iface association tuple: main_addr = %d iface_addr = %d\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_addr()),
//		OLSR::node_id(tuple->main_addr()),
//		OLSR::node_id(tuple->iface_addr()));
//
//	state_.insert_ifaceassoc_tuple(tuple);
//}

///
/// \brief Removes an interface association tuple from the Interface Association Set.
///
/// \param tuple the interface association tuple to be removed.
///
//void
//OLSR::rm_ifaceassoc_tuple(OLSR_iface_assoc_tuple* tuple) {
//	debug("%f: Node %d removes iface association tuple: main_addr = %d iface_addr = %d\n",
//		CURRENT_TIME,
//		OLSR::node_id(ra_addr()),
//		OLSR::node_id(tuple->main_addr()),
//		OLSR::node_id(tuple->iface_addr()));
//
//	state_.erase_ifaceassoc_tuple(tuple);
//}

///
/// \brief Gets the main address associated with a given interface address.
///
/// \param iface_addr the interface address.
/// \return the corresponding main address.
///
//nsaddr_t
//OLSR::get_main_addr(nsaddr_t iface_addr) {
//	OLSR_iface_assoc_tuple* tuple =
//		state_.find_ifaceassoc_tuple(iface_addr);
//
//	if (tuple != NULL)
//		return tuple->main_addr();
//	return iface_addr;
//}

///
/// \brief Determines which sequence number is bigger (as it is defined in RFC 3626).
///
/// \param s1 a sequence number.
/// \param s2 a sequence number.
/// \return true if s1 > s2, false in other case.
///
bool OLSR::seq_num_bigger_than(u_int16_t s1, u_int16_t s2) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = seq_num_bigger_than\n", CURRENT_TIME );

	return (s1 > s2 && s1-s2 <= OLSR_MAX_SEQ_NUM/2)
		|| (s2 > s1 && s2-s1 > OLSR_MAX_SEQ_NUM/2);
}

///
/// \brief This auxiliary function (defined in RFC 3626) is used for calculating the MPR Set.
///
/// \param tuple the neighbor tuple which has the main address of the node we are going to calculate its degree to.
/// \return the degree of the node.
///
int OLSR::degree(OLSR_nb_tuple* tuple) {
    fprintf (stdout, "CURRENT TIME = %f , STAGE = degree\n", CURRENT_TIME );

	int degree = 0;
	for (nb2hopset_t::iterator it = nb2hopset().begin(); it != nb2hopset().end(); it++) {
		OLSR_nb2hop_tuple* nb2hop_tuple = *it;
		if (nb2hop_tuple->nb_main_node_id() == tuple->nb_main_node_id()) {
			OLSR_nb_tuple* nb_tuple =
				state_.find_nb_tuple(nb2hop_tuple->nb_main_node_id());
			if (nb_tuple == NULL)
				degree++;
		}
	}
	return degree;
}

//===============================================================
int OLSR::traf_queue_add(Traf_Queue *q, Traf_Queue_Content *data)
{//在队列末尾加入新的业务
    fprintf (stdout, "CURRENT TIME = %f , STAGE = traf_queue_add\n", CURRENT_TIME );

    if((q->rear_ + 1) % Traf_Queue_Size == q->front_)
    {
        fprintf(stdout,"[time] %f [node] %d [full_queue]\n",CURRENT_TIME, ra_node_id());
            return -1;
    }
//    if( ra_node_id() == 7)
//    {
//    fprintf(stdout,"[time] %f [node] %d [add_queue]\n",CURRENT_TIME, ra_node_id());
//    }

    q->queue_content_[q->rear_].message_common_ = data->message_common_;
    q->queue_content_[q->rear_].frame_type_ = data->frame_type_;
    q->queue_content_[q->rear_].route_frame_content_pt_ = data->route_frame_content_pt_;
    q->queue_content_[q->rear_].short_message_pt_ = data->short_message_pt_;
    q->queue_content_[q->rear_].ack_content_ = data->ack_content_;
    q->queue_content_[q->rear_].endroute_pt_ = data->endroute_pt_;
    q->queue_content_[q->rear_].voice_pt_ = data->voice_pt_;

    q->rear_ = (q->rear_ + 1) % Traf_Queue_Size;
    return 0;
    return 0;
}
//==================================================================
///
/// \brief Converts a decimal number of seconds to the mantissa/exponent format.
///
/// \param seconds decimal number of seconds we want to convert.
/// \return the number of seconds in mantissa/exponent format.
///
//u_int8_t
//OLSR::seconds_to_emf(double seconds) {
//	// This implementation has been taken from unik-olsrd-0.4.5 (mantissa.c),
//	// licensed under the GNU Public License (GPL)
//
//	int a, b = 0;
// 	while (seconds/OLSR_C >= pow((double)2, (double)b))
//		b++;
//	b--;
//
//	if (b < 0) {
//		a = 1;
//		b = 0;
//	}
//	else if (b > 15) {
//		a = 15;
//		b = 15;
//	}
//	else {
//		a = (int)(16*((double)seconds/(OLSR_C*(double)pow(2, b))-1));
//		while (a >= 16) {
//			a -= 16;
//			b++;
//		}
//	}
//
//	return (u_int8_t)(a*16+b);
//}

///
/// \brief Converts a number of seconds in the mantissa/exponent format to a decimal number.
///
/// \param olsr_format number of seconds in mantissa/exponent format.
/// \return the decimal number of seconds.
///
//double
//OLSR::emf_to_seconds(u_int8_t olsr_format) {
//	// This implementation has been taken from unik-olsrd-0.4.5 (mantissa.c),
//	// licensed under the GNU Public License (GPL)
//	int a = olsr_format >> 4;
//	int b = olsr_format - a*16;
//	return (double)(OLSR_C*(1+(double)a/16)*(double)pow(2,b));
//}


///
/// \brief Returns the identifier of a node given the address of the attached OLSR agent.
///
/// \param addr the address of the OLSR routing agent.
/// \return the identifier of the node.
///
//int
//OLSR::node_id(nsaddr_t addr) {
//	// Preventing a bad use for this function
//	    fprintf(stdout , "node_id is %d , addr is addr",node_id,addr );
//        if ((u_int32_t)addr == IP_BROADCAST)
//		return addr;
//	// Getting node id
//	Node* node = Node::get_node_by_address(addr);
//
//	assert(node != NULL);
//	return node->nodeid();
//
//}

//added by li
//void
//OLSR::add_nodeidassoc_tuple(OLSR_nodeid_assoc_tuple* tuple) {
//
//	state_.insert_nodeidassoc_tuple(tuple);
//}

//added by xyy
void OLSR::UCDS_CD_computation()
{
	fprintf (stdout, "CURRENT TIME = %f , STAGE = UCDS_CD_computation\n", CURRENT_TIME );

	fprintf (stdout,"  1\n");
	state_.print_cd_node_id();
	fprintf (stdout,"  \n");

	//先处理之前接收到的其他节点发送的HELLO消息里面记录的骨干节点内容
	int local_cd = 0;
	int main_node_id = 0;
	for (linkset_t::iterator it0 = linkset().begin(); it0 != linkset().end(); it0++) {
		OLSR_link_tuple* link_tuple = *it0;
		main_node_id = link_tuple->local_main_node_id();

		//查看本节点是否被其他节点选为骨干节点
		if ( link_tuple->local_main_node_cd() == 1)
			local_cd = 1;

		fprintf (stdout,"  main_node = %d, nb_node_id = %d. \n",
				link_tuple->local_main_node_id(),  link_tuple->nb_main_node_id());

		fprintf (stdout,"      link_tuple->local_main_node_cd() = %d, link_tuple->nb_main_node_cd() = %d .\n",
				link_tuple->local_main_node_cd(), link_tuple->nb_main_node_cd());

		//查看其邻居节点有没有被其他节点选为骨干节点
		if ( link_tuple->nb_main_node_cd() == 1 && state_.find_cd_node_id( link_tuple->nb_main_node_id() ) == 0 ){
			state_.insert_cd_node_id( link_tuple->nb_main_node_id() );//如果CD表中没有该节点，则加入
			fprintf (stdout,"    33 :  link_tuple->nb_main_node_id() = [%d] .\n",  link_tuple->nb_main_node_id());
		}
		else if ( link_tuple->nb_main_node_cd() == 0 && state_.find_cd_node_id( link_tuple->nb_main_node_id() ) == 1){
			state_.erase_cd_node_id( link_tuple->nb_main_node_id() );// 如果该节点已经不是骨干节点，则从表中删除
			fprintf (stdout,"    44 :  link_tuple->nb_main_node_id() = [%d] .\n",  link_tuple->nb_main_node_id());
		}


	}

		if ( local_cd == 1 && state_.find_cd_node_id(main_node_id ) == 0 ){
			state_.insert_cd_node_id( main_node_id );//如果CD表中没有本节点，则加入
			fprintf (stdout,"    11 :  link_tuple->local_main_node_id() = [%d] .\n",  main_node_id);
		}
		else if ( local_cd == 0 && state_.find_cd_node_id( main_node_id ) == 1){
			state_.erase_cd_node_id( main_node_id );//如果该节点已经不是骨干节点，则从表中删除
			fprintf (stdout,"    22 :  link_tuple->local_main_node_id() = [%d] .\n",  main_node_id);
		}



	fprintf (stdout,"\n  2\n");
	state_.print_cd_node_id();
	fprintf (stdout,"  \n");


	//计算N中的虚拟骨干节点
	//(2) 本节点统计其一跳节点的邻居个数，可以查看通过其中某一个一跳节点到达二跳节点的链路的个数，
	//其中包括通过二跳节点是一跳节点集中的链路，最后将总数+1即可得到其一跳节点的邻居个数
	std::set<int> N_nb_num;//创建一个数组存放与N中顺序相同的节点邻居数量
	int main_node_nbnum = 0;
	int nb_node_nbnum = 0;
	bool ok = true;

	for (nbset_t::iterator it1 = nbset().begin(); it1 != nbset().end(); it1++){
		OLSR_nb_tuple* nb_tuple = *it1;
		if ( nb_tuple->status() == OLSR_STATUS_SYM ){
			main_node_nbnum++;
		}
	}
//	fprintf (stdout," \n");

	for (linkset_t::iterator it0 = linkset().begin(); it0 != linkset().end(); it0++) {
		OLSR_link_tuple* link_tuple = *it0;

//		//如果有节点选择本节点为骨干节点，那么就将所有邻居节点中的link_tuple->local_main_node_cd标记为1（不行）
//		//如果有节点选择本节点为骨干节点，那么将link_tuple第一项的local_main_node_cd标记为1
//		if ( local_cd == 1 && it0 == linkset().begin() )
//			link_tuple->local_main_node_cd() = 1;
//		else//否则，都初始化为0
//			link_tuple->local_main_node_cd() = 0;

		link_tuple->nb_main_node_nbnum() = 0 ;
		nb_node_nbnum = 0;
		if( it0 == linkset().begin() ){
			link_tuple->local_main_node_nbnum() = main_node_nbnum ;
			fprintf (stdout, "  main_node_id = [%d], main_node_nbnum = %d \n\n", main_node_id, main_node_nbnum );

			for (nbset_t::iterator it1 = nbset().begin(); it1 != nbset().end(); it1++){
				OLSR_nb_tuple* nb_tuple = *it1;
				fprintf (stdout, "  main_node_id = [%d], nb_node_id = [%d], nb_status = %d .  \n",
						main_node_id, nb_tuple->nb_main_node_id(), nb_tuple->status() );
			}
			fprintf (stdout," \n");

		}
//		fprintf (stdout, "  main_node_id = [%d], link_nb_node_id = [%d] . \n", main_node_id,  link_tuple->nb_main_node_id() );
//		fprintf( stdout, "  nb_node_id = [%d] .\n",link_tuple->nb_main_node_id() );
//		assert( link_tuple->nb_main_node_id() == nb_tuple->nb_main_node_id()
//					  && nb_tuple->status() == OLSR_STATUS_SYM ) ;
//		assert( link_tuple->sym_time() > CURRENT_TIME ) ;

		if( link_tuple->sym_time() < CURRENT_TIME )
			continue;

		for (nb2hopset_t::iterator it2 = nb2hopset().begin(); it2 != nb2hopset().end(); it2++) {
			OLSR_nb2hop_tuple* nb2hop_tuple = *it2;
			if( nb2hop_tuple->nb_main_node_id() != link_tuple->nb_main_node_id() )
				continue;

			ok = true;
			OLSR_nb_tuple* nb_tuple0 = state_.find_sym_nb_tuple(nb2hop_tuple->nb_main_node_id());//(iii)该节点二跳节点的一跳节点是该节点的一跳节点
			if (nb_tuple0 == NULL)
				ok = false;
			else {
				nb_tuple0 = state_.find_nb_tuple(nb2hop_tuple->nb_main_node_id(), OLSR_WILL_NEVER);//(i)该节点二跳节点的一跳节点是该节点的一跳节点，willingness为否
				if (nb_tuple0 != NULL)
					ok = false;
			}

			if (ok){
				nb_node_nbnum++;
				fprintf( stdout, "  nb_node_id =  %d, nb2hop_node_id = [%d] .\n", nb2hop_tuple->nb_main_node_id(), nb2hop_tuple->nb2hop_node_id() );
			}
		}
		link_tuple->nb_main_node_nbnum() = nb_node_nbnum + 1;//这里的1指的是本节点是该一跳节点的邻居
		fprintf( stdout, "  main_ node_id= [%d], nb_node_id = [%d], nb_nbnum =  %d .\n\n",
				main_node_id, link_tuple->nb_main_node_id() , link_tuple->nb_main_node_nbnum());

	}
//	fprintf (stdout," \n");

//	bool mine_cd = true;//指定自己为CD节点标志位
	int max_nbnum_node_id = 0;//存放支配因子最大的节点号
	int max_nbnum = 0;//存放支配因子最大的节点的支配因子
	//指定自己为CD节点
	for (linkset_t::iterator it0 = linkset().begin(); it0 != linkset().end(); it0++) {
		OLSR_link_tuple* link_tuple = *it0;
//		fprintf (stdout, "  nb_node_id = [%d], nb_node_nbnum = %d \n", link_tuple->nb_main_node_id(), link_tuple->nb_main_node_nbnum() );

		if( it0 == linkset().begin() ){//如果是第一个，则将max初始化为本节点的信息
			max_nbnum_node_id = main_node_id;
			max_nbnum				   = main_node_nbnum;
		}

		if ( max_nbnum < link_tuple->nb_main_node_nbnum()
				|| ( max_nbnum == link_tuple->nb_main_node_nbnum()
						&& max_nbnum_node_id < link_tuple->nb_main_node_id() ) ){
			max_nbnum_node_id = link_tuple->nb_main_node_id();
			max_nbnum				   = link_tuple->nb_main_node_nbnum();
		}
	}
//	fprintf (stdout," \n");

	choose_cd_node_id() = 100;
	if( max_nbnum > 0 ){
		choose_cd_node_id() = max_nbnum_node_id;

		if ( state_.find_cd_node_id( max_nbnum_node_id ) == 0 )
			state_.insert_cd_node_id( max_nbnum_node_id );

		fprintf (stdout,"  node[%d] chose node[%d] to be CD node, the neighbor's number is %d . \n",
				main_node_id, choose_cd_node_id(), max_nbnum );
	}

	fprintf (stdout," \n");
	state_.print_cd_node_id();
	fprintf (stdout," \n");

}



//added by xyy
void OLSR::UCDS_mpr_computation()
{
	fprintf (stdout, "CURRENT TIME = %f , STAGE = UCDS_mpr_computation\n", CURRENT_TIME );

	int main_node = 0;
	for (linkset_t::iterator it = linkset().begin(); it != linkset().end(); it++) {
			OLSR_link_tuple* link_tuple = *it;
			main_node = link_tuple->local_main_node_id();
			break;
	}


	// MPR computation should be done for each interface. See section 8.3.1
	// (RFC 3626) for details.
	state_.clear_mprset();

	nbset_t N; nb2hopset_t N2;
	// N is the subset of neighbors of the node, which are
	// neighbor "of the interface I"
	for (nbset_t::iterator it = nbset().begin(); it != nbset().end(); it++)
	{
		if ((*it)->status() == OLSR_STATUS_SYM) // I think that we need this check
		{
			N.push_back(*it);//计算N集合
		}
	}


	for (nbset_t::iterator it = N.begin(); it != N.end(); it++){
		OLSR_nb_tuple* N_tuple = *it;
		fprintf (stdout,"  main_node = %d, N_nb_node_id = %d. \n",main_node, N_tuple->nb_main_node_id());
	}
	fprintf (stdout," \n");

	// N2 is the set of 2-hop neighbors reachable from "the interface
	// I", excluding:
	// (i)   the nodes only reachable by members of N with willingness WILL_NEVER
	// (ii)  the node performing the computation正在进行计算的节点
	// (iii) include all the symmetric neighbors: the nodes for which there exists a symmetric
	//       link to this node on some interface.
	//计算N2集合
	for (nb2hopset_t::iterator it = nb2hopset().begin(); it != nb2hopset().end(); it++) {
		OLSR_nb2hop_tuple* nb2hop_tuple = *it;
		bool ok = true;
//		OLSR_nb_tuple* nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb_main_addr());
		OLSR_nb_tuple* nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb_main_node_id());//(iii)该节点二跳节点的一跳节点是该节点的一跳节点
		if (nb_tuple == NULL)
			ok = false;
		else {
			nb_tuple = state_.find_nb_tuple(nb2hop_tuple->nb_main_node_id(), OLSR_WILL_NEVER);//(i)该节点二跳节点的一跳节点是该节点的一跳节点，willingnessWie否
//			nb_tuple = state_.find_nb_tuple(nb2hop_tuple->nb_main_addr(), OLSR_WILL_NEVER);
			if (nb_tuple != NULL)
				ok = false;
			else {
//				nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb2hop_addr());
				nb_tuple = state_.find_sym_nb_tuple(nb2hop_tuple->nb2hop_node_id());//(iii)该二跳节点是本节点的一跳节点
				if (nb_tuple != NULL)
					ok = false;
			}
		}

		if (ok){
			N2.push_back(nb2hop_tuple);
		}
	}


	for (nb2hopset_t::iterator it = N2.begin(); it != N2.end(); it++) {
//		fprintf (stdout,"  I am come in . \n");
		OLSR_nb2hop_tuple* N2_tuple = *it;
		fprintf (stdout,"  main_node = %d, N2_nb_node_id = %d, N2_nb2hop_node_id = %d. \n",
				main_node,  N2_tuple->nb_main_node_id(), N2_tuple->nb2hop_node_id());
	}
	fprintf (stdout," \n");



	//删除N中非虚拟骨干接点，并删除它们覆盖的N2中的节点
	for (nbset_t::iterator it = N.begin(); it != N.end(); it++) {
		OLSR_nb_tuple* nb_tuple = *it;
//		fprintf (stdout,"  node[%d] isin N . \n", nb_tuple->nb_main_node_id() );
		for (linkset_t::iterator it0 = linkset().begin(); it0 != linkset().end(); it0++) {
			OLSR_link_tuple* link_tuple = *it0;
//			fprintf (stdout,"  node[%d] is come Link . \n", nb_tuple->nb_main_node_id() );
			if( link_tuple->nb_main_node_id() == nb_tuple->nb_main_node_id() ){
				if( state_.find_cd_node_id( link_tuple->nb_main_node_id() ) == 1 )
					fprintf( stdout, "  node[%d] is the CD node .\n", link_tuple->nb_main_node_id() );

				else if( state_.find_cd_node_id( link_tuple->nb_main_node_id() ) == 0 ){
					fprintf( stdout, "  node[%d] isn't CD node .\n", link_tuple->nb_main_node_id() );

					for (nb2hopset_t::iterator it1 = N2.begin(); it1 != N2.end(); it1++) {
						OLSR_nb2hop_tuple* nb2hop_tuple = *it1;
						if ( nb2hop_tuple->nb_main_node_id() == link_tuple->nb_main_node_id() ){
							it1 = N2.erase(it1);
							if ( it1 == N2.end() )
								break;
							else
								it1--;
						}
					}

					it = N.erase(it);
					if ( N.begin() != N.end() )
						it--;

					break;
				}
			}
		}

		if ( it == N.end() )
			break;
	}


	fprintf (stdout," \n");
	for (nb2hopset_t::iterator it = N2.begin(); it != N2.end(); it++) {
				OLSR_nb2hop_tuple* N2_tuple = *it;
				fprintf (stdout,"  main_node = %d, N2_nb_node_id = %d, N2_nb2hop_node_id = %d. \n",
						main_node,  N2_tuple->nb_main_node_id(), N2_tuple->nb2hop_node_id());
	}
	fprintf (stdout," \n");


	// 1. Start with an MPR set made of all members of N with
	// N_willingness equal to WILL_ALWAYS

	// 2. Calculate D(y), where y is a member of N, for all nodes in N.
	// We will do this later.

	// 3. Add to the MPR set those nodes in N, which are the *only*
	// nodes to provide reachability to a node in N2. Remove the
	// nodes from N2 which are now covered by a node in the MPR set.

	mprset_t foundset;

	std::set<int> deleted_node_ids;

	OLSR_nb_tuple* min = NULL;
	int min_r = 0;
	bool found = false;

	while (N2.begin() != N2.end()) {//N2不为空
		for (nb2hopset_t::iterator it = N2.begin(); it != N2.end(); it++) {
			foundset.clear();
			OLSR_nb2hop_tuple* nb2hop_tuple1 = *it;

			fprintf (stdout,"  FIND ONLY NODE : main_node = %d, N2_nb_node_id = %d, N2_nb2hop_node_id = %d. \n",
					main_node,  nb2hop_tuple1->nb_main_node_id(), nb2hop_tuple1->nb2hop_node_id());

			mprset_t::iterator pos = foundset.find(nb2hop_tuple1->nb2hop_node_id());
			if (pos != foundset.end())//如果找到这样的节点，则不执行下面的语句，跳到下一个循环
				continue;//foundset.end??

			found = false;
			for (nbset_t::iterator it2 = N.begin(); it2 != N.end(); it2++) {
				if ((*it2)->nb_main_node_id() == nb2hop_tuple1->nb_main_node_id()) {
					//N中存在连接本节点和N2中节点it2的一跳节点
					found = true;
					break;//跳出最近的一个循环
				}
			}
			if (!found)
				continue;

			found = false;
			for (nb2hopset_t::iterator it2 = it + 1; it2 != N2.end(); it2++) {
				OLSR_nb2hop_tuple* nb2hop_tuple2 = *it2;
				if (nb2hop_tuple1->nb2hop_node_id() == nb2hop_tuple2->nb2hop_node_id()) {
					//从本节点到二跳节点存在多条路径，导致二跳节点集中该二跳节点有多条记录
					//但由于它经过不同的一跳节点到达本节点，所以记录不同
					foundset.insert(nb2hop_tuple1->nb2hop_node_id());//到N2中节点有多条路径的则将第一条存放在foundset中
					found = true;
					break;
				}
			}
			if (!found) {	//此时本节点到该二跳节点只存在一条路径，因此找到只通过唯一N中一跳节点到达该二跳节点的一跳节点
				state_.insert_mpr_node_id(nb2hop_tuple1->nb_main_node_id());//将该节点加入到mpr集中
				fprintf (stdout,"  ONLY_NODE : main_node = %d, mpr_node_id = %d \n\n",main_node, nb2hop_tuple1->nb_main_node_id());

				//将已经加到mpr集合中的节点所覆盖的N2中的节点删除
				for (nb2hopset_t::iterator it2 = N2.begin(); it2 != N2.end(); it2++) {
					OLSR_nb2hop_tuple* nb2hop_tuple2 = *it2;
					if (nb2hop_tuple1->nb_main_node_id() == nb2hop_tuple2->nb_main_node_id()) {
						deleted_node_ids.insert(nb2hop_tuple2->nb2hop_node_id());
						it2 = N2.erase(it2);//erase之后指针就指向下一条了
						it2--;
					}
				}
			}

			else
				continue;

			it = N2.begin();//指针指回最初的N2，重新查找是否有only节点
			it--;

			//将mpr节点覆盖的N2中节点通过其他一跳节点到达本节点的路径删除
			for (std::set<int>::iterator it2 =deleted_node_ids.begin(); it2 != deleted_node_ids.end(); it2++) {
				for (nb2hopset_t::iterator it3 = N2.begin();it3 != N2.end();it3++) {
					if ((*it3)->nb2hop_node_id() == *it2) {
						it3 = N2.erase(it3);//把与已经放到mpr集中的一跳节点相连的二跳节点从N2中删除
						it3--;

						it = N2.begin();//指针指回最初的N2，重新查找是否有only节点
						it--;
					}
				}
			}
			deleted_node_ids.clear();//对于这个刚加入mpr节点的deleted_node_ids集清零


			for (nb2hopset_t::iterator it4 = N2.begin(); it4 != N2.end(); it4++) {
				OLSR_nb2hop_tuple* N2_tuple = *it4;
				fprintf (stdout,"  after delete : main_node = %d, N2_nb_node_id = %d, N2_nb2hop_node_id = %d. \n",
						main_node,  N2_tuple->nb_main_node_id(), N2_tuple->nb2hop_node_id());
			}
			fprintf (stdout," \n");


		}

		if ( N2.begin() == N2.end() )
			break;


	// 4. While there exist nodes in N2 which are not covered by at
	// least one node in the MPR set:

		// 4.1. For each node in N, calculate the reachability, i.e., the
		// number of nodes in N2 which are not yet covered by at
		// least one node in the MPR set, and which are reachable
		// through this 1-hop neighbor
		map<int, std::vector<OLSR_nb_tuple*> > reachability;
		set<int> rs;
		for (nbset_t::iterator it = N.begin(); it != N.end(); it++) {
			OLSR_nb_tuple* nb_tuple = *it;
			int r = 0;
			for (nb2hopset_t::iterator it2 = N2.begin(); it2 != N2.end(); it2++) {
				OLSR_nb2hop_tuple* nb2hop_tuple = *it2;
				if (nb_tuple->nb_main_node_id()== nb2hop_tuple->nb_main_node_id())
					r++;//计算该节点的reachability
			}
			rs.insert(r);
			reachability[r].push_back(nb_tuple);
		}


		// 4.2. Select as a MPR the node with highest N_willingness among
		// the nodes in N with non-zero reachability. In case of
		// multiple choice select the node which provides
		// reachability to the maximum number of nodes in N2. In
		// case of multiple nodes providing the same amount of
		// reachability, select the node as MPR whose D(y) is
		// greater. Remove the nodes from N2 which are now covered
		// by a node in the MPR set.
		min = NULL;
		min_r = 0;
		for (set<int>::iterator it = rs.begin(); it != rs.end(); it++) {
			int r = *it;
			if (r > 0) {
				for (std::vector<OLSR_nb_tuple*>::iterator it2 = reachability[r].begin();
					it2 != reachability[r].end();
					it2++) {
					OLSR_nb_tuple* nb_tuple = *it2;
					fprintf (stdout,"  nb_node_id = %d, nb_tuple_r = %d .\n", nb_tuple->nb_main_node_id(), r);
					if (min == NULL || nb_tuple->willingness() < min->willingness()) {
						min = nb_tuple;
						min_r = r;
					}
					else if (nb_tuple->willingness() == min->willingness()) {
						if (r < min_r) {
							min = nb_tuple;
							min_r = r;
						}
						else if (r == min_r) {
							fprintf (stdout,"  ");
							int degree_nb_tuple = degree(nb_tuple);
//							fprintf (stdout,"  degree_nb_tuple = %d .\n", degree_nb_tuple);
							fprintf (stdout,"  ");
							int degree_min = degree(min);
//							fprintf (stdout,"  degree_min = %d .\n", degree_min);
//							fprintf (stdout," \n");
							if (degree_nb_tuple < degree_min) {
								min = nb_tuple;
								min_r = r;
							}
						}
					}
				}
			}
		}

		fprintf (stdout,"  min_nb = %d, min_r = %d .\n", min->nb_main_node_id(), min_r);


//		//删除N中非虚拟骨干接点，并删除它们覆盖的N2中的节点
		for (nb2hopset_t::iterator it = N2.begin(); it != N2.end(); it++) {
			OLSR_nb2hop_tuple* nb2hop_tuple = *it;
			if ( nb2hop_tuple->nb_main_node_id() == min->nb_main_node_id() ){
				deleted_node_ids.insert(nb2hop_tuple->nb2hop_node_id());
				it = N2.erase(it);
				it--;
			}
		}


		//将mpr节点覆盖的N2中节点通过其他一跳节点到达本节点的路径删除
		for (std::set<int>::iterator it2 =deleted_node_ids.begin(); it2 != deleted_node_ids.end(); it2++) {
			for (nb2hopset_t::iterator it3 = N2.begin();it3 != N2.end();it3++) {
				if ((*it3)->nb2hop_node_id() == *it2) {
					it3 = N2.erase(it3);//把与已经放到mpr集中的一跳节点相连的二跳节点从N2中删除
					it3--;
				}
			}
		}
		deleted_node_ids.clear();//对于这个刚加入mpr节点的deleted_node_ids集清零

	}

}
