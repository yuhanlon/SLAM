%% 
% This functions is for generating the imu motion model in the SLAM algorithm
%
%@author: Yuhan <yuhanlon@andrew.cmu.edu>

function [P,RPY,V,BW,BA] = imuProcessDynamics(p,rpy,v,bw,ba,nw,nbw,na,nba,wm,am,dt,gravity)
	% p: vector of position
	% rpy: vector of roll, pitch, yaw
	% v: vector of velocity
	% bw: IMU body frame angular velocity bias
	% ba: IMU body frame linear acceleration bias
	% nw: additive angular velocity noise
	% nbw: additive angular acceleration noise
	% na: additive linear acceleration noise
	% nba: additive linear jerk noise
	% wm: the measured body angular velocity
	% am: the measured body linear acceleration
	% dt:  time interval

	a = ZYXToR(rpy)*(am-ba-na)-[-0.1226;0;9.8043];
	w = wm-bw-nw;

	P = dt * v + p;
	V = dt * a + v;
	RPY = [1,sin(rpy(1))*tan(rpy(2)),cos(rpy(1))*tan(rpy(2));...
		   0,cos(rpy(1)),-sin(rpy(1));...
	 	   0,sin(rpy(1))/cos(rpy(2)),cos(rpy(1))/cos(rpy(2))]*w*dt+rpy;

	BW =dt * nbw+bw;
	BA =dt * nba+ba;


end
