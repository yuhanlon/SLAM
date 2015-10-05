%%
% this function is the observe model for the SLAM algorithm 
%
%@author: Yuhan Long <yuhanlon@andrew.cmu.edu>

function [obs_point]= landmarkObserve(p,rpy,landmark_pos,rotation_imu_to_leftcam,translation_imu_to_leftcam)
	%p,rpy: the body pose of the drone
	%landmark_pos: the 3D location of the landmark in camera frame
	%rotation_imu_to_leftcam\translation_imu_to_leftcam: rotation and translation from imu the camera

	transform_matrix=[ZYXToR(rpy),p;zeros(1,3),1];
	obs_point = inv(transform_matrix)*[landmark_pos;ones(1,size(landmark_pos,2))];
	obs_point = obs_point(1:3,:);
	obs_point = bsxfun(@plus,rotation_imu_to_leftcam*obs_point,translation_imu_to_leftcam);
end
