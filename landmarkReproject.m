%%
% this is the reprojection of the world point to the camera frame 
% this is a reverse process of landmarkObserve
% 
% @author: Yuhan Long <yuhanlon@andrew.cmu.edu>

function [obs_point] = landmarkReproject(p,rpy,landmark_pos,rotation_imu_to_leftcam,translation_imu_to_leftcam)
	%p,rpy: the body pose of the drone
	%landmark_pos: the 3D location of the landmark in world frame
	%rotation_imu_to_leftcam\translation_imu_to_leftcam: rotation and translation from imu the camera
	
	transform_matrix=[ZYXToR(rpy),p;zeros(1,3),1];
	obs_point = bsxfun(@minus,landmark_pos,translation_imu_to_leftcam);
	obs_point = inv(rotation_imu_to_leftcam)*obs_point;

	
	obs_point = transform_matrix*[obs_point;ones(1,size(obs_point,2))];
	obs_point = bsxfun(@rdivide,obs_point,obs_point(4,:));
end 
