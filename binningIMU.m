%%
% This script is for reading the IMU data and feature trial into a vector
%
% @author: Yuhan Long <yuhanlon@andrew.cmu.edu>


length = size(imu_timestamps,2);

% the IMU data item has three fields:
% time; the time stamp of the data
% a: body acceleration
% w: body angular velocity

imu_data = [];
for i = 1:length
	imu_data = [imu_data,struct('time',imu_timestamps(i),'a',body_accel(:,i),'w',body_angvel(:,i))];
end

imu_size = size(imu_timestamps,2)
image_size=size(image_timestamps,2)


imu_count = 1;
image_count = 1;


time_sequence=[];



while 1
	%compares the timestamp of IMU and feature trial. Pick up the earlier data and bind it to the result vector
	if imu_timestamps(imu_count)<image_timestamps(image_count)
		time_sequence=[time_sequence,struct('type','imu','index',imu_count,'time',imu_timestamps(imu_count))];
		if imu_count+1<=imu_size
			imu_count = imu_count+1;
		end				
	else
		time_sequence=[time_sequence,struct('type','image','index',image_count,'time',image_timestamps(image_count))];
		if image_count+1<=image_size
			image_count = image_count+1;
		end				
	end

	if (image_count ==image_size) || (imu_count == imu_size)
		break;
	end
	disp(imu_count);
	disp(image_count);
end

