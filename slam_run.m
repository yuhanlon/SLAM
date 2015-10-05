%%
% This script is the exection of the slam algorithm
%
% @author: Yuhan Long <yuhanlon@andrew.cmu.edu>
load 'time_sequence.mat'

% initialize the state vector
x = [0.05,0.05,0.03,0,0,1.241,0,0,0,0,0,0,0,0,0]';
landmark_size=0;

landmark_pos = [];



% UKF parameters
alpha = 0.1; 
kappa = 0;
beta = 2;





pos_ind = [1:3];
rpy_ind = [4:6];
vel_ind = [7:9];
bw_ind = [10:12];
ba_ind = [13:15];

gravity = 9.8066;
p = x(pos_ind);
rpy = x(rpy_ind);
v = x(vel_ind);
bw = x(bw_ind);
ba = x(ba_ind);

len = size(time_sequence,2);

t=0;

%add noise charateristics
nw_sigma = ones(1,3)*(0.03^2);
nbw_sigma = ones(1,3)*(0.03^2);
na_sigma = ones(1,3)*(0.08^2);
nba_sigma = ones(1,3)*(0.03^2);
nimg_sigma = 0.1^2;

P = diag([0.01^2,0.01^2,0.01^2,(0.001^2)*ones(1,12)]);


x_history = [];
x_history_cor = [];



for i=1:len
	if strcmp(time_sequence(i).type,'imu')
		% using dynamic model if it is a imu read
		p = x(pos_ind);
		rpy = x(rpy_ind);
		v = x(vel_ind);
		bw = x(bw_ind);
		ba = x(ba_ind);
		
		index = time_sequence(i).index;
		am = imu_data(index).a;
		wm = imu_data(index).w;
		dt = time_sequence(i).time-t;
		
		x_aug = [x;reshape(landmark_pos,[landmark_size*3,1]);zeros(12,1)];
		P_state = P;
		P_state_aug = [P_state,zeros(size(P_state,1),12);zeros(12,size(P_state,2)),diag([nw_sigma,nbw_sigma,na_sigma,nba_sigma])];
		


		% reseed sigma points
		sp = reseedSigmaPoints(x_aug,P_state_aug,alpha,beta,kappa);		
		
		% propagate sigma points 
		x_propagate = [];
		for n = 1:size(sp,2)
			p = sp(pos_ind,n);
			rpy = sp(rpy_ind,n);
			v = sp(vel_ind,n);
			bw = sp(bw_ind,n);
			ba = sp(ba_ind,n);
			nw = sp(16+landmark_size*3:18+landmark_size*3,n);
			nbw = sp(19+landmark_size*3:21+landmark_size*3,n);
			na = sp(22+landmark_size*3:24+landmark_size*3,n);
			nba = sp(25+landmark_size*3:27+landmark_size*3,n);
			[p_next, rpy_next, v_next, bw_next, ba_next] = imuProcessDynamics(p,rpy,v,bw,ba,nw,nbw,na,nba,wm,am,dt,gravity);
			x_propagate = [x_propagate,[p_next;rpy_next;v_next;bw_next;ba_next]]; 	
			
		end
		%calculate new covariance and mean 
			
		[weight_mean,weight_cor]=getWeight(alpha,beta,kappa,15+12+landmark_size*3);
		x_propagate = [x_propagate;sp(16:end,:)];	
	
		x_prior = x_propagate*weight_mean';
		x_tilt = bsxfun(@minus,x_propagate,x_prior);
		P_prior = x_tilt*(bsxfun(@times,x_tilt,weight_cor))';
			
		t = time_sequence(i).time;
		x=x_prior(1:15,1);
		landmark_pos = reshape(x_prior(16:landmark_size*3+15,1),[3,landmark_size]);
		P=P_prior(1:15+landmark_size*3,1:15+landmark_size*3);
		x_history = [x_history,x];
	
	else
		% use observe model if it is a image feature read 
		
		index = time_sequence(i).index;
		landmark_list = feature_trail(index).landmark; 
		obs_list = landmark_list(landmark_list<=landmark_size);
		new_list = landmark_list(landmark_list>landmark_size);
		
		% prepare for correction
		obs_landmark = feature_trail(index).wp(1:3,(landmark_list<=landmark_size));%might have dimensional problems		
		new_landmark = feature_trail(index).wp(1:3,(landmark_list>landmark_size));
		if landmark_size>0		
			landmark_exist = reshape(landmark_pos(1:3,1:landmark_size),[3*landmark_size,1]);
		else
			landmark_exist = [];
		end
		

		if size(new_list,2)>0
			% when new landmark added in to the feature trail
			x_aug = [x;landmark_exist;zeros(landmark_list(end)*3,1)];
			P_aug = [P,zeros(size(P,1),landmark_list(end)*3);zeros(landmark_list(end)*3,size(P,2)),diag(ones(landmark_list(end)*3,1)*nimg_sigma)];
			
			% get sigma point after adding new landmark
			sp = reseedSigmaPoints(x_aug,P_aug,alpha,beta,kappa);
			
			landmark_exist_ind = [16:15+landmark_size*3];
			landmark_exist_noise_ind = [16+landmark_size*3:15+landmark_size*3*2];
			landmark_new_noise_ind = [16+landmark_size*3*2:15+landmark_size*3+landmark_list(1,end)*3];
			
			landmark_new_pos=[];
			
			% project new landmark into camera frame
			for n = 1:size(sp,2)
				p = sp(pos_ind,n);
				rpy = sp(rpy_ind,n);
				new_landmark_with_noise = reshape(reshape(new_landmark,[numel(new_landmark),1])+sp(landmark_new_noise_ind,n),size(new_landmark));
				new_landmark_reproject = landmarkReproject(p,rpy,new_landmark_with_noise,rotation_imu_to_leftcam,translation_imu_to_leftcam);
				landmark_new_pos =[landmark_new_pos,reshape(new_landmark_reproject(1:3,:),[numel(new_landmark),1])];				
			end
			
			sp = [sp(1:15,:);sp(landmark_exist_ind,:);landmark_new_pos;sp(landmark_exist_noise_ind,:);sp(landmark_new_noise_ind,:)];
			[weight_mean,weight_cor]=getWeight(alpha,beta,kappa,15+landmark_size*3+landmark_list(1,end)*3);

			% calculate new covariance matrix
			x_aug = sp*weight_mean';
			x_aug_tilt = bsxfun(@minus,sp,x_aug);
			P_aug = x_aug_tilt*(bsxfun(@times,x_aug_tilt,weight_cor))';

			x = x_aug(1:15,1);

			landmark_pos = reshape(x_aug(16:15+landmark_list(1,end)*3,1),[3,landmark_list(1,end)]);
			landmark_size =landmark_list(1,end);


			P = P_aug(1:15+landmark_size*3,1:15+landmark_size*3);	
		end
		

		if size(obs_list,2)>0 
			number = size(obs_list,2);
			cor_landmark = landmark_pos(:,obs_list);;
			z = reshape(obs_landmark,[3*number,1]);
			z_aug = [reshape(obs_landmark,[3*number,1]);zeros(3*number,1)];
			x_aug = [x;reshape(landmark_pos,[landmark_size*3,1]);zeros(3*number,1)];

			P_obs = [P,zeros(size(P,1),3*number);zeros(3*number,size(P,2)),diag(ones(3*number,1)*nimg_sigma)];
			
			% get sigma point
			sp = reseedSigmaPoints(x_aug,P_obs,alpha,beta,kappa);		
			% propagate sigma points to observe model 
			h=[];
			for n = 1:size(sp,2)
				p = sp(pos_ind,n);
				rpy = sp(rpy_ind,n);
				h =[h,reshape(landmarkObserve(p,rpy,cor_landmark,rotation_imu_to_leftcam,translation_imu_to_leftcam),[3*number,1])+sp(size(P,1)+1:size(P,1)+3*number,n)];
		
			end
			% calculate kalman gain 
			[weight_mean,weight_cor]=getWeight(alpha,beta,kappa,15+landmark_size*3+3*number);
			h_mean = h*weight_mean';
			h_tilt = bsxfun(@minus,h,h_mean);
			P_hh = h_tilt*bsxfun(@times,h_tilt,weight_cor)';
			
			%x_obs_mean = sp*weight_mean';
			x_obs_tilt = bsxfun(@minus,sp,x_aug);
			P_cross = x_obs_tilt*((bsxfun(@times,h_tilt,weight_cor))');

			K = P_cross*inv(P_hh);
			% calculate posterior mean and covariance 
			x_post = x_aug+K*(z-h_mean);
			P_post = P_obs-K*P_hh*K';

			x = x_post(1:15,1);
			landmark_pos = reshape(x_post(16:15+landmark_size*3),[3,landmark_size]);
			P = P_post(1:15+landmark_size*3,1:15+landmark_size*3);

		end
		x_history = [x_history,x];
		x_history_cor = [x_history_cor,x];
		if t>0
			t = time_sequence(i).time;
		end
	
	end
end
