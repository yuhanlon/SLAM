%%
% this function is to unroll a radian angle to the range 0 to 2*pi
%
% @author: Yuhan Long <yuhanlon@andrew.cmu.edu>

function ang_out = unroll(ang_in)
	% ang_in: input radius angle
	ang_out = mod(ang_in, 2*pi);

	if ang_out < 0
    	ang_out = ang_out + 2*pi;
	end

end
