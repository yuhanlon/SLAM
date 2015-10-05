%% 
% this function calculates the shortest rotation of two radius angle
%
% @author Yuhan Long <yuhanlon@andrew.cmu.edu>

function ang_out = shortest_angular_distance(from,to)

	% unroll two angle and calculate the rotation
    ang_out = unroll(unroll(to) - unroll(from));

    % if the rotation is larger then pi then make it to be a rotation smaller than pi to the different direction
    if ang_out > pi
        ang_out = -(2*pi - ang_out);
    end


    % calculate the absolute distance between the from angle and to angle
    ang_out = mod(ang_out + pi, 2*pi);
    if ang_out < 0
        ang_out = ang_out + 2*pi;
    end

    ang_out = ang_out - pi;

end
