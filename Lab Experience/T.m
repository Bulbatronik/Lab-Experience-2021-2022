function [frame_t] = T(frame,dx, dy, dphi)

%T = [cos(dphi), -sin(dphi), dx; ...
%     sin(dphi),  cos(dphi), dy];%; ... 
     % 0,           0,      1]; <- ignore how the heading changes, 
     % since weconsider it constant 

R = [cos(dphi), -sin(dphi); ...
     sin(dphi),  cos(dphi)];

%frame_t = zeros(length(frame), 2);
frame_t = frame*R;
frame_t = frame_t + [dx, dy];%by h
%for i = 1:length(frame)
%    frame_t(i,:) = T*[frame(i,:),h]'; 
%end

