%% 1)Transform GPS to Cartesian

%LIDAR DATA STRUCTURE
%scene = [frame1,frame2, ..., frameN]
%where frameK = [[ang0, dist0], [ang1, dist1], ...,[angX, distX]]

%GPS DATA STRUCTURE
% lat, lon, alt, vel_hor, vel_vert, GPSTime
clear; clc; close all;
load('C:\Users\Asus\Desktop\Lab Experience\Data\gps.mat');
load('C:\Users\Asus\Desktop\Lab Experience\Data\lidar.mat');

[x_g, y_g] = deg2utm(lat,lon);

% figure; geoplot(lat, lon)
% 
% figure; plot(x_g(1:5), y_g(1:5), ".") xlabel("x") ylabel("y")
% title("trajectory") axis('equal');

M = length(Z); % # lidar samples
N = length(x_g); % # gps samples

% % Heading extraction (Linear interpolation)
 h1 = zeros(N-1,1); % DISCARD LAST SAMPLE
 for t = 1:N-1
     h1(t) = atan2d((y_g(t+1) - y_g(t)),(x_g(t+1) - x_g(t)));
 end
%% test 1
% % Set maximum lidar range to be slightly smaller than maximum range of the
% % scanner, as the laser readings are less accurate near maximum range
% maxLidarRange = 15;
% % Set the map resolution to 10 cells per meter, which gives a precision of
% % 10cm
% mapResolution = 5;
% % Create a pose graph object and define information matrix
% pGraph = poseGraph;
% infoMat = [1 0 0 1 0 1];
% % Loop over each scan and estimate relative pose
% %scan = lidarScan(ranges,angles)
% prevScan = lidarScan( Z{1,1}(:,2), Z{1,1}(:,1));
% for i = 2:numel(Z)
%     %currScan = scans{i};
%     currScan = lidarScan(Z{i,1}(:,2),Z{i,1}(:,1));
%     % Estimate relative pose between current scan and previous scan
%     [relPose,stats] = matchScansGrid(currScan,prevScan, ...
%         'MaxRange',maxLidarRange,'Resolution',mapResolution);
%     % Refine the relative pose
%     relPoseRefined = matchScans(currScan,prevScan,'initialPose',relPose);
%     % Add relative pose to the pose graph object
%     pGraph.addRelativePose(relPoseRefined,infoMat);
%     ax = show(pGraph,'IDs','off');
%     %title(ax,'Estimated Robot Trajectory')
%     drawnow
%     prevScan = currScan;
% end
%% 2) GPS interpolation

T_gps = 0.1;%s from the sheet 
% motor frequency: min - 5, typical - 7, max - 12 
% but we can estimate it : T_gps*N/M = 0.1154 -> 8.6655 Hz
T_lidar = 0.1154;% s

% INTERPOLATE GPS DATA AT LIDAR'S POINTS
x = interp1(0:T_gps:(N-1)*T_gps, x_g, 0:T_lidar:(M-1)*T_lidar)';
y = interp1(0:T_gps:(N-1)*T_gps, y_g, 0:T_lidar:(M-1)*T_lidar)';

% Heading extraction (Linear interpolation)
h = zeros(M-1,1); % DISCARD LAST SAMPLE
for t = 1:M-1
    h(t) = atan2d((y(t+1) - y(t)),(x(t+1) - x(t)));
end

h = unwrap(h);
h_g = h;
h = h - h(1);
%% 3a) (lidar) polar to cartesian
R_max = 0;

for i = 1:M
    [Z_xy_loc{i,1}(:,1), Z_xy_loc{i,1}(:,2)] = pol2cart(Z{i,1}(:,1), Z{i,1}(:,2));
   
    R = max(Z{i,1}(:,2));

    if R>R_max
        R_max = R;
    end
end
%% 3b) (lidar) local to global
for i = 1:length(Z_xy_loc)
    Z_xy{i,1} = Z_xy_loc{i,1} - [x(i), y(i)]; % shifting wrt the origin
end
%% 4) Brute force 

% SLAM will store all the values

x_s = -1.2:0.3:1.2; %x_space
y_s = -0.7:0.3:0.7; %y_space
ang_space = -5:1:5; %plot(diff(unwrap(h)),".") <- has to be wrong (sample 556 and 557)

SLAM{1,1} = Z_xy_loc{1,1}; %init scan is correct

Fp = frame(Z_xy_loc{1,1}, R_max);

c = zeros(length(x_s), length(y_s), length(ang_space));
shifts = zeros(M, 3);

for k = 2:50 %frame ||M %AROUND 80 MIN/FRAME
    %x_s = (x_space +  );
    %y_s = (y_space );

    phi_s  = (ang_space + h(k-1))*pi/180;
    disp(['--------k=', num2str(k), '--------'])
    for ix = 1:length(x_s)
        for iy = 1:length(y_s)
            for iphi = 1:length(phi_s)
                Z_temp = T(Z_xy_loc{k,1}, x_s(ix), y_s(iy), phi_s(iphi));

                Fc = frame(Z_temp, R_max);
                c(ix, iy, iphi) = sum(Fp.*Fc, "all");
                
               % s = [num2str(ix),'/',num2str(length(x_s)),'; ', ...
                %     num2str(iy),'/',num2str(length(y_s)),'; ', ...
                 %    num2str(iphi),'/',num2str(length(phi_s)),'; '];
                %disp(s)
            end
        end
    end

    [~,ic_max] = max(c(:));
    [ix_max, iy_max, iphi_max] = ind2sub(size(c),ic_max);
    
    % mb below minus
    Z_v = T(Z_xy_loc{k,1}, x_s(ix_max), y_s(iy_max), phi_s(iphi_max)); % rotate and shift 
    % the other side

    shifts(k,:) = [x_s(ix_max), y_s(iy_max), phi_s(iphi_max)];
    % mb add bias <- GPS
    SLAM{k, 1} = Z_v;
    Fp = frame(Z_v, R_max);
end
%%
for i = 1:length(SLAM)
    SLAM{i,1} = SLAM{i,1} - [x(i), y(i)]; % shifting wrt the origin
end

Saved = 1;
%save 3frames.mat
%%
 figure;hold on;
 for i = 1:length(SLAM)
     plot(SLAM{i,1}(:,1),SLAM{i,1}(:,2), '.')
     pause(1.5)
 end
% plot(SLAM{1,1}(:,1),SLAM{1,1}(:,2), '.')
% hold on
% plot(SLAM{2,1}(:,1),SLAM{2,1}(:,2), '.')
% plot(SLAM{3,1}(:,1),SLAM{3,1}(:,2), '.')
% plot(SLAM{4,1}(:,1),SLAM{4,1}(:,2), '.')

%PLOT THE ORIGINAL(MESS)
%PLOT SLAM
%PLOT GPS AS THE GROUND TRUTH
%PLOT ERROR ON X AXIS
%PLOT ERROR ON Y AXIS
%% 
% figure;
% hold on;
% for k = 1:M-1
%     plot(Z_xy{k,1}(:,1),Z_xy{k,1}(:,2), '.')
%     pause(0.1)
% end
%% 5) Results comparison