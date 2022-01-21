%% Load calibration data 
% calibration_file = '../20220121151001/calibration_result.txt';
calibration_file = '../calibration_result3.txt';

data = load(calibration_file); % slide (m) / pan (rad) / tilt (rad) / pose mat (4x4) flattened in row major 

%% Calibration data check
figure(10)
sgtitle("Edelkrone state")
subplot(3,1,1)
h_slide = plot(data(:,1));
title("slide [m]")

subplot(3,1,2)
h_pan = plot(data(:,2));
title("pan [rad]")

subplot(3,1,3)
h_title = plot(data(:,3));
title("tilt [rad]")

fps = 20;
figure(13)
cla
title("Cam pose history")
grid on
view([46 30])
hold on
axis equal
axis([-0.2 0.2 -0.2 0.2 -0.1 0.1])
for n = 1:2:size(data,1)
    T_zed = reshape(data(n,4:end),4,4)';
    pose.R = T_zed(1:3,1:3);
    pose.t = T_zed(1:3,4);
    SE3plot(pose,0.02,1.6)    
    pause(1.0/fps)
end
xlabel('x')
ylabel('y')
zlabel('z')


%% Symbolic definition 
state = sym('q',[3 1],'real'); % slide (m, +: right direction) / pan (rad) / tilt (rad)
t_pt = sym('p1',[3 1],'real'); % translation from pann to tilt frame 
t_tc = sym('p2',[3 1],'real'); % translation from title to camera frame 
M_q = sym('M',[3 6],'real'); % 

% ref frame to pan baset_oc
R_op = [cos(state(2)) -sin(state(2)) 0; sin(state(2)) cos(state(2)) 0; 0 0 1]; 
t_op = [0 -state(1) 0]'; T_op = [[R_op t_op] ; [ 0 0 0 1]];

% pan base to tile base
R_pt = [0 0 -1 ; -1 0 0; 0 1 0]*[cos(state(3)) -sin(state(3)) 0; sin(state(3)) cos(state(3)) 0; 0 0 1]; 
T_pt = [[R_pt t_pt]; [0 0 0 1]];

% tilt base to cam base 
R_tc = [1 0 0; 0 0 1 ; 0 -1 0]; T_tc = [[R_tc t_tc]; [0 0 0 1]]; 

% Pose w.r.t edel krone state 
T_oc = T_op*T_pt*T_tc;
t_oc = T_oc(1:3,4);
R_oc = T_oc(1:3,1:3);

% initial camera pose (unknown)
initial_state = [0 pi/2.0 0]';  
% T_oc_0 = (subs(T_oc,state,data(1,1:3)')); % still symbolic w.r.t. p1 and p2 
% R_oc_0 = double(subs(R_oc,state,data(1,1:3)'));
T_oc_0 = (subs(T_oc,state,initial_state)); % still symbolic w.r.t. p1 and p2 
R_oc_0 = double(subs(R_oc,state,initial_state));
t_oc_0 = T_oc_0(1:3,4);

T_delta = [[R_oc_0'*R_oc R_oc_0'*(t_oc-t_oc_0)] ; [0 0 0 1]];
R_delta = T_delta(1:3,1:3);
t_delta = T_delta(1:3,4); % translation difference w.r.t R_oc_0


%% Check data sanity R_delta vs zed rotation ? 
% In the current system, R_delta can be fully defined by edelkrone state (pan/tilt)
element_diff_history = [];
t_zed_history = []; % zed visual odometry 
t_delta_history = []; 
total_optim_num = size(data,1);

for n = 1: total_optim_num
    edelkrone_state = data(n,1:3)';
    T_zed = reshape(data(n,4:end),4,4)';
    R_zed = T_zed(1:3,1:3); t_zed_history = [t_zed_history ; T_zed(1:3,4)];
    % the delvelop of rotation does not rely on edelkrone dimension.
    R_delta_subs = double(subs(R_delta, state, edelkrone_state));
    t_delta_subs = subs(t_delta,state,edelkrone_state); 
    t_delta_history = [t_delta_history ; t_delta_subs];
    
    element_diff = norm(R_zed-R_delta_subs);
    element_diff_history = [element_diff_history element_diff];    
end

figure(1);
subplot(2,1,1)
plot(element_diff_history,'ko-');
title("$||R_{0}'R_{1} - R_{zed}||$",'Interpreter','latex')

%% best fitting kinematic parameters? 

M = double(equationsToMatrix(t_delta_history, [t_pt t_tc])) ;
ns = null(M); % tells that z axis of t_pt and (y-axis of t_pt + z axis of t_pc) do not produce  
% In conclusion, p11, p12, p12-p21, p23 only matter 
n_rank = rank(M);
[U,S,V] = svd(M);
vs = V(:,1:4); % rank = 4 basis. x = vs*wv + ns*wm = xv + xn
Mv = M*vs; % y = M * (xv+xn) = M * xv = (M*vs) * wv. Our optim variable is wv now.  
nullspace_is_not_zero = sum(sum(abs(M*ns))); % numerical error: not exactly zero

%% Finding kinematic parameters t_pt / t_tc 
weight = diag([1 1 1]);
big_eye = zeros(length(t_zed_history));
for n = 1: (length(t_zed_history)/3)
   big_eye(3*n-2:3*n,3*n-2:3*n) = weight;
end

wv = (big_eye*Mv)\(big_eye*t_zed_history);
% wv = quadprog(Mv'*Mv, -t_zed_history'*Mv); % same with the above
wn = sym('x_n',[2 1],'real');
x_sol = vs*wv 
error = (abs(t_zed_history - (Mv*wv))); % y element of tilt axis not good..

%% Interpret result
x_sol =[
    0.0988
    0.0294
    0.0000
    0.0887
   -0.0493
   -0.0988
];
calibration_file = '../calibration_result3.txt';
% calibration_file = '../20220121151001/calibration_result.txt';
data = load(calibration_file); % slide (m) / pan (rad) / tilt (rad) / pose mat (4x4) flattened in row major 
xyz_zed = []; 
xyz_delta = []; 
for n = 1:size(data,1)
    edelkrone_state = data(n,1:3)';
    T_zed = reshape(data(n,4:end),4,4)';
    R_zed = T_zed(1:3,1:3); 
    t_zed = T_zed(1:3,4); % x,y,z of zed VO 
    xyz_zed = [xyz_zed  t_zed];
    
    t_delta_subs = subs(t_delta,state,edelkrone_state);
    t_delta_subs = subs(t_delta_subs,t_pt,x_sol(1:3));
    t_delta_subs = subs(t_delta_subs,t_tc,x_sol(4:6));
    xyz_delta = [xyz_delta double(t_delta_subs)];
end

figure(12)
colors = {'r','g','b'};
for i = 1:3
    subplot(3,1,i)
    hold on
    hzed = plot(xyz_zed(i,:),strcat(colors{i},'-'));
    hdelta = plot(xyz_delta(i,:),strcat(colors{i},'--'));
    legend([hzed hdelta],{'zed','delta'})
end






