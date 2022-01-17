%% Load calibration data 
calibration_file = '../calibration_result3.txt';
data = load(calibration_file); % slide (m) / pan (rad) / tilt (rad) / pose mat (4x4) flattened in row major 
t_pt_cal = [
   -0.0294
    0.0988
   -0.0000
    ];
t_tc_cal = [
    0.0887
   -0.0493
   -0.0988
    ]; 


%% Symbolic definition 
state = sym('q',[3 1],'real'); % slide (m, +: right direction) / pan (rad) / tilt (rad)
t_pt = sym('p1',[3 1],'real'); % translation from pann to tilt frame 
t_tc = sym('p2',[3 1],'real'); % translation from title to camera frame 


% ref frame to pan base_oc
R_op = [cos(state(2)) -sin(state(2)) 0; sin(state(2)) cos(state(2)) 0; 0 0 1]; t_op = [0 -state(1) 0]'; T_op = [[R_op t_op] ; [ 0 0 0 1]];

% pan base to tile base
R_pt = [1 0 0 ; 0 0 -1; 0 1 0]*[cos(state(3)) -sin(state(3)) 0; sin(state(3)) cos(state(3)) 0; 0 0 1]; T_pt = [[R_pt t_pt]; [0 0 0 1]];

% tilt base to cam base 
R_tc = [1 0 0; 0 0 1 ; 0 -1 0]; T_tc = [[R_tc t_tc]; [0 0 0 1]]; 

% Pose w.r.t edel krone state 
T_oc = T_op*T_pt*T_tc;
t_oc = T_oc(1:3,4);
R_oc = T_oc(1:3,1:3);

M = equationsToMatrix(t_oc,[t_pt t_tc]);

% initial camera pose (unknown)
initial_state = [0 pi/2.0 0]'; 
% T_oc_0 = (subs(T_oc,state,data(1,1:3)')); % still symbolic w.r.t. p1 and p2 
% R_oc_0 = double(subs(R_oc,state,data(1,1:3)'));
T_oc_0 = (subs(T_oc,state,initial_state)); % still symbolic w.r.t. p1 and p2 
R_oc_0 = double(subs(R_oc,state,initial_state));
t_oc_0 = T_oc_0(1:3,4);

T_delta = [[R_oc_0'*R_oc R_oc_0'*(t_oc-t_oc_0)] ; [0 0 0 1]];
R_delta = T_delta(1:3,1:3);
t_delta = T_delta(1:3,4); % translation difference 


%% Check data sanity R_delta vs zed rotation ? 
rotation_diff_history = [];
transl_diff_history = []; 
t_zed_history = []; % zed transition (relative to original frame)
t_delta_history = []; % forward kinematics transition (relative to original frame)

for n = 1:size(data,1) 
    edelkrone_state = data(n,1:3)';
    T_zed = reshape(data(n,4:end),4,4)';
    R_zed = T_zed(1:3,1:3); 
    t_zed = T_zed(1:3,4);
    t_zed_history = [t_zed_history  t_zed];
    
    % the delvelop of rotation does not rely on edelkrone dimension..?
    R_delta_subs = double(subs(R_delta, state, edelkrone_state));
    t_delta_subs = subs(t_delta,state,edelkrone_state);
    t_delta_subs = subs(t_delta_subs,t_pt,t_pt_cal);
    t_delta_subs = subs(t_delta_subs,t_tc,t_tc_cal);
    
    t_delta_history = [t_delta_history double(t_delta_subs)];
    
    rot_diff = norm(R_zed-R_delta_subs);
    rotation_diff_history = [rotation_diff_history rot_diff];    
    
    xyz_diff = abs(t_zed -double(t_delta_subs));
    transl_diff_history = [transl_diff_history xyz_diff];    
end

figure(1)
subplot(2,1,1)
plot(rotation_diff_history,'ko-');
title("$||R_{0}'R_{1} - R_{zed}||$",'Interpreter','latex')

subplot(2,1,2)
hold on
plot(transl_diff_history(1,:),'ro-')
plot(transl_diff_history(2,:),'go-')
plot(transl_diff_history(3,:),'bo-')
title("$||t_{dev} - t_{zed}||$",'Interpreter','latex')


%% 
figure(2)
hold on
plot3(t_zed_history(1,:),t_zed_history(2,:),t_zed_history(3,:),'k-')
plot3(t_delta_history(1,:),t_delta_history(2,:),t_delta_history(3,:),'b-')
xlabel('x')
ylabel('y')

