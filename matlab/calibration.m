%% Load calibration data 
calibration_file = '../calibration_result.txt';
data = load(calibration_file); % slide (m) / pan (rad) / tilt (rad) / pose mat (4x4) flattened in row major 

%% Symbolic definition 
state = sym('q',[3 1],'real'); % slide (m, +: right direction) / pan (rad) / tilt (rad)
t_pt = sym('p1',[3 1],'real'); % translation from pann to tilt frame 
t_tc = sym('p2',[3 1],'real'); % translation from title to camera frame 
M_q = sym('M',[3 6],'real'); % 


% ref frame to pan baset_oc
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
T_oc_0 = (subs(T_oc,state,data(1,1:3)')); % still symbolic w.r.t. p1 and p2 
R_oc_0 = double(subs(R_oc,state,data(1,1:3)'));
t_oc_0 = T_oc_0(1:3,4);

T_delta = [[R_oc_0'*R_oc R_oc'*(t_oc-t_oc_0)] ; [0 0 0 1]];
R_delta = T_delta(1:3,1:3);
t_delta = T_delta(1:3,4);


%% Check data sanity R_delta vs zed rotation ? 
element_diff_history = [];
t_zed_history = [];
t_delta_history = [];

for n = 1:size(data,1) 
    edelkrone_state = data(n,1:3)';
    T_zed = reshape(data(n,4:end),4,4)';
    R_zed = T_zed(1:3,1:3); t_zed_history = [t_zed_history ; T_zed(1:3,4)];
    % the delvelop of rotation does not rely on edelkrone dimension..?
    R_delta_subs = double(subs(R_delta, state, edelkrone_state));
    t_delta_subs = subs(t_delta,state,edelkrone_state); t_delta_history = [t_delta_history ; t_delta_subs];
    
    element_diff = norm(R_zed-R_delta_subs);
    element_diff_history = [element_diff_history element_diff];    
end

figure(1);
plot(element_diff_history,'ko-');
title("$||R_{0}'R_{1} - R_{zed}||$",'Interpreter','latex')

M = double(equationsToMatrix(t_delta_history, [t_pt t_tc])) 
ns = null(M); % tells that z axis of t_pt and (y-axis of t_pt + z axis of t_pc) do not produce  
% In conclusion, p11, p12, p12-p21, p23 only matter 
n_rank = rank(M);
[U,S,V] = svd(M);
vs = V(:,1:4); % basis. x = vs*wv + ns*wm = xv + xn
Mv = M*vs % y = M * (xv+xn) = M * xv = (M*vs) * wv. Our optim variable is wv now.  
sum(sum(abs(M*ns))) % numerical error: not exactly zero

%% Finding t_pt / t_tc 

wv = Mv\t_zed_history;
wn = sym('x_n',[2 1],'real');
x_sol = vs*wv 
error = abs(t_zed_history - (Mv*wv)); % y element of tilt axis not good..

