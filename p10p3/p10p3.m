addpath('../Data logs/p2')

init04;
load('p10p2_optimal_trajectory.mat')

%% Model definition
% Constants
PI  = 3.1516;
delta_t	= 0.25;          %sampling time
h = delta_t;             %time step

A1 = [1 h 0 0 ;
    0 1 -h*K_2 0;
    0 0 1 h;
    0 0 -h*K_1*K_pp 1-h*K_1*K_pd];

B1 = [0; 0; 0; h*K_1*K_pp];

% Initial values
x1_0 = PI;                          % Lambda
x2_0 = 0;                               % r
x3_0 = 0;                               % p
x4_0 = 0;                               % p_dot
x0 = [x1_0 x2_0 x3_0 x4_0]';           % Initial values

%% LQR cost parameeters and calculation
q_lambda = 80; 
q_r = 10; 
q_pitch = 1;
q_pitch_dot = 10;

R_rpc = 1;

Q = diag([q_lambda, q_r, q_pitch, q_pitch_dot]);
R = R_rpc;

K = dlqr(A1,B1,Q,R,zeros(4,1));
K_transpose = K';

%% Time horizon
sim_time = 25;
hover_time = 5;
N  = 1/delta_t*sim_time;                % Time horizon for states
M  = N;                                 % Time horizon for inputs
nx = N*length(x0);

%% Number of states and inputs
mx = size(A1,2); % Number of states (number of columns in A)
mu = size(B1,2); % Number of inputs(number of columns in B)
nx = N*length(x0);

%% Extract control inputs and states
u  = [z(N*mx+1:N*mx+M*mu);z(N*mx+M*mu)]; % Control input from solution

x1 = [x0(1);z(1:mx:N*mx)];              % State x1 from solution
x2 = [x0(2);z(2:mx:N*mx)];              % State x2 from solution
x3 = [x0(3);z(3:mx:N*mx)];              % State x3 from solution
x4 = [x0(4);z(4:mx:N*mx)];              % State x4 from solution

% zero padding for hovering

num_variables = hover_time/delta_t;
zero_padding = zeros(num_variables,1);
unit_padding  = ones(num_variables,1);

u   = [zero_padding; u; zero_padding];
x1  = [pi*unit_padding; x1; zero_padding];
x2  = [zero_padding; x2; zero_padding];
x3  = [zero_padding; x3; zero_padding];
x4  = [zero_padding; x4; zero_padding];

x_star_vector = zeros(N+1+2*num_variables,4);
x_star_vector(:,1) = x1;
x_star_vector(:,2) = x2;
x_star_vector(:,3) = x3;
x_star_vector(:,4) = x4;

t = 0:h:sim_time+2*hover_time;
x_star_values = struct('values', x_star_vector);
x_star = struct('time', t, 'signals', x_star_values);

pitch_values = struct('values', u);
p_c_star = struct('time', t, 'signals', pitch_values);

