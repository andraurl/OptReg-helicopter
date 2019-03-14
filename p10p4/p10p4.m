
addpath('../Data logs')
addpath('../Help functions')
addpath('../Common files')

init04;
clc;

% Constants
delta_t	= 0.25;          %sampling time
h = delta_t;             %time step
%PI  = 3.1516;
% x = [lambda r p p_dot e e_dot]'
% u = [p_c e_c]'

% Discrete time system model. x = [lambda r p p_dot e e_dot]'
A_l = [ 1 h 0 0 0 0;
        0 1 -h*K_2 0 0 0;
        0 0 1 h 0 0;
        0 0 -h*K_1*K_pp 1-h*K_1*K_pd 0 0;
        0 0 0 0 1 h;
        0 0 0 0 -h*K_3*K_ep 1-h*K_3*K_ed];

B_l = [0 0;
       0 0;
       0 0;
       h*K_1*K_pp 0;
       0 0;
       0 h*K_3*K_ep];

% Number of states and inputs
mx = size(A_l,2); % Number of states (number of columns in A)
mu = size(B_l,2); % Number of inputs(number of columns in B)

% Initial values extended
x1_0 = pi;                          % Lambda
x2_0 = 0;                               % r
x3_0 = 0;                               % p
x4_0 = 0;                               % p_dot
x5_0 = 0;                               % e
x6_0 = 0;                               % e_dot
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0 x6_0]';   % Initial values

%Final Values
x_f = [0 0 0 0 0 0]';                    % Final values

% Time horizon and initialization
N = 40;
M = N;
nx = N*mx;
mz = mx+mu; 
nz = N*mz;
z  = zeros(nz,1);                     % Initialize z for the whole horizon
z0 = z;                               % Initial value for optimization
z0(1) = pi;
% Bounds

ul    = [-30*pi/180, -inf]';          % Lower bound on control
uu  = -ul;                            % Upper bound on control

xl      = -Inf*ones(mx,1);            % Lower bound on states (no bound)
xu      = Inf*ones(mx,1);             % Upper bound on states (no bound)
xl(3)   = ul(1);                      % Lower bound on state x3
xu(3)   = uu(1);                      % Upper bound on state x3

% Generate constraints on measurements and inputs
[vlb,vub]       = gen_constraints(N,M,xl,xu,ul,uu); % hint: gen_constraints
vlb(nz)  = 0;                    % We want the last input to be zero
vub(nz)  = 0;                    % We want the last input to be zero

% Generate the matrix Q and the vector c (objecitve function weights in the QP problem) 
Q1 = zeros(mx,mx);
Q1(1,1) = 1;                            % Weight on state x1
Q1(2,2) = 0;                            % Weight on state x2
Q1(3,3) = 0;                            % Weight on state x3
Q1(4,4) = 0;                            % Weight on state x4
Q1(5,5) = 0;                            % Weight on state x5
Q1(6,6) = 0;                            % Weight on state x6
q_1 = 1;                                % høy vil gi tregt helikopter. Lav vil gi store utslag i vinkler.
q_2 = 0.05;                                % høy q23 vil gi tettere bane til unlin constraint
P1 = diag([q_1 q_2]);                   % Weight on input
Q = gen_q(Q1,P1,N,M);                   % Generate Q, hint: gen_q
c = zeros(nz,1);               % Generate c, this is the linear constant term

%% Generate system matrixes for linear model
A_eq = gen_aeq(A_l,B_l,N,mx,mu);         % Generate A, hint: gen_aeq
B_eq = zeros(nx,1);                      % Generate b
B_eq(1:mx) = A_l*x0;

alpha = 0.2;
beta = 20;
lambda_t = 2*pi/3;

%% LQR Feedback control
q_lambda = 100;
q_r = 1;
q_pitch = 12;
q_pitch_dot = 20; 
q_e = 50; 
q_e_dot = 3;

Q_lqr = diag([q_lambda q_r q_pitch q_pitch_dot q_e q_e_dot]);

r_p_c = 1;
r_e = 0.6;
R_lqr = diag([r_p_c r_e]);

K = dlqr(A_l,B_l,Q_lqr,R_lqr,zeros(mx,mu));
K_transpose = K';

%% Fmincon open loop optimal path
fun = @(z)z'*Q*z;

opt = optimoptions('fmincon','Algorithm', 'sqp','MaxFunEvals',40000);


[z, ZVAL] = fmincon(fun,z0,[],[],A_eq,B_eq,vlb,vub,@unlincon,opt);


%% To Simulink
padding_time = 5;
padding_samples = padding_time/h;
t = (0:h:(N*h+2*padding_time));



zero_padding_x = zeros(mx,padding_samples);
zero_padding_u = zeros(mu,padding_samples);

x1 = [x0(1);z(1:mx:nx)];              % State x1 from solution
x2 = [x0(2);z(2:mx:nx)];              % State x2 from solution
x3 = [x0(3);z(3:mx:nx)];              % State x3 from solution
x4 = [x0(4);z(4:mx:nx)];              % State x4 from solution
x5 = [x0(5);z(5:mx:nx)];              % State x5 from solution
x6 = [x0(6);z(6:mx:nx)];              % State x6 from solution

u0 = [0 0];                           %Output at time 0
u1 = [u0(1);z(nx+1:mu:nz)];           % Output u1 from solution
u2 = [u0(2);z(nx+2:mu:nz)];         % Output u1 from solution

x1 = [x0(1)*ones(padding_samples,1); x1; zeros(padding_samples,1)];         % Padded x1
x2 = [zeros(padding_samples,1); x2; zeros(padding_samples,1)];              % Padded x2
x3 = [zeros(padding_samples,1); x3; zeros(padding_samples,1)];              % Padded x3
x4 = [zeros(padding_samples,1); x4; zeros(padding_samples,1)];              % Padded x4
x5 = [zeros(padding_samples,1); x5; zeros(padding_samples,1)];              % Padded x5
x6 = [zeros(padding_samples,1); x6; zeros(padding_samples,1)];              % Padded x6

u1 = [zeros(padding_samples,1); u1; zeros(padding_samples,1)];
u2 = [zeros(padding_samples,1); u2; zeros(padding_samples,1)];

x_star_values = [x1 x2 x3 x4 x5 x6];
x_star_signals = struct('values', x_star_values);
x_star = struct('time', t, 'signals', x_star_signals);                      % x_star as structure with time 

u_star_values = [u1 u2];
u_star_signals = struct('values',u_star_values); 
u_star = struct('time', t, 'signals', u_star_signals);                      % u_star as structure with time 

   