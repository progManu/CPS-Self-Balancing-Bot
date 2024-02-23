%% Clear env variables
clear;
clc;

% run("staliro\setup_staliro.m");
                                                                           % execute run(...) only first time running the script to install s-taliro

%% Model variables
m_cart = 0.558;                                                            % 0.558 Kg
m_pend = 1.267;                                                            % 1.267 Kg
I_pend = 0.055;                                                            % 0.055 Kg*m^2
l = 0.159;                                                                 % 0.159 m
mu_drag = 0.00002;                                                         % drag_coefficient
g = 9.81;                                                                  % gravity acceleration

                                                                           % IN THIS MODEL THE EQUATION UNSTABLE EQUILIBRIUM POINT IS [0, 0, PI, 0]
                                                                           % WHILE THE STABLE ONE IS [0, 0, 0, 0].

%% EKF Parameters
dt = 0.01;                                                                 % sampling time
x_init = [0; 0; (11/12)*pi; 0];                                            % initial state
initial_cov = eye(4)*5e-4;                                                 % initial covariance matrix
measurement_cov = eye(2)*5e-4;                                             % measurment covariace

%% PID Tuning

x = sym('x', [1, 4]);                                                      % symbolic state-space

u = sym('u');                                                              % symbolic input

m_tot = m_cart + m_pend;                                                   % working parameter

dx1 = x(2);                                                                % cart speed
dx2 = (u - mu_drag*x(2) - m_pend*l*(((-m_pend*g*l*sin(x(3))*cos(x(3)))/...
    (I_pend + m_pend*l^2)) - (x(4)^2)*sin(x(3))))/...
    (m_tot*(1 - (((m_pend^2)*(l^2)*(cos(x(3))^2))/(I_pend + m_pend*l^2))));% cart acceleration
dx3 = x(4);                                                                % angular speed
dx4 = (-m_pend*g*l*sin(x(3)) -m_pend*l*dx2*cos(x(3)))/...
    (I_pend + m_pend*l^2);                                                 % angular acceleration

xddot = [                                                                  % derivatives vector
  dx1;
  dx2;
  dx3;
  dx4
];

A = jacobian(xddot, x);                                                    % A matrix obtained by linearization
B = jacobian(xddot, u);                                                    % B matrix obtained by linearization

A = double(subs(A, x, [0, 0, pi, 0]));                                     % matrix to double
B = double(subs(B, x, [0, 0, pi, 0]));                                     % matrix to double

C = [1 0 0 0;
     0 0 1 0];                                                             % C matrix
D = 0;                                                                     % D matrix

clear x u;                                                                 % clear variables

sys = ss(A, B, C, D);

sysd = c2d(sys, dt);

A = sysd.A;
B = sysd.B;

Co = ctrb(A, B);
Ob = obsv(A, C);

rCo = rank(Co);
rOb = rank(Ob);

fprintf(['Rank of controllability matrix: %d' ...
    ', rank of observability matrix: %d\n'], rCo, rOb);


Kp = 15;                                                                   % optimal Kp
Ki = 5;                                                                    % optimal Ki
Kd = 3;                                                                    % optimal Kd

Kp_values = linspace(10, 20, 11);
                                                                           % Kp range to find the optimal value
Ki_values = linspace(0, 10, 11);
                                                                           % Ki range to find the optimal value
Kd_values = linspace(0, 10, 11);
                                                                           % Kd range to find the optimal value

noise_s1_value = 5e-5;                                                     % nominal noise pwr position
noise_s2_value = 5e-5;                                                     % nominal noise pwr angle

%% Requirement verification for nominal values

model_name = 'self_balancing_slink';                                       % Simulink model name
simopt = simget(model_name);                                               % getting the model
simopt = simset(simopt,'solver', 'ode45', 'FixedStep', dt, ...
    'SaveFormat','Array');                                                 % setting simulation options
sim_result = sim(model_name,[0 10], simopt);                               % getting the simulation results

time = sim_result.yout{1}.Values.Time;                                     % extracting discrete time
output = sim_result.yout{1}.Values.Data;                                   % extracting state-space estimates
input = sim_result.yout{2}.Values.Data;                                    % extracting PID generated system's input

angle = output(:, 3);                                                      % getting angle from state-space
position = output(:, 1);                                                   % getting position from state-space

ss_angle = abs(angle - pi);                                                % generating angle's secondary signal
ss_position = abs(position - 0);                                           % generating position's secondary signal

st_spec1 = '[] (a1)';                                                      % STL first specification (alwais [abs(angle - pi) < pi/12])

st_spec1_Pred(1).str = 'a1';
st_spec1_Pred(1).A = 1;
st_spec1_Pred(1).b = pi/12;

st_spec2 = '[] (a1)';                                                      % STL second specification (alwais [abs(position - 0) < 2])
st_spec2_Pred(1).str = 'a1';
st_spec2_Pred(1).A = 1;
st_spec2_Pred(1).b = 2;

rob1  = fw_taliro(st_spec1,st_spec1_Pred,ss_angle,time);                   % getting robustness for first STL formula
rob2  = fw_taliro(st_spec2,st_spec2_Pred,ss_position,time);                % getting robustness for second STL formula

fprintf('rob1: %f, rob2: %f\n', rob1, rob2);

%% Complete verification process
% IF YOU RUN THIS PART YOU HAVE TO STAY CLOSE TO THE PC BECAUSE MATLAB
% CAN'T SPECIFY A SIMULATION TIME LIMIT AND SOME CONFIGURATION PARAMETERS
% CAN LEAD TO LONG COMPUTATION TIME, IN ORDER TO HAVE A FEASIBLE RUNNING
% TIME THIS ONES CAN BE SKIPED BY PRESSING THE STOP BUTTON WHEN WE SEE THAT
% THE SIMULATION IS TAKING MORE THAT 3 SECONDS. (THE ERROR CONFIGURATIONS
% ARE INSTEAD SKIPPED BY THE CODE ITSELF USING TRY-CATCH)
% (SINCE THE FOR LOOP IS VERY LONGSOME TIME MATLAB DOENS'T RESPOND, 
% CHECK THE MATLAB DOCUMENTATION TO SEE
% THE DETAILS, THE LOOP WORKS IS JUST A MATTER OF LETTING IT FINISH)
% https://it.mathworks.com/help/simulink/ug/using-the-sim-command.html
% https://it.mathworks.com/matlabcentral/answers/94419-why-am-i-unable-to-consistently-use-ctrl-c-to-break-out-of-an-operation-in-matlab-6-0-r12-and-late
% TO EXIT THE SIMULATION PUT THE CURSOR IN THE COMMAND WINDOW AND HOLD
% CTLR+C UTIL THE SIMULATION STOPS

warning('off','all');

[Kg_p, Kg_i, Kg_d] = meshgrid(Kp_values, Ki_values, Kd_values);            % cartesian product of the PID parameters

Kg_p = reshape(Kg_p,1,[])';                                                % vectorize Kg_p
Kg_i = reshape(Kg_i,1,[])';                                                % vectorize Kg_i
Kg_d = reshape(Kg_d,1,[])';                                                % vectorize Kg_d

cartesian_product_size = size(Kg_p);                                       % calculating vector size

rob1_values = zeros(cartesian_product_size);                               % initial values for robustness STL first formula evaluation
rob2_values = zeros(cartesian_product_size);                               % initial values for robustness STL second formula evaluation

for i=1:1:cartesian_product_size
    drawnow;

    Kp = Kg_p(i);                                                          % setting Kp
    Ki = Kg_i(i);                                                          % setting Ki
    Kd = Kg_d(i);                                                          % setting Kv

    try                                                                    % try and catch done because some configurations can raise errors
        model_name = 'self_balancing_slink';                               % same code as: Requirement verification for nominal values
        simopt = simget(model_name);
        simopt = simset(simopt,'solver', 'ode45', 'FixedStep', dt, 'SaveFormat','Array');
        sim_result = sim(model_name,[0 10], simopt);
    
        time = sim_result.yout{1}.Values.Time;
        output = sim_result.yout{1}.Values.Data;
        input = sim_result.yout{2}.Values.Data;
    
        angle = output(:, 3);
        position = output(:, 1);
    
        ss_angle = abs(angle - pi);
        ss_position = abs(position - 0);
    
        st_spec1 = '[] (a1)';
    
        st_spec1_Pred(1).str = 'a1';
        st_spec1_Pred(1).A = 1;
        st_spec1_Pred(1).b = pi/12;
    
        st_spec2 = '[] (a1)';
    
        st_spec2_Pred(1).str = 'a1';
        st_spec2_Pred(1).A = 1;
        st_spec2_Pred(1).b = 2;
    
        rob1  = fw_taliro(st_spec1,st_spec1_Pred,ss_angle,time);
        rob2  = fw_taliro(st_spec2,st_spec2_Pred,ss_position,time);

        rob1_values(i) = rob1;
        rob2_values(i) = rob2;
    catch
        rob1_values(i) = NaN;                                              % setting robustness to NaN if we encounter an error
        rob2_values(i) = NaN;
    end
    fprintf(['Simulation %d of %d, pid {%f, %f, %f}, ' ...
        'robs: {%f, %f}\n'], i, cartesian_product_size(1), ...
        Kp, Ki, Kd, rob1_values(i), rob2_values(i));                       % display progress
end
%% Complete falsification process
% IF YOU RUN THIS PART YOU HAVE TO STAY CLOSE TO THE PC BECAUSE MATLAB
% CAN'T SPECIFY A SIMULATION TIME LIMIT AND SOME CONFIGURATION PARAMETERS
% CAN LEAD TO LONG COMPUTATION TIME, IN ORDER TO HAVE A FEASIBLE RUNNING
% TIME THIS ONES CAN BE SKIPED BY PRESSING THE STOP BUTTON WHEN WE SEE THAT
% THE SIMULATION IS TAKING MORE THAT 3 SECONDS. (THE ERROR CONFIGURATIONS
% ARE INSTEAD SKIPPED BY THE CODE ITSELF USING TRY-CATCH)
% (SINCE THE FOR LOOP IS VERY LONGSOME TIME MATLAB DOENS'T RESPOND, 
% CHECK THE MATLAB DOCUMENTATION TO SEE
% THE DETAILS, THE LOOP WORKS IS JUST A MATTER OF LETTING IT FINISH)
% https://it.mathworks.com/help/simulink/ug/using-the-sim-command.html
% https://it.mathworks.com/matlabcentral/answers/94419-why-am-i-unable-to-consistently-use-ctrl-c-to-break-out-of-an-operation-in-matlab-6-0-r12-and-late
% TO EXIT THE SIMULATION PUT THE CURSOR IN THE COMMAND WINDOW AND HOLD
% CTLR+C UTIL THE SIMULATION STOPS

noise_pwr1 = linspace(5e-5, 5e-3, 10);                                     % position noise power to find noise bounderies
noise_pwr2 = linspace(5e-5, 5e-3, 10);                                     % angle noise power to find noise bounderies

[np1, np2] = meshgrid(noise_pwr1, noise_pwr2);                             % cartesian product of noise values


np1 = reshape(np1,1,[])';                                                  % vectorize np1
np2 = reshape(np2, 1, [])';                                                % vectorize np2

cartesian_product_size = size(np1);                                        % cartesian product

rob1_values = zeros(cartesian_product_size);
rob2_values = zeros(cartesian_product_size);

for i=1:1:cartesian_product_size
    drawnow;
    
    noise_s1_value = np1(i);                                               % initial values for robustness STL first formula evaluation
    noise_s2_value = np2(i);                                               % initial values for robustness STL second formula evaluation

    try                                                                    % try and catch done because some configurations can raise errors
        model_name = 'self_balancing_slink';                               % same code as: Requirement verification for nominal values
        simopt = simget(model_name);
        simopt = simset(simopt,'solver', 'ode45', 'FixedStep', dt, 'SaveFormat','Array');
        sim_result = sim(model_name,[0 10], simopt);
        
        time = sim_result.yout{1}.Values.Time;
        output = sim_result.yout{1}.Values.Data;
        input = sim_result.yout{2}.Values.Data;
        
        angle = output(:, 3);
        position = output(:, 1);
        
        ss_angle = abs(angle - pi); % secondary signal
        ss_position = abs(position - 0); % secondary signal
        
        st_spec1 = '[] (a1)'; % signal-time specification 1
        
        st_spec1_Pred(1).str = 'a1';
        st_spec1_Pred(1).A = 1;
        st_spec1_Pred(1).b = pi/12;
        
        st_spec2 = '[] (a1)'; % signal-time specification 2
        
        st_spec2_Pred(1).str = 'a1';
        st_spec2_Pred(1).A = 1;
        st_spec2_Pred(1).b = 2;
        
        rob1  = fw_taliro(st_spec1,st_spec1_Pred,ss_angle,time);
        rob2  = fw_taliro(st_spec2,st_spec2_Pred,ss_position,time);
    
        rob1_values(i) = rob1;
        rob2_values(i) = rob2;
    catch
        rob1_values(i) = NaN;                                              % setting robustness to NaN if we encounter an error
        rob2_values(i) = NaN;
    end

    fprintf(['Simulation %d of %d, noise values: {%f, %f},' ...
        ' robs: {%f, %f}\n'], i, cartesian_product_size(1), ...
        noise_s1_value, noise_s2_value, rob1_values(i), rob2_values(i));   % display progress
end

