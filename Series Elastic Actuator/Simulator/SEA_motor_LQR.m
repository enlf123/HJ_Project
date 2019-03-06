clear all
clc 
%%
global no_load_speed no_load_current tor_constant
global J_motor k_SEA d_SEA J_load
global Tor

%Dynamixel MX-64
no_load_speed = 78*2*pi()/60; %rad/s
no_load_current = 0.2;  %A
tor_constant = 1.4634;  %Nm/A
J_motor = 3000*10^-7;  % g*cm^2
%

k_SEA = 30; %Nm/rad
d_SEA = 0.1;  %Nms/rad
J_load = 0.001; %kg*m^2
I_max = 3.5; 
dq_m_max = 5;


%% gain  tor p올리면 떨리는게 심해짐 vel p,i올리면 시스템 속도가 빨라짐
kp_vel = 0.30;  %45
ki_vel = 0;  %1500
kd_vel = 0;
I_pid(1,:) = zeros(1,3);

%% LQR Design
C = [50 0 0;
0 120 0;
0 0 1];
Q = C'*C;
R = 0.1;
A = [0 0 0;
0 0 1;
k_SEA/J_load -k_SEA/J_load -d_SEA/J_load];
B= [1;
0;
d_SEA/J_load];

[G, P] = lqr(A,B,Q,R);
% G = [60,12,0.2];
% G= [70,20,0.1];
% G=[42.7834699282007,-15,0.414138122609269];
%% Simulator variable
dt = 1/2000;
dt_tor = 1/400;
dt_vel = 1/1000;
total_time = 1;
t(:,1) = 0:dt:total_time;
for i=1:length(t)
    if i < length(t)/2
        q_j_d(i) = 0.7;
    else
        q_j_d(i) = -0.3;
    end
end
q_m_d = q_j_d;
dq_j_d = zeros(length(t),1);
dq_m_d = zeros(length(t),1);
q_m = zeros(length(t),1);
dq_m = zeros(length(t),1);
q_j = zeros(length(t),1);
dq_j = zeros(length(t),1);
tor_j = zeros(length(t),1);

Tor = 0;
X(1,:)= [q_m(1), dq_m(1), q_j(1), dq_j(1)];
options = odeset('RelTol',1e-4,'AbsTol',1e-6);
for i=1:length(t)-1
    tspan=[t(i) t(i+1)];
    [time,Y] = ode45(@SEA_ode,tspan,X(i,:),options);
    n=length(time);
    X(i+1,:)=Y(n,:);
    q_m(i+1)=X(i+1,1);     dq_m(i+1)=X(i+1,2);
    q_j(i+1)=X(i+1,3);     dq_j(i+1)=X(i+1,4);
    tor_j(i+1) = k_SEA*(q_m(i+1)-q_j(i+1)) + d_SEA*(dq_m(i+1)-dq_j(i+1));
    if mod(t(i+1),dt_tor) == 0        
    dq_m_d(i+1) = G(1)*(q_m_d(i+1)-q_m(i+1))+G(2)*(q_j_d(i+1)-q_j(i+1))+G(3)*(dq_j_d(i+1)-dq_j(i+1));
    else
        dq_m_d(i+1) = dq_m_d(i);
    end
    if dq_m_d(i+1) > dq_m_max
       dq_m_d(i+1) =  dq_m_max;
    end
    if dq_m_d(i+1) < -dq_m_max
       dq_m_d(i+1) =  -dq_m_max;
    end
    
    if mod(t(i+1),dt_vel) == 0
    I_pid(i+1,:) = f_pid([kp_vel,ki_vel, kd_vel, dq_m_d(i+1), dq_m(i+1), dt_vel, I_pid(i,1), I_pid(i,2), I_pid(i,3)]);
    I_m_d(i+1,1) = I_pid(i+1,3);
    else
    I_pid(i+1,:) = I_pid(i,:);
    I_m_d(i+1,1) = I_pid(i,3);
    end
    
    if I_m_d(i+1) > I_max
        I_m_d(i+1) = I_max;
    end
    if I_m_d(i+1) < -I_max
        I_m_d(i+1) = -I_max;
    end
    Tor = tor_constant*I_m_d(i+1);
end
tor_m = tor_constant*I_m_d;
% Torque figure
figure(2)
set(gcf, 'Position', [1200, 250, 500, 400])
plot(t, tor_j);
hold on
xlabel('Time(s)','Fontsize',14)
ylabel('Torque   Joint(Nm)','Fontsize',14)
hold off

% angle figure
figure(3)
set(gcf, 'Position', [650, 100, 500, 600])
subplot(3,1,1:2)
plot(t, q_j);
hold on 
plot(t, q_j_d);
legend('Joint Angle', 'Desired Joint Angle');
xlabel('Time(s)','Fontsize',14)
ylabel('Joint Angle(rad)','Fontsize',14)
hold off
subplot(3,1,3)
plot(t, dq_j);
hold on 
legend('Joint Angular Velocity');
xlabel('Time(s)','Fontsize',14)
ylabel('Joint Angle Velocity(rad/s)','Fontsize',14)
hold off


filename = 'SEA_LQR.gif';  % gif filename
for  i=1:50:length(t)
    motor_figure(q_m(i),q_j(i));
    time = strcat(num2str(t(i)));
    text(20,30,time,'Fontsize',13)
    hold off
end