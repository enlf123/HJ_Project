clear all
clc
%%
global no_load_speed no_load_current tor_constant
global J_motor k_SEA d_SEA J_load
global Tor

%motor spec
no_load_speed = 78*2*pi()/60; %rad/s
no_load_current = 0.2;  %A
tor_constant = 1.4634;  %Nm/A
J_motor = 3000*10^-7;  % g*cm^2

%
k_SEA = 24; %Nm/rad
d_SEA = 0.1;  %Nms/rad
J_load = 0.001; %kg*m^2
I_max = 8;
dq_m_max =8;

%% gain  tor p올리면 떨리는게 심해짐 vel p,i올리면 시스템 속도가 빨라짐
kp_vel = 0.3;  %45
ki_vel = 0;  %1500
kd_vel = 0;
kp_tor = 45; %20
ki_tor = 120;
kd_tor = 0;

I_pid(1,:) = zeros(1,3);
V_pid(1,:) = zeros(1,3);

%% simulator variable
dt = 1/2000;
dt_tor = 1/400;
dt_vel = 1/1000;
total_time = 1;
t(:,1) = 0:dt:total_time;
tor_j_d = -3*ones(round(length(t)/2),1);
tor_j_d(round(length(t)/2)+1:length(t),1) = -0*ones(length(t)-round(length(t)/2),1);
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
    X(i+1,3) = 0;       X(i+1,4) = 0;
    q_m(i+1)=X(i+1,1);     dq_m(i+1)=X(i+1,2);
    q_j(i+1)=X(i+1,3);     dq_j(i+1)=X(i+1,4);
    
    tor_j(i+1) = k_SEA*(q_m(i+1)-q_j(i+1));% + d_SEA*(dq_m(i+1)-dq_j(i+1));
    
    if mod(t(i+1),dt_tor) == 0
        V_pid(i+1,:) = f_pid([kp_tor,ki_tor, kd_tor, tor_j_d(i+1), tor_j(i+1), dt_tor, V_pid(i,1), V_pid(i,2), V_pid(i,3)]);
        dq_m_d(i+1,1) = V_pid(i+1,3);
    else
        V_pid(i+1,:) = V_pid(i,:);
        dq_m_d(i+1,1) = V_pid(i,3);
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
    
    Tor = tor_constant*I_m_d(i+1);
    if abs(q_m(i+1)-q_j(i+1)) > 10
        asda;
    end

end
tor_m = tor_constant*I_m_d;
figure(2)
set(gcf, 'Position', [1200, 250, 500, 400])
plot(t, tor_j);
hold on
plot(t, tor_j_d);
legend('joint torque', 'desired torque');
xlabel('Time(s)','Fontsize',14)
ylabel('Torque   Joint(Nm)','Fontsize',14)
hold off



% filename = 'SEA_PI.gif';  % gif filename
% for  i=1:50:length(t)-1
%     
%     motor_figure(q_m(i+1),q_j(i+1));
%     time = strcat(num2str(t(i+1)));
%     text(20,30,time,'Fontsize',13)
%     hold off
%     
% end