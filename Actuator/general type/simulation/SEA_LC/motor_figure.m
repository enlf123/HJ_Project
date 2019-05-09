function motor_figure(q1,q2)
q_motor = q1;
q_joint = q2;
r_motor = 5;
r_gear = 5;
r_inertia = 9;
d_m_g = 12;
d_g_i = 30;
deflection = q_motor - q_joint;
deflect = 10*(q_motor - q_joint);


for i =1:360
    motor(i,:) = [r_motor*cosd(i), r_motor*sind(i)];
end

for i =1:360
    gear(i,:) = [r_gear*cosd(i), r_gear*sind(i)];
end

for i =1:360
    inertia(i,:) = [r_inertia*cosd(i), r_inertia*sind(i)];
end

coor_bar = [0 0;0 1; r_motor 1;r_motor 0];
coor_inertia = [0 0;0 1; r_inertia 1;r_inertia 0];

coor_motor = rotation([coor_bar'], q_motor);
coor_gear = rotation([coor_bar'], q_motor);
coor_inertia = rotation([coor_inertia'], q_joint);
coor_motor = coor_motor';
coor_gear = coor_gear';
coor_inertia = coor_inertia';

string = [0, 0; 1+deflect/6, 2; 2+2*deflect/6,-2; 3+3*deflect/6 ,2; 4+4*deflect/6, -2; 5+5*deflect/6, 2; 6+deflect,0];


figure(1)
set(gcf, 'Position', [100, 250, 500, 500])
fill(motor(:,1), motor(:,2),'w');
hold on
fill(d_m_g+gear(:,1), gear(:,2),'w');
fill(d_m_g+d_g_i+inertia(:,1), inertia(:,2),'w');
fill(coor_motor(:,1),coor_motor(:,2),'k');
fill(d_m_g+coor_gear(:,1), coor_gear(:,2),'k');
fill(d_m_g+d_g_i+coor_inertia(:,1),coor_inertia(:,2),'k');
plot([d_m_g+r_gear,d_m_g+r_gear+6-deflect], [0,0],'k'); 
plot([d_m_g+r_gear+6-deflect+string(7,1),d_m_g+d_g_i-r_inertia], [0,0],'k'); 
plot(d_m_g+r_gear+6-deflect+string(:,1),string(:,2),'k');
text(-4,16,'Motor','Fontsize',13)
text(-6+d_m_g,16,'Gearbox','Fontsize',13)
text(-4+d_m_g+d_g_i,16,'Joint','Fontsize',13)
text(-28+d_m_g+d_g_i,-9,'Elastic element','Fontsize',13)
spring = strcat(sprintf('deflection : '),num2str(deflection));
text(-28+d_m_g+d_g_i,-16,spring,'Fontsize',13)
angle_motor = strcat(num2str(q_motor));
angle_gear = strcat(num2str(q_motor));
angle_joint = strcat(num2str(q_joint));
text(-3,18,angle_motor,'Fontsize',13)
text(-4+d_m_g,12,angle_gear,'Fontsize',13)
text(-2+d_m_g+d_g_i,12,angle_joint,'Fontsize',13)

axis equal
xlim([-10 60])
axis off

end

function R = rotation(P,angle)
R = [cos(angle) -sin(angle);sin(angle) cos(angle)]*P;
end