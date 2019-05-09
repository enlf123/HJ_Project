function save = f_pid(X)
kp = X(1);
ki = X(2);
kd = X(3);
desire = X(4);
value = X(5);
dt = X(6);
err = zeros(1,3);
err(1) = desire - value;
err(2) = X(7);
err(3) = X(8);
pre_pid = X(9);
G1 = kp + ki * dt + kd/ dt;
G2 = -(kp + 2* kd / dt);
G3 = kd / dt;

pid = pre_pid+ G1*err(1) + G2*err(2) + G3*err(3);

save(1) = err(1);
save(2) = err(2);
save(3) = pid;
end

