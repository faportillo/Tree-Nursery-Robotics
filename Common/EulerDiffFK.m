function [dx, dy, dTheta] = EulerDiffFK(WL, WR, Rw, l, theta, dt, Sl, Sr, skidD)
Wl = WL;
Wr = WR;
Vl = Wl * Rw * (1 - Sl);          %Left wheel forward ground speed
Vr = Wr * Rw * (1 - Sr);          %Right wheel forward ground speed
V = (Vl + Vr) / 2;                %Robot forward groud speed
Vy = V * tan(skidD);              %perpendicular skid speed
cosT = cos(theta);
sinT = sin(theta);

%change in position in world frame
dx = dt * (V * cosT - Vy * sinT);
dy = dt * (V * sinT + Vy * cosT);
omega = (Vl - Vr) / l;            %rotational velocity
dTheta = omega * dt;              %change in angle to world frame
end