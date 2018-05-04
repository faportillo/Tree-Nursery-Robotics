function [dx, dy, dTheta] = GeometricDiffFK(WL, WR, Rw, l, theta, dt)
  ep = 1e-8;
  Wl = WL;
  Wr = WR;
  Vl = Wl * Rw;
  Vr = Wr * Rw;
  V = (Vl + Vr) / 2;
  sinT = sin(theta);
  cosT = cos(theta);
  if (abs(Wl - Wr) < ep)
    dx = dt * V * cosT;
    dy = dt * V * sinT;
    dTheta = 0;
  else
    omega = (Vl - Vr) / l;
    R = l / 2 * ((Vl + Vr) / (Vr - Vl));
    xy = [cos(omega * dt), -sin(omega * dt);...
      sin(omega * dt), cos(omega * dt)] * [(R * sinT);...
      (-R * cosT)] + [(-R * sinT); (R * cosT)];
    dTheta = omega * dt;
    dx = xy(1);
    dy = xy(2);
  end
end