function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

if isempty(u) || any(isnan(u))
    new_mu = mu;
    new_sigma = sigma;
    return;
end

dt = sampling_period;
x = mu(1);
y = mu(2);
th = mu(3);

if kf.agent_drive.type == 2 % differential drive
    b = kf.agent_drive.interwheel_dist;
    vR = u(1);
    vL = u(2);
    v = (vR + vL) / 2;
    w = (vR - vL) / b;
    
    if abs(w) < 1e-6
        x_new = x + v * dt * cos(th);
        y_new = y + v * dt * sin(th);
        th_new = th;
        G = [1, 0, -v * dt * sin(th);
             0, 1,  v * dt * cos(th);
             0, 0,  1];
    else
        x_new = x + (v / w) * (sin(th + w * dt) - sin(th));
        y_new = y - (v / w) * (cos(th + w * dt) - cos(th));
        th_new = th + w * dt;
        G = [1, 0, (v / w) * (cos(th + w * dt) - cos(th));
             0, 1, (v / w) * (sin(th + w * dt) - sin(th));
             0, 0, 1];
    end
else % omnidirectional fallback
    x_new = x + u(1) * dt;
    y_new = y + u(2) * dt;
    th_new = th;
    if numel(u) >= 3
        th_new = th + u(3) * dt;
    end
    G = eye(3);
end

new_mu = [x_new; y_new; wrapToPi(th_new)];
new_sigma = G * sigma * G' + kf.R;

end
