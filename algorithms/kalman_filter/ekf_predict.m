function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, dt)

    interwheel = kf.interwheel_dist;
    vR = u(1);
    vL= u(2);
    v = (vR + vL) / 2;
    w = (vR - vL) / interwheel;

    theta = mu(3);

    if abs(w) < 1e-6
        new_mu = mu + [v*cos(theta)*dt;
                       v*sin(theta)*dt;
                       0];
        dx_dtheta = -v*sin(theta)*dt;
        dy_dtheta = v*cos(theta)*dt;
    else
        new_mu = [mu(1) + (v/w)*(sin(theta + w*dt) - sin(theta));
                  mu(2) - (v/w)*(cos(theta + w*dt) - cos(theta));
                  theta + w*dt];
        dx_dtheta = (v/w)*(cos(theta + w*dt) - cos(theta));
        dy_dtheta = (v/w)*(sin(theta + w*dt) - sin(theta));
    end

    new_mu(3)= atan2(sin(new_mu(3)), cos(new_mu(3)));

    J = [1, 0, dx_dtheta;
         0, 1, dy_dtheta;
         0, 0, 1];

    new_sigma = J*sigma*J' + kf.R;
end
