function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, dt)

    interWheelDistance  = kf.interwheel_dist;
    vRight = u(1);
    vLelft = u(2);
    
    v = (vRight+vLelft)/2;
    w = (vRight-vLelft)/interWheelDistance;
    theta = mu(3);
    
    new_mu = mu+[v*cos(theta)*dt; v*sin(theta)*dt; w*dt];
    new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));
    
    J = [1, 0, -v*sin(theta)*dt;
         0, 1,  v*cos(theta)*dt;
         0, 0, 1];
    
    new_sigma = J*sigma*J'+kf.R;

    u
    v
    w
    theta
end