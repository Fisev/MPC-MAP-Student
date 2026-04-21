function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE KF correction step using GNSS measurement (linear observation)    
    innovation = z - kf.C * mu;
    
    S = kf.C*sigma*kf.C'+kf.Q;
    
    K = sigma*kf.C'/S;
    
    new_mu = mu+K*innovation;
    new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));
    
    new_sigma = (eye(3)-K*kf.C)*sigma;
end