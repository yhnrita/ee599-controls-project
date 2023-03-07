function [xy, vX, vY] = simulate_trajectory(x_init, X, H, dt, noise_drift)
    xy(:,1) = x_init;
    for i=1:H
      vX(i) = X(H+1+i, 1);
      vY(i) = X(3*H+2+i, 1);
      xy(:,i+1) = xy(:,i) + dt*[vX(i); vY(i)] + noise_drift;
    end
end