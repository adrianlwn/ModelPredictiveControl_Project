%% Test Param inf : 

x_phi = @(phi) sin(phi)./(1+cos(phi).^2)
y_phi = @(phi) sin(phi).*cos(phi)./(1+cos(phi).^2)

PH = 0:0.1:2*pi;

plot(x_phi(PH),y_phi(PH),'-o')