%MACH Sensor Wing sizing code
%Keep Span constant
%Vary theta
%Vary chord
%All dimensions in inches.
in2m = 0.0254;
b = 8*in2m;
steps = 1000;
thetaSteps = 500;
sensorMassSteps = 500;
thetaRange = [0, 45];
theta = linspace(thetaRange(1), thetaRange(2), thetaSteps);
sensorMassRange = [0.1, 1]; %kg
sensorMass = linspace(sensorMassRange(1), sensorMassRange(2), sensorMassSteps);
CL_Cruise = 0.5;

wing_ref_area = zeros(thetaSteps, sensorMassSteps);
for i = 1:thetaSteps
    for j = 1:sensorMassSteps
        wing_ref_area(i,j) = b*1.5*in2m;
    end
end
AR = b^2./wing_ref_area;
e = 0.8;
v = 29;
CD_0 = 0.06;
rho = 1.225;

for i = 1:length(theta)
    for j = 1:length(sensorMass)
        for iter = 1:steps
            
            
            %Calculate guess for lift based on current wing area
            L = 1/2*rho*v^2*wing_ref_area(i,j)*CL_Cruise;
            
            %Calculate drag from craft
            D = 1/2*rho*v^2*wing_ref_area(i,j)*CD_0 + 2*L^2/(rho*v^2*wing_ref_area(i,j)*pi*AR(i,j)*e);
            
            %Tcos(theta) = D
            %Tsin(theta) + Lreq = mg
            
            T = D/cosd(theta(i));
            Lreq = sensorMass(j)*9.81 - T*sind(theta(i));
            
            if Lreq < 0
                wing_ref_area(i,j) = 0;
                break
            end
            
            wing_ref_area_req = 2*Lreq/(rho*v^2*CL_Cruise);
            
            wing_ref_area_err = wing_ref_area_req - wing_ref_area(i,j);
            
            if (abs(wing_ref_area_err) <= 1e-8)
                fprintf("Converged after %d iterations \n", iter);
                break
            end
            
            wing_ref_area(i,j) = wing_ref_area(i,j) + 0.1*wing_ref_area_err;
            AR(i,j) = b^2/wing_ref_area(i,j);
        end
    end
end

[X, Y] = meshgrid(sensorMass, theta);
contourf(X, Y, wing_ref_area);
colorbar;
xlabel('Sensor Mass (kg)');
ylabel('Tether Angle (degrees)');