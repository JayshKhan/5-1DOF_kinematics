clc; clear all; clf;
hold on;
a1 = 10.5; a2 =10; a3 = 16.5;
px = 0; px = 37; pz =0 ; phiDeg = -95; phi= phiDeg*pi/180;
% Define a range of phi values to explore
phiDegValues = -180:10:180; % Example: Explore from -180 to 180 degrees with 10-degree increments

theta0 = atan2(py,px) % turn Table Angle
px_rotated = px*cos(theta0)+py*cos(theta0)
px = px_rotated

for phiDeg = phiDegValues
    phi = phiDeg * pi / 180;
    
    wx = px - (a3*cos(phi));wz = pz - (a3*sin(phi));
    c2 = (wx*wx + wz*wz - a1*a1 - a2*a2)/(2*a1*a2);

    if c2 <= 1
        s2_1 = sqrt(1-c2^2); s2_2 = -sqrt(1-c2^2);
        theta2_1 = atan2(s2_1,c2); theta2_2 = atan2(s2_2,c2);
        denom_1 = a1^2 + a2^2 + 2*a1*a2*cos(theta2_1);
        denom_2 = a1^2 + a2^2 + 2*a1*a2*cos(theta2_2);
        s1_1 = (wz*(a1+a2*cos(theta2_1)) - a2*sin(theta2_1)*wx)/denom_1;
        s1_2 = (wz*(a1+a2*cos(theta2_2)) - a2*sin(theta2_2)*wx)/denom_2;
    
        c1_1 = (wx*(a1+a2*cos(theta2_1)) + a2*sin(theta2_1)*wz)/denom_1;
        c1_2 = (wx*(a1+a2*cos(theta2_2)) + a2*sin(theta2_2)*wz)/denom_2;
    
        theta1_1 = atan2(s1_1,c1_1); %first solution
        theta1_2 = atan2(s1_2,c1_2); %second solution

        theta0 = atan2(py,px);
    
        theta3_1 = phi - theta1_1 - theta2_1;
        theta3_2 = phi - theta1_2 - theta2_2;
        theta3_1 = theta3_1+pi;
        theta3_2 = theta3_2+pi;

    
        fprintf('theta1_1 = %f, theta2_1 = %f, theta3_1 = %f\n',rad2deg(theta1),rad2deg(theta2),rad2deg(theta3));
        fprintf('theta1_2 = %f, theta2_2 = %f, theta3_2 = %f\n',rad2deg(theta1_2),rad2deg(theta2_2),rad2deg(theta3_2));
   
   
        % visualizing the solution
        ax_1 =  a1*cos(theta1_1);
        az_1 = a1*sin(theta1_1)+11.5;
        ax_2 = a1*cos(theta1_2);
        az_2 = a1*sin(theta1_2)+11.5;
    
        bx = ax_1 + a2*cos(theta1_1+theta2_1);
        bz = az_1 + a2*sin(theta1_1+theta2_1);
    
        cx = px;cz=pz;
    
        xAxisArrayXCord = [-2,2];
        xAxisArrayYCord = [0,0];
    
        zAxisArrayXCord = [0,0];
        zAxisArrayYCord = [-2,2];
    
        link1XCoords_1 = [0,ax_1];
        link1ZCoords_1 = [0,az_1];
        link1XCoords_2 = [0,ax_2];
        link1ZCoords_2 = [0,az_2];
    
        link2XCoords_1 = [ax_1,bx];
        link2ZCoords_1 = [az_1,bz];
        link2XCoords_2 = [ax_2,bx];
        link2ZCoords_2 = [az_2,bz];
    
        link3XCoords_1 = [bx,cx];
        link3ZCoords_1 = [bz,cz];
    
        plot(xAxisArrayXCord,xAxisArrayYCord,'k',zAxisArrayXCord,zAxisArrayYCord,'k',link1XCoords_1,link1ZCoords_1,'y',link2XCoords_1,link2ZCoords_1,'m',link3XCoords_1,link3ZCoords_1,'black');
       hold on;
        plot(xAxisArrayXCord,xAxisArrayYCord,'k',zAxisArrayXCord,zAxisArrayYCord,'k',link1XCoords_2,link1ZCoords_2,'r',link2XCoords_2,link2ZCoords_2,'g',link3XCoords_1,link3ZCoords_1,'b');
    
    
    end
end