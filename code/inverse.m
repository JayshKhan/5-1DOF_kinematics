clc; clear all; clf;
hold on;
a1 = 10.5; a2 =10; a3 = 16.5;
px = 18.5; py =-16.5 ; phiDeg = -90; phi= phiDeg*pi/180;

wx = px - (a3*cos(phi));wy = py - (a3*sin(phi));

up = wx^2 + wy^2 - a1^2 - a2^2;
down = 2*a1*a2;
full = up/down;
c2 = (wx*wx + wy*wy - a1*a1 - a2*a2)/(2*a1*a2);
% printing all the uper variables
fprintf('a1=%f, a2=%f, a3=%f\n',a1,a2,a3);
fprintf('px=%f, py=%f, phi=%f\n',px,py,phi);
fprintf('wx=%f, wy=%f\n',wx,wy);
fprintf('up=%f, down=%f, full=%f\n',up,down,full);
fprintf('c2=%f\n',c2);
if c2 <= 1
    s2_1 = sqrt(1-c2^2); s2_2 = -sqrt(1-c2^2);
    theta2_1 = atan2(s2_1,c2); theta2_2 = atan2(s2_2,c2);
    denom_1 = a1^2 + a2^2 + 2*a1*a2*cos(theta2_1);
    denom_2 = a1^2 + a2^2 + 2*a1*a2*cos(theta2_2);
    s1_1 = (wy*(a1+a2*cos(theta2_1)) - a2*sin(theta2_1)*wx)/denom_1;
    s1_2 = (wy*(a1+a2*cos(theta2_2)) - a2*sin(theta2_2)*wx)/denom_2;

    c1_1 = (wx*(a1+a2*cos(theta2_1)) + a2*sin(theta2_1)*wy)/denom_1;
    c1_2 = (wx*(a1+a2*cos(theta2_2)) + a2*sin(theta2_2)*wy)/denom_2;

    theta1_1 = atan2(s1_1,c1_1); %first solution
    theta1_2 = atan2(s1_2,c1_2); %second solution

    theta3_1 = phi - theta1_1 - theta2_1;
    theta3_2 = phi - theta1_2 - theta2_2;

    fprintf('theta1_1 = %f, theta2_1 = %f, theta3_1 = %f\n',theta1_1,theta2_1,theta3_1);
    fprintf('theta1_2 = %f, theta2_2 = %f, theta3_2 = %f\n',theta1_2,theta2_2,theta3_2);

    % visualizing the solution
    ax_1 =  a1*cos(theta1_1);
    ay_1 = a1*sin(theta1_1);
    ax_2 = a1*cos(theta1_2);
    ay_2 = a1*sin(theta1_2);

    bx = ax_1 + a2*cos(theta1_1+theta2_1);
    by = ay_1 + a2*sin(theta1_1+theta2_1);

    cx = px;cy=py;

    xAxisArrayXCord = [-2,2];
    xAxisArrayYCord = [0,0];

    yAxisArrayXCord = [0,0];
    yAxisArrayYCord = [-2,2];

    link1XCoords_1 = [0,ax_1];
    link1YCoords_1 = [0,ay_1];
    link1XCoords_2 = [0,ax_2];
    link1YCoords_2 = [0,ay_2];

    link2XCoords_1 = [ax_1,bx];
    link2YCoords_1 = [ay_1,by];
    link2XCoords_2 = [ax_2,bx];
    link2YCoords_2 = [ay_2,by];

    link3XCoords_1 = [bx,cx];
    link3YCoords_1 = [by,cy];

    plot(xAxisArrayXCord,xAxisArrayYCord,'k',yAxisArrayXCord,yAxisArrayYCord,'k',link1XCoords_1,link1YCoords_1,'r',link2XCoords_1,link2YCoords_1,'g',link3XCoords_1,link3YCoords_1,'b');
    hold on;
    plot(xAxisArrayXCord,xAxisArrayYCord,'k',yAxisArrayXCord,yAxisArrayYCord,'k',link1XCoords_2,link1YCoords_2,'r',link2XCoords_2,link2YCoords_2,'g',link3XCoords_1,link3YCoords_1,'b');


end