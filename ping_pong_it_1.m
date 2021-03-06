function res = ping_pong_it_1()
close all
%2-D model of a ping pong ball with initial angle theta hitting the table %and spinning off again %Only spin is about z-axis
%GOAL: working flight, impact, and post-impact flight model

m = .0027; %kg
r_ball = .020; %m
A = 0.00502654824; %m^2
Cd = 0.5;
rho = 1.225; %kg/m^3
g = 9.80; %m/s^2
mu = 0.6;


theta = -pi/10; %launch angle in radians
v0 = 10; %m/s

Times = 0:.01:1;
Initial = [0;0.2;(v0*cos(theta));(v0*sin(theta))]; %x0 y0 vx0 vy0

options = odeset('Events',@events);
%FB stands for First Bounce
[T1, FB] = ode45(@proj_derivs,Times,Initial,options);
Initial2 = [FB(end,1) FB(end,2) FB(end,3) -1*FB(end,4)];
% Initial2 = [0.3739   -0.0000    1.5580    1.8161];
%after ode stops, next call:
[T2, SB] = ode45(@proj_derivs,Times,Initial2,options);
plot(FB(:,1),FB(:,2))
hold on;
plot(SB(:,1),SB(:,2))
X = [0, 2.74];
Y = [0, 0];
X2 = [1.37, 1.37];
Y2 = [0, 0.1525];
plot (X,Y)
plot (X2, Y2)

%FRICTION HAPPENING @ EL IMPACT
% Torque = (Lf - L0) / t_impact
% omega = T*t + L0 - r x p / I
%L_0 = Initial(2) * m * Initial(3); %initial height times mass times vx

    function [value,isterminal,direction] = events(t,PV)
        value = PV(2);
        isterminal = 1;
        direction = -1;
    end

    function derivs = proj_derivs(t,PV)
        x = PV(1);
        y = PV(2);
        vx = PV(3);
        vy = PV(4);
        
        dxdt = vx;
        dydt = vy;
        
        Vhat = [vx;vy] ./ norm([vx;vy]);
        
        Fd = -0.5 * rho * A * Cd * (norm([vx;vy]))^2 .* Vhat;
        
        dvxdt = Fd(1) / m;
        dvydt = -g + Fd(2) / m;
        derivs = [dxdt;dydt;dvxdt;dvydt];
        %percent = t*100
    end

end
