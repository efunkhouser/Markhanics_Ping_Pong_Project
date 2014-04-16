function res = ping_pong_it_1()
%2-D model of a ping pong ball with initial angle theta hitting the table
%and spinning off again
%Only spin is about z-axis
%GOAL: working flight, impact, and post-impact flight model

m = .0027; %kg
r_ball = 0.020; %m
g = 9.80; %m/s^2
A = 0.00502654824; %m^2
Cd = 0.5;
rho = 1.225; %kg/m^3
mu = 0.6; %coeff. of friction between ball and table

theta = 0; %launch angle in radians
v0 = 2; %m/s

Times = 0:.01:1;
Initial = [0;0.2;(v0*cos(theta));(v0*sin(theta))]; %x0 y0 vx0 vy0

options = odeset('Events',@events);
%FB stands for First Bounce
[T1, FB] = ode45(@proj_derivs,Times,Initial,options);
Initial2 = [FB(end,1) FB(end,2) FB(end,3) -1*FB(end,4)];
% Initial2 = [0.3739   -0.0000    1.5580    1.8161];
%after ode stops, next call:
[T2, SB] = ode45(@proj_derivs,Times,Initial2,options);
% plot(FB(:,1),FB(:,2))
% hold on;
%plot(SB(:,1),SB(:,2))

%COLLISION MODELING: ADDING SPIN
% Torque * time of impact = Lf - L0
L_0 = Initial(2) * m * Initial(3); % h * m * vx
t_impact = 2 * -0.0014 / FB(end,4); %double compression distance (1.4 mm) / impact v
F_impact = -2 * m * FB(end,4) / t_impact; % 2mv / t = change in p over t
friction = mu * F_impact * -1 * sign(FB(end,3)); % mu * N * -vxhat
Torque = r_ball * friction; %r x F
rxp = FB(end,1)*m*-1*FB(end,4); %r x p = range * m * vy
Omega = (Torque * t_impact + L_0 - rxp) / ((2/3)*m*r_ball^2) %CALC R CROSS P



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