function res = ping_pong_it_1()
close all
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
S = 0.000207;

theta = -pi/10; %launch angle in radians
v0 = 10; %m/s

Times = 0:.01:1;
Initial = [0;0.2;(v0*cos(theta));(v0*sin(theta));0]; %x0 y0 vx0 vy0

options = odeset('Events',@events);
%FB stands for First Bounce
[T1, FB] = ode45(@proj_derivs,Times,Initial,options);
% Initial2 = [0.3739   -0.0000    1.5580    1.8161];
%after ode stops, next call:


%COLLISION MODELING: ADDING SPIN
% Torque * time of impact = Lf - L0
L_0 = Initial(2) * m * Initial(3); % h * m * vx
t_impact = 2 * -0.0014 / FB(end,4); %double compression distance (1.4 mm) / impact v
F_impact = -2 * m * FB(end,4) / t_impact; % 2mv / t = change in p over t
friction = mu * F_impact * -1 * sign(FB(end,3)); % mu * N * -vxhat
Torque = r_ball * friction; %r x F
rxp = FB(end,1)*m*-1*FB(end,4); %r x p = range * m * vy
Omega = (Torque * t_impact + L_0 - rxp) / ((2/3)*m*r_ball^2); %CALC R CROSS P

Initial2 = [FB(end,1);FB(end,2);FB(end,3);-1*FB(end,4);Omega];
[T2, SB] = ode45(@proj_derivs,Times,Initial2,options);
disp(SB(:,5))


plot(FB(:,1),FB(:,2))
hold on;
plot(SB(:,1),SB(:,2))
X = [0, 2.74];
Y = [0, 0];
X2 = [1.37, 1.37];
Y2 = [0, 0.1525];
plot (X,Y)
plot (X2, Y2)

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
        omega = PV(5);
        
        dxdt = vx;
        dydt = vy;
        
        Vhat = [vx;vy] ./ norm([vx;vy]);
        
        Fd = -0.5 * rho * A * Cd * (norm([vx;vy]))^2 .* Vhat;
        Fm_x = S*-1*omega*vx;
        Fm_y = S*omega*vy;
        dvxdt = Fd(1) / m + Fm_x/m;
        dvydt = -g + Fd(2) / m + Fm_y/m;
        derivs = [dxdt;dydt;dvxdt;dvydt; 0];
        %percent = t*100
    end

end