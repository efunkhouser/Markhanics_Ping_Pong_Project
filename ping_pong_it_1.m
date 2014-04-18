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
magnus_coeff = .000207;

theta = -pi/3.5; %launch angle in radians
v0 = 8; %m/s

Times = 0:.01:1;
Initial = [0;0.2;(v0*cos(theta));(v0*sin(theta));0]; %x0 y0 vx0 vy0

options = odeset('Events',@events);
%B1 stands for Bounce 1
[T1, B1] = ode45(@proj_derivs,Times,Initial,options);


%COLLISION MODELING: FIRST BOUNCE
% Torque * time of impact = Lf - L0
L_0 = Initial(2) * m * Initial(3); % h * m * vx
t_impact = 2 * -0.0014 / B1(end,4); %double compression distance (1.4 mm) / impact v
F_impact = -2 * m * B1(end,4) / t_impact; % 2mv / t = change in p over t
friction = mu * F_impact * -1 * sign(B1(end,3)); % mu * N * -vxhat
Torque = r_ball * friction; %r x F
rxp = B1(end,1)*m*-1*B1(end,4); %r x p = range * m * vy
Omega = (Torque * t_impact + L_0 - rxp) / ((2/3)*m*r_ball^2); %CALC R CROSS P

Initial2 = [B1(end,1); B1(end,2); B1(end,3); -1*B1(end,4); Omega];
%after ode stops, next call:
[T2, B2] = ode45(@proj_derivs,Times,Initial2,options);

figure;
plot(B1(:,1),B1(:,2))
hold on;
plot(B2(:,1),B2(:,2))





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
        
        Mx = magnus_coeff * omega * vx;
        if vy > 0
            My = -1 * magnus_coeff * omega * vy;
        else
            My = magnus_coeff * omega * vy;
        end
        
        dvxdt = (Fd(1) + Mx) / m;
        dvydt = -g + (Fd(2) + My) / m;
        derivs = [dxdt;dydt;dvxdt;dvydt;(0.03*omega/100)];
        %percent = t*100
    end
%THE TABLE
X = [0, 2.74];
Y = [0, 0];
X2 = [1.37, 1.37];
Y2 = [0, 0.1525];
plot (X,Y,'k','linewidth',2)
plot (X2, Y2,'k','linewidth',2)
%axis([0 6 0 1])
end