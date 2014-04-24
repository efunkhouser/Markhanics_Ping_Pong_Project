function res = ping_pong_it_3()
%2-D model of a ping pong ball with initial angle theta hitting the table
%and spinning off again
%Only spin is 2-D
%GOAL: implement MAGNUS FORCE

m = .0027; %kg
r_ball = 0.020; %m
g = 9.81; %m/s^2
A = pi*r_ball^2; %m^2 %surface area not area
Cd = 0.5;
rho = 1.225; %kg/m^3
mu = 0.6; %coeff. of friction between ball and table
magnus_coeff = .000207;

theta = -pi/9; %launch angle in radians
v0 = 8; %m/s

Times = 0:.01:5;
Initial = [0;0.1;(v0*cos(theta));(v0*sin(theta));0]; %x0 y0 vx0 vy0 omega

options = odeset('Events',@events);
%B1 stands for Bounce 1
[T1, B1] = ode45(@proj_derivs,Times,Initial,options);


%COLLISION MODELING: FIRST BOUNCE
t_impact = 2 * 0.0014 / abs(B1(end,4)); %double compression distance (1.4 mm) / impact v

BounceInitials = B1(end,:)';
BounceTimes = [0:10^-6:t_impact];

[Timp,Impact1] = ode45(@during_the_bounce,BounceTimes,BounceInitials);

    function bouncederivs = during_the_bounce(t,PV)
        x = PV(1);
        y = PV(2);
        vx = PV(3);
        vy = PV(4);
        omega = PV(5);
        
        dxdt = vx;
        dydt = vy;
        
        Inertia = (2/3)*m*(r_ball^2);
        
        F_impact = -2 * m * vy / t_impact; % 2mv / t = change in p over t
        
        if abs(vx) > abs(r_ball*omega)
            friction = mu*F_impact;
        elseif abs(vx) <= abs(r_ball*omega)
            friction = 0;
        end
        
        Torque = r_ball * friction; %r x F
        
        dvxdt = friction / m;
        dvydt = (F_impact / m) - g;
        domegadt = Torque / Inertia;
        
        bouncederivs = [dxdt;dydt;dvxdt;dvydt;domegadt];
    end

plot(Timp,Impact1(:,2))
% Initial2 = [B1(end,1); B1(end,2); B1(end,3); -1*B1(end,4); Omega1];
% %after ode stops, next call:
% [T2, B2] = ode45(@proj_derivs,Times,Initial2,options);
% 
% 
% Initial3 = [B2(end,1); B2(end,2); B2(end,3); -1*B2(end,4); Omega2];
% [T3,B3] = ode45(@proj_derivs,Times,Initial3,options);
% 
% figure;
% plot(B1(:,1),B1(:,2), 'LineWidth', 1.5)
% hold on;
% plot(B2(:,1),B2(:,2), 'LineWidth', 1.5)
% hold on;
% plot(B3(:,1),B3(:,2), 'LineWidth', 1.5)


% res = B2(end,2);

    function [value,isterminal,direction] = events(t,PV)
        value = PV(2)-r_ball;
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
        
        Mx = magnus_coeff * omega * vy;
        My = magnus_coeff * omega * vx;
        
        
        
        dvxdt = (Fd(1) + Mx) / m;
        dvydt = -g + (Fd(2) + My) / m;
        derivs = [dxdt;dydt;dvxdt;dvydt;(0.03*omega/100)];
        %percent = t*100
    end
%THE TABLE
% X = [0, 2.74];
% Y = [0, 0];
% X2 = [1.37, 1.37];
% Y2 = [0, 0.1525];
% plot (X,Y,'k','linewidth',2)
% plot (X2, Y2,'k','linewidth',2)
%axis([0 6 0 1])
end