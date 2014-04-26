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
mu = .6; %coeff. of friction between ball and table
magnus_coeff = 0.000023;
% magnus_coeff = 2.6237e-08;

% theta = -pi/5; %launch angle in radians
% v0 = 20; %m/s

Times = 0:.01:7;
Initial = [0;0.1;25;-5;0]; %x0 y0 vx0 vy0 omega

options = odeset('Events',@events);
%B1 stands for Bounce 1
[T1, B1] = ode45(@proj_derivs,Times,Initial,options);


%COLLISION MODELING: FIRST BOUNCE
t_impact = 2 * 0.0014 / abs(B1(end,4)); %double compression distance (1.4 mm) / impact v
impact_velocity = B1(end,4);
BounceInitials = B1(end,:)';
BounceTimes = [0:10^-6:t_impact];

[Timp,Imp1] = ode45(@during_the_bounce,BounceTimes,BounceInitials);

    function bouncederivs = during_the_bounce(t,PV)
        x = PV(1);
        y = PV(2);
        vx = PV(3);
        vy = PV(4);
        omega = PV(5);
        
        dxdt = vx;
        dydt = vy;
        
        Inertia = (2/3)*m*(r_ball^2);
        
        F_impact = -2 * m * impact_velocity / t_impact; % 2mv / t = change in p over t
        
%         if abs(vx) > abs(r_ball*omega)
%             friction = -1*mu*F_impact;
%         elseif abs(vx) <= abs(r_ball*omega)
%             friction = 0;
%         end
        if vx + r_ball*omega > 0
            friction = -1*mu*F_impact;
        elseif vx + r_ball*omega < 0
            friction = mu*F_impact;
        else
            friction = 0;
        end
        
        Torque = y * friction; %r x F
        
        dvxdt = friction / m;
        dvydt = (F_impact / m) - g;
        domegadt = Torque / Inertia;
        
        bouncederivs = [dxdt;dydt;dvxdt;dvydt;domegadt];
    end

% plot(Timp,Imp1(:,2))
Initial2 = Imp1(end,:)';
% %after ode stops, next call:
[T2, B2] = ode45(@proj_derivs,Times,Initial2,options);
% 
% 
% Initial3 = [B2(end,1); B2(end,2); B2(end,3); -1*B2(end,4); Omega2];
% [T3,B3] = ode45(@proj_derivs,Times,Initial3,options);
% 
figure;
plot(B1(:,1),B1(:,2), 'LineWidth', 1.5)
hold on;
plot(Imp1(:,1), Imp1(:,2), 'LineWidth', 1.5)
plot(B2(:,1),B2(:,2), 'LineWidth', 1.5)
norm(B2(1,3:4))
B2(1,5)

% hold on;
% plot(B3(:,1),B3(:,2), 'LineWidth', 1.5)


% MAKE THE VX and R OMEGA GRAPH
% hold on
% plot(T1, B1(:,3), 'b')
% plot(T1, -1*B1(:,5)*r_ball, 'g')
% plot(T1(end)+Timp, Imp1(:,3), 'b')
% plot(T1(end)+Timp, -1*Imp1(:,5)*r_ball,'g')
% plot(T1(end)+Timp(end)+T2, B2(:,3), 'b')
% plot(T1(end)+Timp(end)+T2, -1*B2(:,5)*r_ball, 'g')



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
        
        Mx = -1*magnus_coeff * omega * vy;
        My = magnus_coeff * omega * vx;
        
        
        
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
% axis([0 6 0 1])
end