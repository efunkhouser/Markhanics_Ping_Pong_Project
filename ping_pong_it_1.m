function res = ping_pong_it_1()
%2-D model of a ping pong ball with initial angle theta hitting the table
%and spinning off again
%Only spin is about z-axis
%GOAL: working flight, impact, and post-impact flight model

m = .0027; %kg
g = 9.80; %m/s^2
A = 0.00502654824; %m^2
Cd = 0.5;
rho = 1.225; %kg/m^3

theta = pi/4; %launch angle in radians
v0 = 10; %m/s

Times = [0:0.01:1];
Initial = [0;0.1;(v0*cos(theta));(v0*sin(theta))]; %x0 y0 vx0 vy0

[T, PosVel] = ode45(@proj_derivs,Times,Initial);
plot(T, PosVel(:,1), 'b' ,'LineWidth', 1.5)
hold on
plot(T, PosVel(:,2), 'g', 'LineWidth', 1.5)

    function derivs = proj_derivs(t,PV)
        x = PV(1);
        y = PV(2);
        vx = PV(3);
        vy = PV(4);
        
        dxdt = vx;
        dydt = vy;
        
        Vhat = [vx;vy] ./ norm([vx;vy]);
        
        Fd = 0.5 * rho * A * Cd * (norm([vx;vy]))^2 * -1 .* Vhat;
        
        dvxdt = Fd(1) / m;
        dvydt = -g / m - Fd(2) / m;
        
        derivs = [dxdt;dydt;dvxdt;dvydt];
    end



end
