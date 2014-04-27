function res = ping_pong_it_3()

for i = 197:1000
    serve(i);
    if F2(end, 1) < 2.74
        V(i) = F3(end,1)-B2(end,1);
    else
        V(i) = 0;
    end
end

i = 197:1000;
plot(i, V, 'b', 'LineWidth', 1.5)

serve(600)

function res = serve(init_omega)
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
magnus_coeff = 0;
%magnus_coeff = 0.0000207;

theta = -0.1974; %launch angle in radians
v0 = 25.4951; %m/s

F1_Times = 0:.01:7;
%F1_Initial = [0;0.1;v0*cos(theta); v0*sin(theta);500];
F1_Initial = [0;0.1;9; -5; init_omega];
options = odeset('Events',@events);

%F1 stands for flight 1, ball is in flight first
[T_F1, F1] = ode45(@flight_derivs,F1_Times,F1_Initial, options);

    function derivs = flight_derivs(t,PV)
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

%COLLISION MODELING: FIRST BOUNCE

t_impact = 2 * 0.0014 / abs(F1(end,4)); %double compression distance (1.4 mm) / impact v
impact_velocity = F1(end,4);
B1_Initial = F1(end,:)';
B1_Times = [0:10^-6:t_impact];

[T_B1, B1] = ode45(@bounce_derivs,B1_Times, B1_Initial);

    function bouncederivs = bounce_derivs(t,PV)
        x = PV(1);
        y = PV(2);
        vx = PV(3);
        vy = PV(4);
        omega = PV(5);
        
        dxdt = vx;
        dydt = vy;
        
        Inertia = (2/3)*m*(r_ball^2);
        
        F_impact = -2 * m * impact_velocity / t_impact; % 2mv / t = change in p over t
        
        if vx + r_ball*omega > 0
            friction = -1*mu*F_impact;
            %disp('Boyz 2 Men')
        elseif vx + r_ball*omega < 0
            friction = mu*F_impact;
            %disp('Boyz 2 Men')
        else
            friction = 0;
            disp('Slip 2 Roll')
        end
        
        Torque = y * friction; %r x F
        
        dvxdt = friction / m;
        dvydt = (F_impact / m) - g;
        domegadt = Torque / Inertia;
        
        bouncederivs = [dxdt;dydt;dvxdt;dvydt;domegadt];
    end

%second flight phase
F2_Initial = B1(end,:)';
[T_F2, F2] = ode45(@flight_derivs,F1_Times,F2_Initial,options);

%second bounce phase
t_impact = 2 * 0.0014 / abs(F2(end,4)); %double compression distance (1.4 mm) / impact v
impact_velocity = F2(end,4);
B2_Initial = F2(end,:)';
B2_Times = [0:10^-6:t_impact];

[T_B2, B2] = ode45(@bounce_derivs, B2_Times, B2_Initial);
%third and final flight phase
F3_Initial = B2(end,:)';
[T_F3, F3] = ode45(@flight_derivs, F1_Times, F3_Initial, options);

% figure;
% hold on
% plot(F1(:,1),F1(:,2), 'LineWidth', 1.5)
% plot(B1(:,1), B1(:,2), 'LineWidth', 1.5)
% plot(F2(:,1),F2(:,2), 'LineWidth', 1.5)
% plot(B2(:,1), B2(:,2), 'LineWidth', 1.5)
% plot(F3(:,1),F3(:,2), 'LineWidth', 1.5)

% hold on;
% plot(B3(:,1),B3(:,2), 'LineWidth', 1.5)


% MAKE THE VX and R OMEGA GRAPH
% figure;
% hold on
% plot(T_F1, F1(:,3), 'b')
% plot(T_F1, -1*F1(:,5)*r_ball, 'g')
% plot(T_F1(end)+ T_B1, B1(:,3), 'b')
% plot(T_F1(end) + T_B1, -1*B1(:,5)*r_ball, 'g')
% plot(T_F1(end)+T_B1(end) + T_F2, F2(:,3), 'b')
% plot(T_F1(end)+T_B1(end) + T_F2, -1*F2(:,5)*r_ball, 'g')
% plot(T_F1(end)+T_B1(end) + T_F2(end) + T_B2, B2(:,3), 'b')
% plot(T_F1(end)+T_B1(end) + T_F2(end) + T_B2, -1*B2(:,5)*r_ball, 'g')
% plot(T_F1(end)+T_B1(end) + T_F2(end) + T_B2(end)+T_F3, F3(:,3), 'b')
% plot(T_F1(end)+T_B1(end) + T_F2(end) + T_B2(end)+T_F3, -1*F3(:,5)*r_ball, 'g')




% res = B2(end,2);

    function [value,isterminal,direction] = events(t,PV)
        value = PV(2)-r_ball;
        isterminal = 1;
        direction = -1;
    end


%THE TABLE
X = [0, 2.74];
Y = [0, 0];
X2 = [1.37, 1.37];
Y2 = [0, 0.1525];
% plot (X,Y,'k','linewidth',2)
% plot (X2, Y2,'k','linewidth',2)
%axis([0 6 0 1])

end
end