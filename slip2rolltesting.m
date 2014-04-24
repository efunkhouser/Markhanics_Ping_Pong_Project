%COLLISION MODELING: SLIP2ROLL

%SLIP
% if vx > rw
%     f = mu*N
% elseif vx == rw
%     f = 0
%     
% end

t_impact = 3.6972 * 10^-4;

%normpdf(domain,mean,std) creates normal curve
normalcurve = normpdf([0:10^-6:t_impact],(t_impact/2),10^-4);
plot([0:10^-6:t_impact],normalcurve)