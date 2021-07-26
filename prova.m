Ts=0.1;
T=5;
t = 0:Ts:T;
%% LINEAR TRAJECTORY

% tau_t = t/T;
% disp(tau_t)
% s = -2*tau_t.^3 + 3*tau_t.^2;
% ds = (6*t)/T^2 - (6*t.^2)/T^3;
% dds = 6/T^2 - (12*t)/T^3;
% start = [1.3660; 2.3660];
% finish = [2.5; 0];
% r = start + s.*(finish-start);
% dr = (finish-start)*ds;
% ddr = (finish-start)*dds;


%% CIRCULAR TRAJECTORY

h = 0;
if t > T
    h = 1
end
tau_t = t/T - h;
center = [1.5; 1];
radius = 0.5;
s = -2*tau_t.^3 + 3*tau_t.^2;
ds = (6*t)/T^2 - (6*t.^2)/T^3;
dds = 6/T^2 - (12*t)/T^3;
r = center + radius * [cos(s* 2*pi) ; sin(s * 2 * pi)];
dr = 2*pi * radius .* [-sin(s.*2.*pi); cos(2.*s.*pi)] .* ds;
ddr = (2 .*pi)^2 .* radius .* [-cos(s.*2.*pi); -sin(2.*s.*pi)].* dds;

figure
plot(t,ddr);
grid on;