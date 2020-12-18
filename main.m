clc
clear
close all

global mode f1 f2 xd d dist f

%% Params
boost = 0 % Boost/Buck converter
mode = 1; %Initial mode for switching controller
dist = 1; %Disturbance enable

if boost
    R=6;
    C=110e-6;
    L=150e-6;
    Vin = 1.5;
    A1=[0,0;0,-1/(R*C)];
    A2=[0,-1/L;1/C,-1/(R*C)];
    B1 = [Vin/L;0];
    B2 = B1;
    xd = [1.21;3.3];
    dc = 0.33;
    ds = 0.18;
    x0 = [1.3;3.3]%[1.4 ; 3.4];
else
    L = 220e-6;
    C = 100e-6;
    R = 10;
    Vin = 24;
    A1=[0,-1/L;1/C,-1/(R*C)];
    A2=[0,-1/L;1/C,-1/(R*C)];
    B1 = [Vin/L;0];
    B2 = [0;0];
    xd = [1.2;12];
    dc = 0.6;
    ds = 0.3; 
    x0 = [1.15;12.05];%[1; 12.4];
end

f1 = @(x)(A1*x+B1);
f2 = @(x)(A2*x+B2);

dt=10e-8;

%% Simulation - continuous

% [t1_, x1_] = ode45(@minripplecontroller,[0:dt:T], x0);
% [t2_, x2_] = ode45(@minswitchcontroller,[0:dt:T], x0);

% dtswitch=10e-8;
dtripple=10e-7;
T=0.01;
x1 = x0;
x1_= x0;
x2 = x0;
x2_= x0;
d=dc;
% 
% [t1_, x1_] = ode45(@minripplecontroller,[0:dtripple:T], x0);
% [t2_, x2_] = ode45(@minswitchcontroller,[0:dt:T], x0);
if mode == 1
    f=f1;
else
    f=f2;
end

% min controller switching time (dtripple) required to prevent Zeno executions.
for i=0:dtripple:T-dtripple
    i
    [t1, x1a] = ode45(@evolution,[0:dt:dtripple-dt], x1); % evolve to dtripple.
    x1=x1a(end,:)'; % take last point.
    x1_=[x1_, x1a']; %save trajectory
    m = minripplemode(x1); % check mode
    if m == 1 %change mode
        f=f1;
    else
        f=f2;
    end
end
x1_=x1_';

for i=0:dt:T-dt
    i
    [t2, x2a] = ode45(@minswitchcontroller,[0,dt], x2);
    x2=x2a(end,:)';
    x2_=[x2_, x2];
end

x2_=x2_';
t_=[0:dt:T]';

%% Simulation - sampled

dtsample=10e-6;
T=0.01;
x1 = x0;
x1_= x0;
x2 = x0;
x2_= x0;
d=ds;
%Min Ripple
if mode == 1
    f=f1;
else
    f=f2;
end

for i=0:dtsample:T-dtsample
    i
    [t1, x1a] = ode45(@evolution,[0:dt:dtsample-dt], x1); % evolve to dtsample.
    x1=x1a(end,:)'; % take last point.
    x1_=[x1_, x1a']; %save trajectory
    m = minripplemode(x1); % check mode
    if m == 1 %change mode
        f=f1;
    else
        f=f2;
    end
end
x1_=x1_';
%Min Switching
for i=0:dtsample:T-dtsample
    i
    [t2, x2a] = ode45(@evolution,[0:dt:dtsample-dt], x2); % evolve to dtsample.
    x2 = x2a(end,:)'; % take last point.
    x2_=[x2_, x2a']; %save trajectory
    m = minswitchmode(x2a,m); % check mode
    if m == 1 %change mode
        f=f1;
    else
        f=f2;
    end
end
x2_=x2_';
%% Presentation

theta = 0:0.01:2*pi;
xc=xd(1) + dc*cos(theta);
yc=xd(2) + dc*sin(theta);

    %buck converter value here
xs=xd(1) + ds*cos(theta);
ys=xd(2) + ds*sin(theta);

figure(1)
plot(x1_(:,1), x1_(:,2))
hold on 
plot(xc, yc, '--k')
plot(xs, ys, '--k')
plot(x0(1),x0(2),'o')
title('Min Ripple controller')
xlabel('$$i_L$$', 'interpreter','latex')
ylabel('$$V_o$$', 'interpreter','latex')

figure(2)
plot(x2_(:,1), x2_(:,2))
hold on 
plot(xc, yc, '--k')
plot(xs, ys, '--k')
plot(x0(1),x0(2),'o')
title('Min Switching controller')
xlabel('$$i_L$$', 'interpreter','latex')
ylabel('$$V_o$$', 'interpreter','latex')

% figure(3)
% plot(xb, yb, '--k')
% hold on
% h = animatedline;
% 
% for k = 1:length(t_)
%     addpoints(h,x2_(k,1),x2_(k,2));
%     pause(0.1)
% end

