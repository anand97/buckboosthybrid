d=0;
ld=0;
dmax=100;
x1range=[0,2.5];
x2range=[2.97,3.63];

ddelta=0.01
dtheta=0.01
global R Vin
%% Params
boost = 0 % Boost/Buck converter

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
    dcmin = 0.06;
    %ds = 0.22;
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
    dcmin = 0.03;
    % ds = 
    x0 = [1.15;12.05];%[1; 12.4];
end

f1 = @(x)(A1*x+B1);
f2 = @(x)(A2*x+B2);

inprod1=@(d,t)(d*(cos(t)*Vin/L - sin(t)*xd(2)/(R*C))-((d*sin(t))^2)*((R*C)^-1));
inprod2=@(d,t)(d*(cos(t)*(Vin/L - xd(2)/L)+sin(t)*(xd(1)/C-xd(2)/(R*C)))+((d^2)*(sin(t)*cos(t)*((1/C)-(1/L))-((sin(t))^2)*((R*C)^-1))));

best_d=0;

for i = 0:ddelta:dc
    for j=0:dtheta:2*pi
        val1 = inprod1(i,j);
        val2 = inprod2(i,j);
        mat(round(i/ddelta+1),round(j/dtheta+1))=(val1<0 && val2<0);
        if (val1<0 && val2<0)
            
            if best_d<i
                best_d=i;
            end
        end
    end
end

dsample= 10e-6;


inprod1=@(d,t)((d*cos(t)-xd(1))*d2(t)/L - (d*(d*sin(t)-xd(2))*d1(t)/C));
inprod2=@(d,t)((d*cos(t)-xd(1))*d2(t)/L - (d*sin(t)*(d*sin(t)-xd(2))*d1(t)/C) - (d*sin(t)*(d*cos(t)-xd(1))/L) + (d*cos(t)*(d*sin(t)-xd(2))/C));
best_dsamp=0;

%Get ball radius
for i = dcmin:ddelta:dc
    for j=0:dtheta:2*pi
        val1 = inprod1(i,j);
        val2 = inprod2(i,j);

        mat(round(i/ddelta+1),round(j/dtheta+1))=(val1<0 && val2<0);
        if (val1<0 && val2<0)
%             x = [i*cos(j);i*sin(j)] ;
%             val1 = i+dsample*(norm(f1(x)));
%             val2 = i+dsample*(norm(f2(x)));
%             maximum = max([val1 val2]);
            if best_dsamp<i %&& maximum<i
                best_dsamp=i;    
                
            end
        end
    end
end

%Get smaller ball radius
best_ds = dcmin;
for i = dcmin:ddelta:dc
    for j=0:dtheta:2*pi
        x = [i*cos(j);i*sin(j)] ;
        val1 = i+dsample*(norm(f1(x)));
        val2 = i+dsample*(norm(f2(x)));
        maximum = max([val1 val2]);
        if best_ds<i && maximum<best_dsamp
            best_ds=i;    

        end
    end
end

function d = d1(t)
global R
dmax=1/(R);
dmin=1/(R*1.05);
if t>=pi/2 && t<3*pi/2
    d=dmin;
else
    d=dmax;
end
end

function d = d2(t)
global Vin
dmax=Vin;
dmin=Vin*0.95;
if t>=0 && t<pi
    d=dmax;
else
    d=dmin;
end

end
