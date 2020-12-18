function dx = minripplecontroller(t,x)
global xd f1 f2 dist
if dist
    di=randn(2,1)*1e3;
else
    di=zeros(2,1);
end
sys1 = f1(x)+di;
sys2 = f2(x)+di;
m1 = ((x-xd)'*sys1)/norm(sys1);
m2 = ((x-xd)'*sys2)/norm(sys2);

if m1 < m2
    dx = sys1;
else
    dx = sys2;
end

end