function mode = minripplemode(x)
global xd f1 f2

sys1 = f1(x);
sys2 = f2(x);
m1 = ((x-xd)'*sys1)/norm(sys1);
m2 = ((x-xd)'*sys2)/norm(sys2);

if m1 < m2
    mode = 1;
else
    mode = 2;
end

end