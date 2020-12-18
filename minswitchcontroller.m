function dx = minswitchcontroller(t,x)
global mode f1 f2 xd d dist

if dist
    di=randn(2,1)*1e3;
else
    di=zeros(2,1);
end

if norm(x-xd)>d
    if mode == 1   
        mode = 2;
    else
        mode = 1;
    end
end

if mode== 1
    dx = f1(x)+di;
else
    dx = f2(x)+di;
end



% 
% if norm(x-xd)>=d
%     if mode(end) == 1   
%         mode = [mode 2];
%     else
%         mode = [mode 1];
%     end
% else
%     mode = [mode mode(end)];
% end
% 
% if mode(end) == 1
%     dx = f1(x);
% else
%     dx = f2(x);
% end



end