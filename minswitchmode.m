function mode = minswitchmode(x, mode)
global xd d

%To check whether the trajectory enters the inner ball before switching again.
if sum(sqrt(sum((x'-xd).^2))<=d)>0

    if norm(x(end,:)'-xd)>d
        if mode==1
            mode=2;
        else 
            mode=1;
        end
    end
    
end