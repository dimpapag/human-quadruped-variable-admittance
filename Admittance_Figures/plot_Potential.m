clear all 
close all

fm = [0.01:0.01:4];
f0 = 2.5;

for i =1:length(fm)
    if fm(i)<f0
        W(i) = 0.5 * ((1/fm(i))-(1/f0))^2 + 0.5*2.0*(fm(i)-f0)^2;
        dW(i) = 1/(f0*(fm(i))^2) -1/(fm(i)^3) + 2.0*(fm(i)-f0);
    else
        W(i) = 0.0;
        dW(i) = 0.0;
    end

end

yyaxis left
plot(fm,W)
ylim([-20,20])
yyaxis right
plot(fm,dW)
ylim([-70,70])
