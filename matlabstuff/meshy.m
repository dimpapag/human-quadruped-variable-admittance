close all
clear all

[P1, P2] = meshgrid ([430:5:700], [430:5:700]);

P5 = 400;

d0 = 300;
[N,M] = size(P1);

for i=1:N
  for j =1:M
    
    ds(i,j) = smin(P1(i,j),P2(i,j),10)-P5; 
    
    Ws(i,j) = 0.5 * ((1/ds(i,j)) - (1/d0))^2;
    
    d(i,j) = min(P1(i,j),P2(i,j))-P5; 
    
    W(i,j) = 0.5 * ((1/d(i,j)) - (1/d0))^2;
    
    
    
    
  endfor
endfor


surf(P1,P2,Ws, 'FaceAlpha', 0.5)
hold on
surf(P1,P2,W, 'FaceAlpha', 0.5)

xlabel('P1 [mbar]')
ylabel('P2 [mbar]')
zlabel('W')

