%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                            Quy dao                                  %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Do not use to RUN %%
function [xR, yR, phR, wR]=Path(vR,dt)
global x_start y_start xB yB thetaball vball deltat vR
%% Toa do cac diem
%vi tri xe
x1  = x_start             ; y1  = y_start       ;
% vi tri bong
x3 = xB             ; y3 = yB       ;

%giua khung thanh
%phai
x4  = 1200           ; y4 = 0        ;
%trai
x5 = -1200           ; y5 = 0       ;
%kc tu bong den tam khung thanh
d_b=sqrt((x3-x4)^2+(y3-y4)^2);
rate = (d_b+200)/d_b;
%diem lay da
x2  = 1200-(1200-xB)*rate             ; y2  = yB*rate    ;
e=(xB-x_start)*cos(thetaball)+(yB-y_start)*sin(thetaball);
    at=vR*vR-vball*vball;
    deltat=1;
    bt=-2*(vball*vball*deltat+vball*e);
    ct=-2*vball*e*deltat-vball*vball*deltat*deltat - (xB-x_start)*(xB-x_start) - (yB-y_start)*(yB-y_start);
    tpredict    = giaiPTBac2(1,0,1);
    xt1 = xB+vball*(tpredict+1)*cos(thetaball);
    yt1 = yB+vball*(tpredict+1)*sin(thetaball);
%% Thiet lap cac doan line
x = [x1 x3 xt1];
y = [y1 y3 yt1];
plot(x,y,'.')
hold on
axis equal;
axis([-1500 1500 -800 800])

%% 1 ->2
 phR1(1) = atan2(y2-y1,x2-x1);
 xR1(1)  = x1-vR*dt*cos(phR1(1));
 yR1(1)  = y1-vR*dt*sin(phR1(1));
 wR1(1)  = 0;
 L1     = sqrt((x2-x1)^2+(y2-y1)^2);
 k1     = ceil(L1/(vR*dt));
 for i = 2:k1
     phR1(i) = atan2(y2-y1,x2-x1);
     xR1(i)  = xR1(i-1) + vR*dt*cos(phR1(i));
     yR1(i)  = yR1(i-1) + vR*dt*sin(phR1(i));
     wR1(i)  = 0;
 end
%% 2->3
xR2(1)  = x2;
yR2(1)  = y2;
phR2(1) = atan2(y3-y2,x3-x2);
wR2(1)  = 0;
L2     = sqrt((x3-x2)^2+(y3-y2)^2)-150;
k2     = ceil(L2/(vR*dt));
for i = 2:k2
     phR2(i) = atan2(y3-y2,x3-x2);
     xR2(i)  = xR2(i-1) + vR*dt*cos(phR2(i));
     yR2(i)  = yR2(i-1) + vR*dt*sin(phR2(i));
     wR2(i)  = 0;
end
 
%% Ve path - xuat du lieu map tim giao diem
% 
plot(xR1,yR1,'y','LineWidth',1);
% plot(xR2,yR2,'y','LineWidth',1);
  
%      xR  = [xR1 , xR2 ];
%      yR  = [yR1 , yR2 ];
%      phR = [phR1, phR2];
%      wR  = [wR1 , wR2 ];
xR  = [xR1 ];
     yR  = [yR1  ];
     phR = [phR1];
     wR  = [wR1  ];
end
