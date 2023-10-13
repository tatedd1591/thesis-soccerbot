%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                            VE SA BAN                                %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Do not use to RUN %%
function Map(vR,dt)
global xt yt x_start y_start
%% Toa do cac diem
%vong tron giua san
rR1 = 125;
x1  = 0             ; y1  = 0       ;
% bien
x21 = -1200         ; y21 = -600    ;
x22 = -1200         ; y22 = 600     ;
x23 = 1200          ; y23 = 600     ;
x24 = 1200          ; y24 = -600    ;
%vach giua san
x31 = 0             ; y31  = -600   ;
x32 = 0             ; y32  = 600    ;
%khung thanh
%phai
x41 = 1200          ; y41 = -200    ;
x42 = 1400          ; y42 = -200    ;
x43 = 1400          ; y43 =  200    ;
x44 = 1200          ; y44 =  200    ;

%trai
x51 = -1200         ; y51 = -200    ;
x52 = -1400         ; y52 = -200    ;
x53 = -1400         ; y53 =  200    ;
x54 = -1200         ; y54 =  200    ;

%penalty box
%phai
x61 = 1200           ; y61 = -250    ;
x62 = 1000           ; y62 = -250    ;
x63 = 1000           ; y63 = 250     ;
x64 = 1200           ; y64 = 250     ;
%trai
x71 = -1200          ; y71 = -250    ;
x72 = -1000          ; y72 = -250    ;
x73 = -1000          ; y73 = 250     ;
x74 = -1200          ; y74 = 250     ;
%
x81 = x_start;          y81= y_start;
x82 = xt;          y82= yt;

%% Thiet lap cac doan line
x = [x1 x21 x22 x23 x24 x31 x32 x41 x42 x43 x44 x51 x52 x53 x54 x61 x62 x63 x64 x71 x72 x73 x74 x81 x82];
y = [y1 y21 y22 y23 y24 y31 y32 y41 y42 y43 y44 y51 y52 y53 y54 y61 y62 y63 y64 y71 y72 y73 y74 y81 y82];
plot(x,y,'.')
hold on
axis equal;
axis([-1500 1500 -800 800])

%% vong tron giua san
xR1(1)  = x1+rR1;
yR1(1)  = y1;
phR1(1) = 0;
wR1(1)  = vR/rR1;
L1     = 2*pi*rR1;
k1     = ceil(L1/(vR*dt));
for i = 2:k1
    phR1(i) = phR1(i-1) + wR1(i-1)*dt;
    xR1(i)  = x1 + rR1*cos(phR1(i));
    yR1(i)  = y1 + rR1*sin(phR1(i));
    wR1(i)  = vR/rR1;
end
%% bien
xR2(1)  = x21;
yR2(1)  = y21;
phR2(1) = pi/2;
wR2(1)  = 0;
L21     = y23*2;
L22     = x23*2;
L23     = L21;
L24     = L22;
k21     = ceil(L21/(vR*dt));
k22     = ceil(L22/(vR*dt));
k23     = ceil(L23/(vR*dt));
k24     = ceil(L24/(vR*dt));
for i = 2:k21 + k22 +k23 + k24
    if i<= k21
        xR2(i)  = xR2(i-1);
        yR2(i)  = yR2(i-1) + vR*dt;
        phR2(i) = pi/2;
        wR2(i)  = 0;
    elseif i<= k21+k22
        xR2(i)  = xR2(i-1) + vR*dt;
        yR2(i)  = yR2(i-1) ;
        phR2(i) = 0;
        wR2(i)  = 0;
    elseif i<= k21+k22+k23
        xR2(i)  = xR2(i-1) ;
        yR2(i)  = yR2(i-1) - vR*dt;
        phR2(i) = -pi/2;
        wR2(i)  = 0;
    else
        xR2(i)  = xR2(i-1) - vR*dt;
        yR2(i)  = yR2(i-1) ;
        phR2(i) = pi;
        wR2(i)  = 0;
    end
end
%% vach giua san
xR3(1)  = x31;
yR3(1)  = y31;
phR3(1) = pi/2;
wR3(1)  = 0;
L31     = y32*2;
k31     = ceil(L31/(vR*dt));
for i = 2:k31
    xR3(i)  = xR3(i-1);
    yR3(i)  = yR3(i-1)+vR*dt;
    phR3(i) = pi/2;
    wR3(i)  = 0;
end
%% khung thanh phai
xR4(1)  = x41;
yR4(1)  = y41;
phR4(1) = 0;
wR4(1)  = 0;
L41     = x43-x44;
L42     = y43*2;
L43     = L41;
L44     = L42;
k41     = ceil(L41/(vR*dt));
k42     = ceil(L42/(vR*dt));
k43     = ceil(L43/(vR*dt));
k44     = ceil(L44/(vR*dt));
for i = 2:k41 + k42 +k43 + k44
    if i<= k41
        xR4(i)  = xR4(i-1) + vR*dt;
        yR4(i)  = yR4(i-1) ;
        phR4(i) = 0;
        wR4(i)  = 0;
    elseif i<= k41+k42
        xR4(i)  = xR4(i-1);
        yR4(i)  = yR4(i-1) + vR*dt ;
        phR4(i) = pi/2;
        wR4(i)  = 0;
    elseif i<= k41+k42+k43
        xR4(i)  = xR4(i-1) - vR*dt;
        yR4(i)  = yR4(i-1) ;
        phR4(i) = pi;
        wR4(i)  = 0;
    else
        xR4(i)  = xR4(i-1) ;
        yR4(i)  = yR4(i-1) - vR*dt;
        phR4(i) = -pi/2;
        wR4(i)  = 0;
    end
end
%% khung thanh trai
xR5(1)  = x51;
yR5(1)  = y51;
phR5(1) = 0;
wR5(1)  = 0;
L51     = x43-x44;
L52     = y43*2;
L53     = L51;
L54     = L52;
k51     = ceil(L51/(vR*dt));
k52     = ceil(L52/(vR*dt));
k53     = ceil(L53/(vR*dt));
k54     = ceil(L54/(vR*dt));
for i = 2:k51 + k52 +k53 + k54
    if i<= k51
        xR5(i)  = xR5(i-1) - vR*dt;
        yR5(i)  = yR5(i-1) ;
        phR5(i) = pi;
        wR5(i)  = 0;
    elseif i<= k51+k52
        xR5(i)  = xR5(i-1);
        yR5(i)  = yR5(i-1) + vR*dt ;
        phR5(i) = pi/2;
        wR5(i)  = 0;
    elseif i<= k51+k52+k53
        xR5(i)  = xR5(i-1) + vR*dt;
        yR5(i)  = yR5(i-1) ;
        phR5(i) = 0;
        wR5(i)  = 0;
    else
        xR5(i)  = xR5(i-1) ;
        yR5(i)  = yR5(i-1) - vR*dt;
        phR5(i) = -pi/2;
        wR5(i)  = 0;
    end
end
%% penalty box phai
xR6(1)  = x61;
yR6(1)  = y61;
phR6(1) = 0;
wR6(1)  = 0;
L61     = 200;
L62     = 500;
L63     = L61;
L64     = L62;
k61     = ceil(L61/(vR*dt));
k62     = ceil(L62/(vR*dt));
k63     = ceil(L63/(vR*dt));
k64     = ceil(L64/(vR*dt));
for i = 2:k61 + k62 +k63 + k64
    if i<= k61
        xR6(i)  = xR6(i-1) - vR*dt;
        yR6(i)  = yR6(i-1) ;
        phR6(i) = pi;
        wR6(i)  = 0;
    elseif i<= k61+k62
        xR6(i)  = xR6(i-1);
        yR6(i)  = yR6(i-1) + vR*dt ;
        phR6(i) = pi/2;
        wR6(i)  = 0;
    elseif i<= k61+k62+k63
        xR6(i)  = xR6(i-1) + vR*dt;
        yR6(i)  = yR6(i-1) ;
        phR6(i) = 0;
        wR6(i)  = 0;
    else
        xR6(i)  = xR6(i-1) ;
        yR6(i)  = yR6(i-1) - vR*dt;
        phR6(i) = -pi/2;
        wR6(i)  = 0;
    end
end
%% penalty box trai
xR7(1)  = x71;
yR7(1)  = y71;
phR7(1) = 0;
wR7(1)  = 0;
L71     = 200;
L72     = 500;
L73     = L71;
L74     = L72;
k71     = ceil(L71/(vR*dt));
k72     = ceil(L72/(vR*dt));
k73     = ceil(L73/(vR*dt));
k74     = ceil(L74/(vR*dt));
for i = 2:k71 + k72 +k73 + k74
    if i<= k71
        xR7(i)  = xR7(i-1) + vR*dt;
        yR7(i)  = yR7(i-1) ;
        phR7(i) = 0;
        wR7(i)  = 0;
    elseif i<= k71+k72
        xR7(i)  = xR7(i-1);
        yR7(i)  = yR7(i-1) + vR*dt ;
        phR7(i) = pi/2;
        wR7(i)  = 0;
    elseif i<= k71+k72+k73
        xR7(i)  = xR7(i-1) - vR*dt;
        yR7(i)  = yR7(i-1) ;
        phR7(i) = 0;
        wR7(i)  = 0;
    else
        xR7(i)  = xR7(i-1) ;
        yR7(i)  = yR7(i-1) - vR*dt;
        phR7(i) = -pi/2;
        wR7(i)  = 0;
    end
end

%% Ve map - xuat du lieu map tim giao diem

plot(xR1,yR1,'c','LineWidth',1);
plot(xR2,yR2,'c','LineWidth',1);
plot(xR3,yR3,'c','LineWidth',1);
plot(xR4,yR4,'c','LineWidth',1);
plot(xR5,yR5,'c','LineWidth',1);
plot(xR6,yR6,'c','LineWidth',1);
plot(xR7,yR7,'c','LineWidth',1);
 
    xR  = [xR1 , xR2 , xR3 , xR4 , xR5 , xR6, xR7 ];
    yR  = [yR1 , yR2 , yR3 , yR4 , yR5 , yR6, yR7 ];
    phR = [phR1, phR2, phR3, phR4, phR5, phR6, phR7];
    wR  = [wR1 , wR2 , wR3 , wR4 , wR5 , wR6, wR7 ];
end
