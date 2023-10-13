%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                   KIEM TRA HAM TRUYEN DONG CO                       %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Click Run to RUN %%
clc
clear all
close all
% %% %PWM
% IN=[
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 100
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 200
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 300
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 400
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 500
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 600
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 700
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 800
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% 900
% ]*0.1
% ;
% %% Dong co 1
% OUT1= [	

% ];
% %% Dong co 2
% OUT2=[	

% ];
% %% Dong co 2
% OUT3=[	

% ];
load('ident1.mat');
load('ident2.mat');
load('ident3.mat');
IN=PWM;
OUT1=w1;
OUT2=w2;
OUT3=w3;
global PWMLe PWMRi PWMMi
PWMLe=0;
PWMRi=0;
PWMMi=0;
pre_nRi=0;
pre_nLe=0;
pre_nMi=0;
%% --- %%
for j=2:360
    if j==2
        PWMRi = PWMRi + 10;
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 21
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 41
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 61
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 81
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 101
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 121
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 141
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 161
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 181
        PWMRi = 10; 
        PWMLe = 10;
        PWMMi = 10;
    end
    if j == 201
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 221
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 241
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 261
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 281
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 301
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 321
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    if j == 341
        PWMRi = PWMRi + 10; 
        PWMLe = PWMLe + 10;
        PWMMi = PWMMi + 10;
    end
    [tt,y1] = ode45(@motor1Mi_tf, [0 0.01], pre_nMi);
    pre_nMi = y1(length(y1),1);
    wMi(j)  = pre_nMi;
    
    [tt,y2] = ode45(@motor2Ri_tf,[0 0.01],pre_nRi);
    pre_nRi = y2(length(y2),1);
    wRi(j)  = pre_nRi;
    
    [tt,y3] = ode45(@motor3Le_tf, [0 0.01], pre_nLe);
    pre_nLe = y3(length(y3),1);
    wLe(j)  = pre_nLe;
end 
%% --- %%
%% Ve duong dap ung tu ham truyen mo phong va thuc te
%% Dong co 1
plot(IN,'k--','Linewidth',2);
hold on
plot(OUT1,'k','Linewidth',1);
plot(wMi,'k-.','Linewidth',1);
legend('%PWM','w1 thuc te','w1 tinh toan');
xlabel('time(10^-^2s)');
ylabel('RPM');
xlim([0,365]);
ylim([0,350]);
title('Duong dap ung cua ham truyen dong co 1 (ti le fit: 94.59%)');
%% Dong co 2
figure();
plot(IN,'k--','Linewidth',2);
hold on;
plot(OUT2,'k','Linewidth',1);
plot(wRi,'k-.','Linewidth',1);
legend('%PWM','w2 thuc te','w2 tinh toan');
xlabel('time(10^-^2s)');
ylabel('RPM');
xlim([0,365]);
ylim([0,350]);
title('Duong dap ung cua dong co 2 (ti le fit: 95.24%)');
%% Dong co 3
figure();
plot(IN,'k--','Linewidth',2);
hold on;
plot(OUT3,'k','Linewidth',1);
plot(wLe,'k-.','Linewidth',1);
legend('%PWM','w3 thuc te','w3 tinh toan');
xlabel('time(10^-^2s)');
ylabel('RPM');
xlim([0,365]);
ylim([0,350]);
title('Duong dap ung cua dong co 3 (ti le fit: 95.22%)');

%% Duong dap ung toc do thuc te
% plot(IN,'k--','Linewidth',2);
% hold on
% plot(OUT1,'k','Linewidth',2);
% % plot(wRi,'black');
% legend('%PWM','w1 thuc te');
% xlabel('time(10^-^2s)');
% ylabel('RPM');
% xlim([0,365]);
% ylim([0,350]);
% title('Duong dap ung toc do cua dong co 1 tu thuc te');
% % hold on
% figure();
% 
% plot(IN,'k--','Linewidth',2);
% hold on;
% plot(OUT2,'k','Linewidth',2);
% % plot(wLe,'black');
% legend('%PWM','w2 thuc te');
% xlabel('time(10^-^2s)');
% ylabel('RPM');
% xlim([0,365]);
% ylim([0,350]);
% title('Duong dap ung toc cua dong co 2 tu thuc te');
% %hold on
% 
% figure();
% 
% plot(IN,'k--','Linewidth',2);
% hold on;
% plot(OUT3,'k','Linewidth',2);
% % plot(wLe,'black');
% legend('%PWM','w3 thuc te');
% xlabel('time(10^-^2s)');
% ylabel('RPM');
% xlim([0,365]);
% ylim([0,350]);
% title('Duong dap ung toc cua dong co 3 tu thuc te');