%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                 KIEM TRA BO DIEU KHIEN PID DONG CO                  %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Click Run to RUN %%
clc
clear all
close all
DatatestPIDmotor();
global PWMLe PWMRi PWMMi 
PWMLe = 0;
PWMRi = 0;
PWMMi = 0;

t(1)        = 0;
dtpid       = 0.01;
vR          = 400;
giamchandc  = 1;
omegandc    = 4/(giamchandc*0.1);
%% PID dong co trai
pre_nLe = 0;
eLe     = 0;
int_eLe = 0;
de_dtLe = 0;

kpLe    = (2*giamchandc*omegandc-22.29)/68.07;
kiLe    = omegandc^2/68.07;
kdLe 	= 0; 
%% PID dong co phai
pre_nRi = 0;
eRi     = 0;
int_eRi = 0;
de_dtRi = 0;

kpRi    = (2*giamchandc*omegandc-22.91)/70.53;
kiRi    = omegandc^2/70.53;
kdRi    = 0;

%% PID dong co mid
pre_nMi = 0;
eMi     = 0;
int_eMi = 0;
de_dtMi = 0;

kpMi    = (2*giamchandc*omegandc-24.13)/73.4;
kiMi    = omegandc^2/73.4;
kdMi    = 0;

%% Mo phong
% ref=100;%500/(85/2)*(30/pi);
for j=2:200
    if      j <= 100
        ref = 80;
        OUTPID3(j)=OUTPID3(j)*80/88;
    elseif  j > 100
        ref = 40;
    end 
%% PID dong co MID
    pre_eMi = eMi;
    eMi     = ref - pre_nMi;
    int_eMi = int_eMi + (eMi)*dtpid;
    de_dtMi = (eMi-pre_eMi)/dtpid;
    PWMMi   = (kpMi*eMi + kiMi*int_eMi + kdMi*de_dtMi);
    if PWMMi > 90
        PWMMi  = 90;
    end
    [tt,y]  = ode45(@motor1Mi_tf,[0 dtpid],pre_nMi);
    pre_nMi = y(length(y),1);
    wMi(j)  = pre_nMi;
%% PID dong co trai
    pre_eLe = eLe;
    eLe     = ref - pre_nLe;
    int_eLe = int_eLe + (eLe)*dtpid;
    de_dtLe = (eLe-pre_eLe)/dtpid;
    PWMLe   = (kpLe*eLe + kiLe*int_eLe + kdLe*de_dtLe);
    if PWMLe > 90
        PWMLe  = 90;
    end
    [tt,y]  = ode45(@motor3Le_tf,[0 dtpid],pre_nLe);
    pre_nLe = y(length(y),1);
    wLe(j)  = pre_nLe;
%% PID dong co phai
    pre_eRi = eRi;
    eRi     = ref - pre_nRi;
    int_eRi = int_eRi + (eRi)*dtpid;
    de_dtRi = (eRi-pre_eRi)/dtpid;
    PWMRi   = (kpRi*eRi + kiRi*int_eRi + kdRi*de_dtRi);
    if PWMRi > 90
        PWMRi  = 90;
    end   
    [tt,y]  = ode45(@motor2Ri_tf,[0 dtpid],pre_nRi);
    pre_nRi = y(length(y),1);
    wRi(j)  = pre_nRi;
    t(j)    = dtpid*j;
    a(j)    = PWMLe;
    b(j)    = PWMRi;
    c(j)    = PWMMi;

end 
plot(t,wLe,'black--','Linewidth',1.5);
hold on;
plot(t,OUTPID2,'k');
legend('w3','w3 thuc te');
xlabel('time(s)');
ylabel('RPM');
xlim([0,2]);
ylim([0,100]);
title('Duong dap ung ly thuyet va thuc te cua dong co 3');
figure();
plot(t,wRi,'black--','Linewidth',1.5);
hold on;
plot(t,OUTPID1,'k');
legend('w2 ','w2 thuc te');
xlabel('time(s)');
ylabel('RPM');
xlim([0,2]);
ylim([0,100]);
title('Duong dap ung ly thuyet va thuc te cua dong co 2');
figure();
plot(t,wRi,'black--','Linewidth',1.5);
hold on;
plot(t,OUTPID3,'k');
legend('w1 ','w1 thuc te');
xlabel('time(s)');
ylabel('RPM');
xlim([0,2]);
ylim([0,100]);
title('Duong dap ung ly thuyet va thuc te cua dong co 1');
