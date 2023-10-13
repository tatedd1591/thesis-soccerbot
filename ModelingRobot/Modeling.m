%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                       MO PHONG SOCCER ROBOT                         %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Click Run to RUN %%
    clc
    clear all
    close all

%% Khai bao cac bien co ban
    global PWMLe PWMRi PWMMi vInputf vInputn wInput d xM yM xB yB 
    global thetaM r vR oR R x_start y_start theta_start
    % He so lyapunov 
    k1      = 1   ;
    k2      = 0.1 ;
    k3      = 1 ;
    % Point
        % Start point (1)
        x_start   = -200   ;                    
        y_start   = -500   ;
        theta_start  = 0.5*pi ; 
        % Ball point (3)
        xB      = 400 ;
        yB      = 300 ;
        % Speed point (2)
        xt      = 1200-(1200-xB)*1.5              ; 
        yt      = yB*1.5                        ;
        pht     = atan2(yB-yt,xB-xt) ;   
    % Length
    lt1     = sqrt((xt-x_start)^2+(yt-y_start)^2) ;     % Khoang cach tu 1 -> 2 
    lt2     = sqrt((xB-xt)^2+(yB-yt)^2)       ;     % Khoang cach tu 2 -> 3
    %PWM dong co
    PWMMi   = 0 ;                                   % PWM dc 1.
    PWMRi   = 0 ;                                   % PWM dc 2.
    PWMLe   = 0 ;                                   % PWM dc 3.     
    % Dai luong mong muon
    vR      = 400  ;                                % Toc do tham chieu can ðat.
    tp      = vR/3 ;
    oR      = 0.1  ;                                % Toc do goc
    dt      = 0.1  ;                                % Thoi gian lay mau PID bam quy dao.
    dtpid   = 0.01 ;                                % Thoi gian lay mau PID dong co.
    t       = 0    ;                                % Time start
    % Do lech chuan sai so do cam bien gay ra.
    errorcb = 0.5   ;                               % Sai so cam bien
    errorcby= 0     ;
    errorcbp= 0     ;
    
%% Thong so co ban cua robot
    r       = 58/2  ;                               % Ban kinh banh xe.
    R       = 90    ;                               % ban kinh khung xe
    w       = 13    ;                               % 1 nua do day banh xe
    d       = 0     ;                               % tam cb
    
%% Ve Map
    Map(1, dtpid);                                  % San dau
    [xR, yR, phR, wR] = Path(vR,dtpid);             % Quy dao

%% Vi tri bat dau
    xM(1) = x_start;
    yM(1) = y_start;
    thetaM(1)= theta_start;
    pre_pos = [xM(1); yM(1); thetaM(1)];

%% Hinh dang robot
    % Than xe
    h1=plot(1,1, 'black');                      
    % Truc banh xe.
    h2=plot(1,1, 'black');
    h3=plot(1,1, 'black');          
    h4=plot(1,1, 'black');
    % Banh xe
    h5=plot(1,1, 'blue');                           % Banh dong co 1.
    h6=plot(1,1, 'blue');                           % Banh dong co 2
    h7=plot(1,1, 'blue');                           % Banh dong co 3.
    % Truc xM
    h8=plot(1,1, 'green');                      
    % Bong
    h9=plot(1,1, 'black');
    % Vet cac doi tuong
    obit=animatedline('Color','black','LineWidth',1);       % Tam khung xe
    banhmid=animatedline('Color','r','LineWidth',0.5);      % Banh dong co 1
    banhright=animatedline('Color','b','LineWidth',0.5);    % Banh dong co 2
    banhleft=animatedline('Color','g','LineWidth',0.5);     % Banh dong co 3

% %% Bo dieu khien PID bam line
%     pre_E2=0;
%     intE2=0;
% 
%     %||| omegan <2*giamchan*vR/d    |||%
%     omegan=4.4;                     % He so giam chan PID bam line.
%     giamchan=1;                     % Tan so giao dong rieng PID bam line.
%     %||| hoi tiep: K=[k1;k2]        |||%
%     k1=-omegan^2/vR;          
%     k2=-(2*giamchan*omegan+d*k1)/vR;
%     %||| Bo so PID                  |||%
%     kpE2=-1.*k1/(1+k2*d);
%     kdE2=-1.*k2/(1+k2*d);
%     KiE2=0.1;
%% Bo dieu khien PID dong co
    giamchandc=1;                   % He so giam chan PID dong co
    omegandc=4/(giamchandc*0.1);    % Tan so giao dong rieng PID dong co
    %||| Dong co 2 - trai           |||%
    pre_nLe=0;
    eLe=0;
    int_eLe=0;
    de_dtLe=0;

    kpLe=(2*giamchandc*omegandc-32.65)/76.66;
    kiLe=omegandc^2/76.66;
    kdLe=0.0; 
    %||| Dong co 1 - phai          |||%
    pre_nRi=0;
    eRi=0;
    int_eRi=0;
    de_dtRi=0;

    kpRi=(2*giamchandc*omegandc-32.05)/73.2;
    kiRi=omegandc^2/73.2;
    kdRi=0.00;
    i=1;
    %||| Dong co 3 - giua           |||%
    pre_nMi=0;
    eMi=0;
    int_eMi=0;
    de_dtMi=0;

    kpMi=(2*giamchandc*omegandc-32.65)/76.66;
    kiMi=omegandc^2/76.66;
    kdMi=0.0; 

%% Mo phong bam line
% while (size(P,2) ~= 0)
    while (t<30)
        %% Cap nhat vi tri hinh dang robot di chuyen
        delete(h1);
        delete(h2);
        delete(h3);
        delete(h4);
        delete(h5);
        delete(h6);
        delete(h7);
        delete(h8);
        
        %% Quy tich cac chi tiet
        % Than xe
        phb     =linspace(thetaM(i),thetaM(i)+2*pi,180);
        xb1     = xM(i) + 100*cos(phb);
        yb1     = yM(i) + 100*sin(phb);
        % Truc banh
        xw1     = linspace(xM(i), xM(i) + 100*cos(thetaM(i)+pi), 2);
        yw1     = linspace(yM(i), yM(i) + 100*sin(thetaM(i)+pi), 2);
        xw2     = linspace(xM(i), xM(i) + 100*cos(thetaM(i)+pi/3), 2);
        yw2     = linspace(yM(i), yM(i) + 100*sin(thetaM(i)+pi/3), 2);
        xw3     = linspace(xM(i), xM(i) + 100*cos(thetaM(i)-pi/3), 2);
        yw3     = linspace(yM(i), yM(i) + 100*sin(thetaM(i)-pi/3), 2);
        % Thanh
        xs   = linspace(xM(i) - 100*sin(thetaM(i))  , xM(i) + 100*sin(thetaM(i)),2);
        ys   = linspace(yM(i) + 100*cos(thetaM(i))  , yM(i) - 100*cos(thetaM(i)),2);
        % Banh xe
        xwmid   = linspace( xM(i) + 100*cos(thetaM(i)+pi)-30*sin(thetaM(i)+pi), xM(i) + 100*cos(thetaM(i)+pi)+30*sin(thetaM(i)+pi) , 2);
        ywmid   = linspace( yM(i) + 100*sin(thetaM(i)+pi)+30*cos(thetaM(i)+pi), yM(i) + 100*sin(thetaM(i)+pi)-30*cos(thetaM(i)+pi), 2);
        xwleft  = linspace( xM(i) + 100*cos(thetaM(i)+pi/3)-30*sin(thetaM(i)+pi/3), xM(i) + 100*cos(thetaM(i)+pi/3)+30*sin(thetaM(i)+pi/3) , 2);
        ywleft  = linspace( yM(i) + 100*sin(thetaM(i)+pi/3)+30*cos(thetaM(i)+pi/3), yM(i) + 100*sin(thetaM(i)+pi/3)-30*cos(thetaM(i)+pi/3), 2);
        xwright = linspace( xM(i) + 100*cos(thetaM(i)-pi/3)-30*sin(thetaM(i)-pi/3), xM(i) + 100*cos(thetaM(i)-pi/3)+30*sin(thetaM(i)-pi/3) , 2);
        ywright = linspace( yM(i) + 100*sin(thetaM(i)-pi/3)+30*cos(thetaM(i)-pi/3), yM(i) + 100*sin(thetaM(i)-pi/3)-30*cos(thetaM(i)-pi/3), 2);
        % Bong
        phball=linspace(0,2*pi,180);
        xball   = xB + 25*cos(phball);
        yball   = yB + 25*sin(phball);
        h1=plot(xb1, yb1, 'black','LineWidth',1); % Than xe.
        
        %% Output
        % Truc banh
        h2      = plot(xw1, yw1, 'black','Linewidth',1);
        h3      = plot(xw2, yw2, 'black','Linewidth',1);         
        h4      = plot(xw3, yw3, 'black','Linewidth',1);
        % Banh xe
        h5      = plot(xwmid, ywmid, 'blue','Linewidth',2);    % Banh dong co 1.
        h6      = plot(xwright, ywright, 'blue','Linewidth',2);% Banh dong co 2.
        h7      = plot(xwleft, ywleft, 'blue','Linewidth',2);  % Banh dong co 3.
        % Thanh
        h8      = plot(xs, ys, 'black','Linewidth',1);
        % Bong
        h9      = plot(xball, yball, 'black','Linewidth',1);
        % Vet banh xe va khung xe
        addpoints(banhmid ,  xM(i) + 100*cos(thetaM(i)+pi)  , yM(i) + 100*sin(thetaM(i)+pi)  );
        addpoints(banhleft , xM(i) + 100*cos(thetaM(i)+pi/3), yM(i) + 100*sin(thetaM(i)+pi/3));
        addpoints(banhright, xM(i) + 100*cos(thetaM(i)-pi/3), yM(i) + 100*sin(thetaM(i)-pi/3));
        addpoints(obit, xM(i), yM(i));
        drawnow;
        
        %% Giao diem giua tam robot va tam quy dao
        P = Giaodiem([xs;ys],[xR;yR]);
        if size(P,2) == 0
        break;
        end

        %% Tinh toan sai so
        e(1,i) =  (P(1,1) - xM(i))* cos(thetaM(i)) + (P(2,1) - yM(i))*sin(thetaM(i));                                                                        
        e(2,i) = -(P(1,1) - xM(i))* sin(thetaM(i)) + (P(2,1) - yM(i))*cos(thetaM(i))+errorcb*randn;
        e(3,i) =   phR(i) - thetaM(i);
        
%         %% Tinh toan van toc goc w tu PID --> van toc dai v cua 2 banh 
%         eE2    = e(2,i);
%         intE2  = intE2+(eE2+pre_E2)*dt;
%         w(i)   = (kpE2*eE2 + kdE2*(eE2-pre_E2)/dt + KiE2*(intE2)); %PID
%         pre_E2 = e(2,i);

        %% Bo dieu khien Lyapunov
        eE3=e(3,i);
        eE1=e(1,i);
        eE2= e(2,i);
        v(i) = vR*cos(eE3)+k1*eE1;
        w(i) = k2*vR*eE2/1000 + k3*sin(eE3);

        % Van toc 3 banh.
        vMi(i) = R*w(i);
        vRi(i) = v(i)*cos(pi/6) + R*w(i);
        vLe(i) = -v(i)*cos(pi/6) + R*w(i);
        % Toc do quay dong co Rad/s
        wLe(i) = vLe(i)/r;
        wRi(i) = vRi(i)/r;
        wMi(i) = vMi(i)/r;
        
        %% PID dong co
        jj=round(dt/dtpid);
        if jj==0
            jj=1;
        end
        for j=1:(jj)
            if i>1
            %% Dong co 1 - giua
                pre_eMi = eMi;
                eMi     = wMi(i)*30/pi - pre_nMi;
                int_eMi = int_eMi + (eMi)*dtpid;
                de_dtMi = (eMi-pre_eMi)/dtpid;
                PWMMi   = (kpMi*eMi + kiMi*int_eMi + kdMi*de_dtMi);
                % Gioi han PWM 90%.
                if PWMMi>90
                PWMMi=90;
                end
                [tt,y]  = ode45(@motor1Mi_tf, [0 dtpid], pre_nMi);
                pre_nMi = y(length(y),1);
                vMi(i)  = pre_nMi*pi/30*r;
            %% Dong co 2 - phai
                pre_eRi = eRi;
                eRi     = wRi(i)*30/pi - pre_nRi;
                int_eRi = int_eRi + (eRi)*dtpid;
                de_dtRi = (eRi-pre_eRi)/dtpid;
                PWMRi   = (kpRi*eRi + kiRi*int_eRi + kdRi*de_dtRi);
                % Gioi han PWM 90%.
                if PWMRi>90
                PWMRi=90;
                end   
                [tt,y]  = ode45(@motor2Ri_tf, [0 dtpid], pre_nRi);
                pre_nRi = y(length(y),1);
                vRi(i)  = pre_nRi*pi/30*r;
            %% Dong co 3 - trai
                pre_eLe = eLe;
                eLe     = wLe(i)*30/pi - pre_nLe;
                int_eLe = int_eLe + (eLe)*dtpid;
                de_dtLe = (eLe-pre_eLe)/dtpid;
                PWMLe   = (kpLe*eLe + kiLe*int_eLe + kdLe*de_dtLe);
                % Gioi han PWM 90%.
                if PWMLe>90
                PWMLe=90;
                end
                [tt,y]  = ode45(@motor3Le_tf, [0 dtpid], pre_nLe);
                pre_nLe = y(length(y),1);
                vLe(i)  = pre_nLe*pi/30*r;
            else 
                vLe(i)=0;
                vRi(i)=0;
                vMi(i)=0;
            end

        %% Dong hoc thuan
            W_inv = (r/(3*1))* [0          ,   sqrt(3)     ,   -sqrt(3);
                                -2         ,   1           ,   1       ;
                                1/R        ,   1/R         ,   1/R     ];
            ro(:,i) = [vMi(i)/r;vRi(i)/r;vLe(i)/r];
            pos(:,i) = W_inv*ro(:,i); 
            vInputf = pos(1,i);
            vInputn = pos(2,i);
            wInput  = pos(3,i);

        %% Cap nhat vi tri robot qua phuong trinh dong hoc
            [tt,y]  = ode45(@kinematic_tf, [0 dtpid], pre_pos);
            xM(i+1) = y(length(y), 1);
            yM(i+1) = y(length(y), 2);
            thetaM(i+1)= y(length(y), 3);
            pre_pos = [xM(i+1); yM(i+1); thetaM(i+1)];
            xM(i+1) = xM(i+1);
            yM(i+1) = yM(i+1);
        end
    t(i)=(i)*dt;                    
    i=i+1;                  
    end

%% Ve do thi
    %% Ve vet bam line - mo phong chuyen dong
    xlabel('mm');

    xlim([-1500 1500]);
    ylim([-800 800]);
    title('Mo phong bam quy dao');
    %% Do thi sai so bam e2 va toc do goc w tu PID bam
    figure();

    plot(t,e(2,:),'r','Linewidth',1);
    hold on
    plot(t,w,'b--','Linewidth',1);

    legend('Sai so bam e2', 'toc do goc w');
    xlabel('time(s)');
    ylabel('Sai so bam e2 (mm)');
    title('Do thi sai so bam e2 ');
    %% Van toc 3 banh
    figure();

    plot(t,vMi,'r-.','Linewidth',1)
    hold on
    plot(t,vRi,'b--','Linewidth',1)
    hold on
    plot(t,vLe,'g','Linewidth',1);

    legend('v1','v2','v3');
    xlabel('time(s)');
    ylabel('Van toc v (mm/s)');
    title('van toc v cua 3 dong co');

    ylim([-1000 1000]);

    %% Van toc banh 1
    figure();

    plot(t,vMi,'r','Linewidth',1);

    legend('v1 ');
    xlabel('time(s)');
    ylabel('Van toc v (mm/s)');
    title('Do thi van toc v cua dong co 1 ');

    ylim([-1000 1000]);

    %% Van toc banh 2
    figure();

    plot(t,vRi,'b','Linewidth',1);

    legend('v2 ');
    xlabel('time(s)');
    ylabel('Van toc v (mm/s)');
    title('Do thi van toc v cua dong co 2 ');

    ylim([-1000 1000]);

    %% Van toc banh 3
    figure();

    plot(t,vLe,'g','Linewidth',1);

    legend('v3');
    xlabel('time(s)');
    ylabel('Van toc v (mm/s)');
    title('van toc v cua dong co 3');

    ylim([-1000 1000]);

    %% Do thi sai so bam e3 va toc do goc w tu PID bam line
    figure();

    plot(t,e(3,:)*180/pi,'m','Linewidth',1);

    legend('Sai so bam e3');
    xlabel('time(s)');
    ylabel('Sai so bam e3 (mm)');
    title('Do thi sai so bam e3');
%% Ket thuc