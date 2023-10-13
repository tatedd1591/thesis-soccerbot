%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                          TEST DONG HOC                              %%
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
    global vInputf vInputn wInput xM yM phM
    global st_px st_py st_pth vf vn w R r dw vR
    % Start point 
    st_px   = 0    ;                    
    st_py   = 0    ;
    st_pth  = 0    ;    
    % Dai luong mong muon
    % Toc do tham chieu can ðat.
    vf      = 400  ;
    vn      = 00    ;
    vR      = sqrt(vf^2+vn^2);
    w       = 2*pi    ;
    dt      = 0.005  ;                                % Thoi gian lay mau
    t       = 0    ;                                % Time start
    
%% Thong so co ban cua robot
    r       = 58/2  ;                               % Ban kinh banh xe.
    R       = 90    ;                               % ban kinh khung xe
    dw       = 13    ;                               % 1 nua do day banh xe
%% Ve Map
    hold on
    axis equal;
    axis([-600 600 -200 600])
%% Vi tri bat dau
    xM(1) = st_px;
    yM(1) = st_py;
    phM(1)= st_pth;
    pre_pos = [xM(1); yM(1); phM(1)];
    i=1;
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
    h8=plot(1,1, 'blue');
    % Truc yM
    h9=plot(1,1, 'red');
    % Vet cac doi tuong
    obit=animatedline('Color','black','LineWidth',1);       % Tam khung xe
    banhmid=animatedline('Color','r','LineWidth',0.5);      % Banh dong co 1
    banhright=animatedline('Color','b','LineWidth',0.5);    % Banh dong co 2
    banhleft=animatedline('Color','g','LineWidth',0.5);     % Banh dong co 3

%% Mo phong bam line
% while (size(P,2) ~= 0)
    while (t<=2)
        %% Cap nhat vi tri hinh dang robot di chuyen
        delete(h1);
%         delete(h2);
%         delete(h3);
%         delete(h4);
        delete(h5);
        delete(h6);
        delete(h7);
        delete(h8);
        delete(h9);
        %% Quy tich cac chi tiet
        % Than xe
        phb     =linspace(phM(i),phM(i)+2*pi,180);
        xb1     = xM(i) + 100*cos(phb);
        yb1     = yM(i) + 100*sin(phb);
        % Truc banh
        xw1     = linspace(xM(i), xM(i) + 100*cos(phM(i)+pi), 2);
        yw1     = linspace(yM(i), yM(i) + 100*sin(phM(i)+pi), 2);
        xw2     = linspace(xM(i), xM(i) + 100*cos(phM(i)+pi/3), 2);
        yw2     = linspace(yM(i), yM(i) + 100*sin(phM(i)+pi/3), 2);
        xw3     = linspace(xM(i), xM(i) + 100*cos(phM(i)-pi/3), 2);
        yw3     = linspace(yM(i), yM(i) + 100*sin(phM(i)-pi/3), 2);
        % truc y
        xn   = linspace(xM(i)   , xM(i) + 50*cos(phM(i)+pi/2),2);
        yn   = linspace(yM(i)   , yM(i) + 50*sin(phM(i)+pi/2),2);
        % truc x
        xf   = linspace(xM(i)   , xM(i) + 50*cos(phM(i)),2);
        yf   = linspace(yM(i)  , yM(i) + 50*sin(phM(i)),2);
        % Banh xe
        xwmid   = linspace( xM(i) + 100*cos(phM(i)+pi)-r*sin(phM(i)+pi), xM(i) + 100*cos(phM(i)+pi)+r*sin(phM(i)+pi) , 2);
        ywmid   = linspace( yM(i) + 100*sin(phM(i)+pi)+r*cos(phM(i)+pi), yM(i) + 100*sin(phM(i)+pi)-r*cos(phM(i)+pi), 2);
        xwleft  = linspace( xM(i) + 100*cos(phM(i)+pi/3)-r*sin(phM(i)+pi/3), xM(i) + 100*cos(phM(i)+pi/3)+r*sin(phM(i)+pi/3) , 2);
        ywleft  = linspace( yM(i) + 100*sin(phM(i)+pi/3)+r*cos(phM(i)+pi/3), yM(i) + 100*sin(phM(i)+pi/3)-r*cos(phM(i)+pi/3), 2);
        xwright = linspace( xM(i) + 100*cos(phM(i)-pi/3)-r*sin(phM(i)-pi/3), xM(i) + 100*cos(phM(i)-pi/3)+r*sin(phM(i)-pi/3) , 2);
        ywright = linspace( yM(i) + 100*sin(phM(i)-pi/3)+r*cos(phM(i)-pi/3), yM(i) + 100*sin(phM(i)-pi/3)-r*cos(phM(i)-pi/3), 2);
        
        h1=plot(xb1, yb1, 'black','LineWidth',1); % Than xe.
        
        %% Output
        % Truc banh
%         h2      = plot(xw1, yw1, 'black','Linewidth',1);
%         h3      = plot(xw2, yw2, 'black','Linewidth',1);         
%         h4      = plot(xw3, yw3, 'black','Linewidth',1);
        % Banh xe
        h5      = plot(xwmid, ywmid, 'blue','Linewidth',dw/2);    % Banh dong co 1.
        h6      = plot(xwright, ywright, 'blue','Linewidth',dw/2);% Banh dong co 2.
        h7      = plot(xwleft, ywleft, 'blue','Linewidth',dw/2);  % Banh dong co 3.
        drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} );
        % truc y
         h8      = plot(xn, yn, 'red','Linewidth',1);
%         h8      = drawArrow(xn,yn,'linewidth',1,'color','r');
        % truc x
        h9      = plot(xf, yf, 'blue','Linewidth',1);
               

        % Vet banh xe va khung xe
        addpoints(banhmid ,  xM(i) + 100*cos(phM(i)+pi)  , yM(i) + 100*sin(phM(i)+pi)  );
        addpoints(banhleft , xM(i) + 100*cos(phM(i)+pi/3), yM(i) + 100*sin(phM(i)+pi/3));
        addpoints(banhright, xM(i) + 100*cos(phM(i)-pi/3), yM(i) + 100*sin(phM(i)-pi/3));
        addpoints(obit, xM(i), yM(i));
        drawnow;
%     if (t<1)
%             vfi(i)=0;
%             vni(i)=0;
%             wi(i)=0;
    if (t<1)
            vfi(i)=00*cos(-0*w/2*i*dt+0*pi/2);
            vni(i)=00*sin(-0*w/2*i*dt+0*pi/2);
            wi(i)=2*pi;
    else
            vfi(i)=-00*cos(-0*w/2*i*dt+0*pi/2);
            vni(i)=-00*sin(-0*w/2*i*dt+0*pi/2);
            wi(i)=-2*pi;
    end
        % Van toc 3 banh.
        vMi(i) = -vni(i)+R*wi(i);
        vRi(i) = vfi(i)*cos(pi/6) +vni(i)*cos(pi/3)+ R*wi(i);
        vLe(i) = -vfi(i)*cos(pi/6) +vni(i)*cos(pi/3)+ R*wi(i);
        % Toc do quay dong co Rad/s
        wLe(i) = vLe(i)/r;
        wRi(i) = vRi(i)/r;
        wMi(i) = vMi(i)/r;
        
 
        %% Dong hoc thuan
%             W_inv = (r/(3*1))* [0          ,   sqrt(3)     ,   -sqrt(3);
%                                 -2         ,   1           ,   1       ;
%                                 1/R        ,   1/R         ,   1/R     ];
            a_inv =  [0          ,         -1,   R;
                      cos(pi/6)  ,  sin(pi/6),   R;
                      -cos(pi/6)  ,  sin(pi/6),   R];
            W_inv = r*a_inv^-1;
            ro(:,i) = [wMi(i);wRi(i);wLe(i)];
            pos(:,i) = W_inv*ro(:,i); 
            vInputf = pos(1,i);
            vInputn = pos(2,i);
            wInput  = pos(3,i);

        %% Cap nhat vi tri robot qua phuong trinh dong hoc
            [tt,y]  = ode45(@kinematic_tf, [0 dt], pre_pos);
            xM(i+1) = y(length(y), 1);
            yM(i+1) = y(length(y), 2);
            phM(i+1)= y(length(y), 3);
            pre_pos = [xM(i+1); yM(i+1); phM(i+1)];
            xM(i+1) = xM(i+1);
            yM(i+1) = yM(i+1);      
    t(i)=(i)*dt;                    
    i=i+1;                  
    end

%% Ve do thi
    %% Ve vet bam line - mo phong chuyen dong
    xlabel('mm');
    xlim([-600 600]);
    ylim([-200 600]);
    title('Mo phong');
    
     %% Van toc dau vao
    figure();
    yyaxis left
    plot(t,vfi,'k:','Linewidth',3,'MarkerSize',100);
    hold on
    plot(t,vni,'k-.','Linewidth',3,'MarkerSize',100);
     ylim([-600 600]);
     ylabel('Van toc dai Vx, Vy (mm/s)','FontSize',20,'Color','black');
    yyaxis right
    plot(t,wi/pi,'k--','Linewidth',3,'MarkerSize',100);
    ylim([-2.5 2.5]);
    legend('Vx','Vy','wz','FontSize',20);
    xlabel('time(s)','FontSize',20,'Color','black');
    ylabel('Van toc goc (pi*rad/s)','FontSize',20,'Color','black');
    title('Van toc dau vao','FontSize',20,'Color','black');

    %% Van toc 3 banh
    figure();

    plot(t,wMi*30/pi,'k:','Linewidth',3,'MarkerSize',100)
    hold on
    plot(t,wRi*30/pi,'k-.','Linewidth',3,'MarkerSize',100)
    hold on
    plot(t,wLe*30/pi,'k--','Linewidth',3,'MarkerSize',100);

    legend('w1','w2','w3','FontSize',20);
    xlabel('time(s)','FontSize',20,'Color','black');
    ylabel('Van toc goc (RPM)','FontSize',20,'Color','black');
    title('Van toc goc cua 3 dong co','FontSize',20,'Color','black');

    ylim([-300 300]);

    %% Van toc banh 1
    figure();

    plot(t/10,vMi,'r','Linewidth',1);

    legend('v1 ');
    xlabel('time(s)');
    ylabel('Van toc v (mm/s)');
    title('Do thi van toc v cua dong co 1 ');

    ylim([-600 600]);

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

    plot(t/10,vLe,'g','Linewidth',1);

    legend('v3');
    xlabel('time(s)');
    ylabel('Van toc v (mm/s)');
    title('van toc v cua dong co 3');

    ylim([-1000 1000]);

%% Ket thuc