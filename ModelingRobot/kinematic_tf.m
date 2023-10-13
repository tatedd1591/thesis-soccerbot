%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                    PHUONG TRINH DONG HOC ROBOT                      %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Do not use to RUN %%
function dq = kinematic_tf(t,q)
    global  vInputf vInputn wInput
    dq    = zeros(3,1);
    dq(1) = cos(q(3))*vInputf - sin(q(3))*vInputn; % xM
    dq(2) = sin(q(3))*vInputf + cos(q(3))*vInputn; % yM
    dq(3) = wInput;           % phiM
end