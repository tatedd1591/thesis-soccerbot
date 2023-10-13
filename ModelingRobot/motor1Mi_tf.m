%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                      HAM TRUYEN DONG CO 1                           %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Do not use to RUN %%
function dwdt = motor1Mi_tf(t,w)
global K a b PWMMi
K = 73.4;
a = 24.13;
b = 1;

dwdt = (-a*w(1) + K*PWMMi)/b;

end
   