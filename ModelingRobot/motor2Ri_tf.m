%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                       HAM TRUYEN DONG CO 2                          %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Do not use to RUN %%
function dwdt = motor2Ri_tf(t,w)
global K a b PWMRi
K = 70.53;
a = 22.91;
b = 1;

dwdt = (-a*w(1) + K*PWMRi)/b;

end
   
   