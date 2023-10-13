%% ------------------------------------------------------------------- %%
%%              Mo phong Robot - Do an tot nghiep ME4327               %%
%%                     Truong Dai hoc Bach Khoa                        %%
%%                           Khoa Co khi                               %%
%%                           ---***---                                 %%
%%                      HAM TRUYEN DONG CO 3                           %%
%|               SVTH:                                                 |%
%|                                                                     |%
%|               GVHD:                                                 |%
%|               Gmail:                                                |%
%% ------------------------------------------------------------------- %%
%% Do not use to RUN %%
function dwdt = motor3Le_tf(t,w)
global K a b PWMLe
K = 68.07;
a = 22.29;
b = 1;

dwdt = (-a*w(1) + K*PWMLe)/b;

end
   