clear all    % Xóa tất cả các biến hiện có ở Workspace
clc          % Xóa mọi dòng trên Command Window

% Khai báo các variables cùng các điều kiện của biến
syms q1 q2 q3 q4 a1 a2 d3 d4
% q1, q2 q3 q4 là các biến khớp (góc quay) 
% d1 d2 d3 d4 khoảng cách hệ trục các khâu

% D-H value
assume(a1,'real');assume(a1>0); 
assume(a2,'real');assume(a2>0); 
assume(d3,'real');assume(d3>0);
assume(d4,'real');assume(d4>0); 
assume(q1,'real');
assume(q2,'real');
assume(q3,'real');
assume(q4,'real');

% Vector các tọa độ suy rộng q
q = [q1;q2;q3;q4];  
q2=pi/6;
% Nhập các ma trận D-H
T_01=[cos(q1) -sin(q1) 0 ; sin(q1)  cos(q1) 0 ; 0 0  1];
T_12=[0  -1 a1; cos(q2) sin(q2) a1; -cos(q2) sin(q2) a1];
T_23=[0  sqrt(3) -sqrt(3); -2 1 1; 1/a1 1/a1 1/a1];
% Tính các ma trận truyền
T_04=simplify(T_01*T_23) % Sau khi tính toán ta thu gọn kết quả ngay bằng lệnh simplify

T3=simplify(T_12*(T_01^-1))
R_04=T_04(1:3,1:3);
