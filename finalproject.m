addpath("./InvKin_UR5")
ur5=ur5_interface();

%% RR test
start = [0, -1, 0, 0.47; 0, 0, 1, 0.55; -1, 0, 0, 0.12; 0, 0, 0, 1];
target = [0, -1, 0, -0.30; 0, 0, 1, 0.39; -1, 0, 0, 0.12; 0, 0, 0, 1];
p=zeros([4,4]);
p(3,4)=0.3;
startup=start+p;
targetup=target+p;
home = startup;
home(1:3, 4) = (startup(1:3,4) + targetup(1:3,4)) / 2;

q=ur5InvKin(home);
%ur5.move_joints(ur5.home+[pi/2;pi/5;pi/6;pi/5;pi/10;pi/11],10)
[bestq,found]=find_bestQ(q);
ur5.move_joints(bestq,10);
pause(10);
pick(start,target,"RR",ur5);

%% TJ test
start = [0, -1, 0, 0.47; 0, 0, 1, 0.55; -1, 0, 0, 0.12; 0, 0, 0, 1];
target = [0, -1, 0, -0.30; 0, 0, 1, 0.39; -1, 0, 0, 0.12; 0, 0, 0, 1];
p=zeros([4,4]);
p(3,4)=0.3;
startup=start+p;
targetup=target+p;
home = startup;
home(1:3, 4) = (startup(1:3,4) + targetup(1:3,4)) / 2;

q=ur5InvKin(home);

[bestq,found]=find_bestQ(q);
ur5.move_joints(bestq,10);
pause(10);
pick(start,target,"TJ",ur5);

%% IK test
start = [0, -1, 0, 0.47; 0, 0, 1, 0.55; -1, 0, 0, 0.12; 0, 0, 0, 1];
target = [0, -1, 0, -0.30; 0, 0, 1, 0.39; -1, 0, 0, 0.12; 0, 0, 0, 1];

pick(start,target,"IK",ur5);
