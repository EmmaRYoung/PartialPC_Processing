%Purpose:
%Using transformation matrix created in cloud compare, stitch two kinect
%camera views into one point cloud
clear
clc
close all

%read in joints
fname = 'S01TposeJoints.json';
str = fileread(fname);
val = jsondecode(str)
KJ = val.frames(10).bodies.joint_positions; 

%Read in transformation matrix obtained from cloud compare, create
%transformation matrix to use later
T = csvread("TransformSub.txt")

%Read in point cloud ob subject
path1 = "S01TposeMaster.0.ply";
Master = pcread(path1);

path2 = "S01TposeSub.0.ply";
Sub_orig = pcread(path2);

OutFile = "S01_BackAdded.txt";

%Apply transformation matrix to subordinate
% Sub_Tform = pctransform(Sub_orig, tform)
Sub_orig=[Sub_orig.Location,ones(size(Sub_orig.Location,1),1)]';
Sub_Tform=T*Sub_orig;
Sub_orig=Sub_orig(1:3,:)';
Sub_Tform=Sub_Tform(1:3,:)';
Sub_Tform = pointCloud(Sub_Tform)

%combine into one point cloud
% PC_combine = pcmerge(Master, Sub_Tform, 1)
PC_S = Sub_Tform.Location;
PC_M = Master.Location;

%bounding box where person generally should be
%WARNING: specific only to this camera configuration!
x_lim = [-1000 1000];
y_lim = [-1000 1000];
z_lim = [2000 3000];

%remove background, important to do this BEFORE mean centering
PC_S(PC_S(:,1) < x_lim(1), :) = [];
PC_S(PC_S(:,2) < y_lim(1), :) = [];
PC_S(PC_S(:,3) < z_lim(1), :) = [];

PC_S(PC_S(:,1) > x_lim(2), :) = [];
PC_S(PC_S(:,2) > y_lim(2), :) = [];
PC_S(PC_S(:,3) > z_lim(2), :) = [];

PC_M(PC_M(:,1) < x_lim(1), :) = [];
PC_M(PC_M(:,2) < y_lim(1), :) = [];
PC_M(PC_M(:,3) < z_lim(1), :) = [];

PC_M(PC_M(:,1) > x_lim(2), :) = [];
PC_M(PC_M(:,2) > y_lim(2), :) = [];
PC_M(PC_M(:,3) > z_lim(2), :) = [];

%save front view
%writematrix(PC_M, "S01Tpose_FrontOnly.txt")
Tpose = [PC_M; PC_S];
Tpose = pointCloud(Tpose);
showPointCloud(Tpose)
savefig("FrontRight.fig")

%mean center master view, and apply same transformation to sub view

for i=1:3
    PC_M_center(:,i) = PC_M(:,i) - mean(PC_M(:,i));
    PC_S_center(:,i) = PC_S(:,i) - mean(PC_M(:,i));
    KJ_center(:,i) = KJ(:,i) - mean(PC_M(:,i));
end
PC_L_center = [PC_S_center(:,1)*-1, PC_S_center(:,2), PC_S_center(:,3)];

%Combine all views into one array
Tpose_mirror = [PC_M_center; PC_S_center; PC_L_center];


%Un-mean center so joints are in the right location
for i=1:3
    Tpose_mirror(:,i) = Tpose_mirror(:,i) + mean(PC_M(:,i));
end

%Clean up
Tpose_PC = pointCloud(Tpose_mirror);
Tpose_PC = pcdenoise(Tpose_PC);
pcshow(Tpose_PC)
savefig('LeftRight.fig')
% scatter3(Tpose_PC.Location(:,1), Tpose_PC.Location(:,2), Tpose_PC.Location(:,3),'.')
% hold on
% scatter3(KJ(:,1),KJ(:,2),KJ(:,3))
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
%writematrix(Tpose_PC.Location, OutFile)

%% Remove dots on floor
KPC = Tpose_PC.Location;

x = KPC(:,1);
y = KPC(:,2);
z = KPC(:,3);


zDiv = 2600;
xDiv1 = -380;
xDiv2 = 150;
yDiv = 800;

Div2a = z < zDiv;
Div2b = x < xDiv1;
Div2c = x > xDiv2;
Div2d = y > yDiv;
Div2 = and(and(Div2a, Div2b), Div2d);
Div3 = and(and(Div2a, Div2c), Div2d);

xnew = x(Div2);
ynew = y(Div2);
znew = z(Div2);
Body2 = [xnew ynew znew];

xnew = x(Div3);
ynew = y(Div3);
znew = z(Div3);
Body3 = [xnew ynew znew];

DivDiv = and(not(Div2), not(Div3))
xnew = x(DivDiv);
ynew = y(DivDiv);
znew = z(DivDiv);
KPC = [xnew ynew znew];
x = xnew;
y = ynew;
z = znew; 

%% Isolate torso region

%hip joint : 23
%shoulder joint: 13
%upper torso joint: 3

scatter3(KPC(:,1), KPC(:,2), KPC(:,3),'.')
hold on
scatter3(KJ([13, 23, 3],1), KJ([13, 23, 3],2), KJ([13, 23, 3],3))
axis equal
xlabel('x')
ylabel('y')

%estimate where the armpit is 
ydist = 0.75*(abs(KJ(13,2) - KJ(3,2)));
armpit = [KJ(3,1) KJ(3,2)-ydist KJ(3,3)]

%region between armput and hip joint:
yDiv1 = KJ(23,2)
yDiv2 = armpit(:,2)
xDiv1 = KJ(13,1);
xDiv2 = KJ(6,1);

Div1 = y < yDiv1;
Div2 = y > yDiv2;
Div3 = x > xDiv1;
Div4 = x < xDiv2;

DivDiv = and(and(and(Div1, Div2), Div3), Div4);
xnew = x(DivDiv);
ynew = y(DivDiv);
znew = z(DivDiv);
Torso = [xnew ynew znew];
scatter3(Torso(:,1), Torso(:,2), Torso(:,3))
hold on


%% Reconstruct back

%Divide back into slices,
ymin = min(Torso(:,2))
ymax = max(Torso(:,2))

valRange = round(abs(ymin - ymax));
inc = 5;
lower = ymax - inc;
upper = lower - inc*2;
backslice_store = [];
for i=1:valRange/inc
    %get 2d projection of current slice
    k1 = find(Torso(:,2) < lower & Torso(:,2) > upper);% & Torso(:,2) < upper)
    curslice = [Torso(k1,1) Torso(k1,3)];

%     figure;
%     scatter(curslice(:,1), curslice(:,2))
%     hold on

    %find the last two points on either side of the back for the current
    %slice
    %these will probably be the largest "z". So we sort the current slice
    %to find them
    temp = sortrows(curslice, 2);
    side1x = temp(end,1);
    side2x = temp(end-1,1);

    side1z = temp(end,2);
    side2z = temp(end-1,2); 

    backslicex = linspace(side1x,side2x, 30);
    backslicez = linspace(side1z, side2z, 30);
%     scatter(backslicex, backslicez,'filled')
%     axis equal
%     axis off
%     set(gcf,'color', [1 1 1]);


    yval = ones(1,30)*Torso(k1(1),2); 
    backslice = [backslicex' yval' backslicez'];
    KPC = [KPC; backslice];

    backslice_store = [backslice_store; backslice]

    %move current slice up the back
    lower = lower - inc;
    upper = upper - inc; 
end

close all
figure;
scatter3(Torso(:,1), Torso(:,2), Torso(:,3),20)
hold on
axis equal
scatter3(backslice_store(:,1), backslice_store(:,2), backslice_store(:,3),30,'filled')
% axis off
set(gcf,'color', [1 1 1]);
xlabel('x')
ylabel('y')
zlabel('z')
legend("Torso","Estimated Back")


%writematrix(backslice_store, "S22_BackSection.txt", "Delimiter",' ')


%uncomment to export point cloud
% down sample to 2x the number of verts in corresponding SMPL mesh (~4184
% right now)
len = length(KPC);
percent = (4184*2)/len;
KPC_  = pointCloud(KPC);
KPC_d = pcdownsample(KPC_,'random', percent);
figure;
scatter3(KPC_d.Location(:,1),KPC_d.Location(:,2),KPC_d.Location(:,3))

axis equal
writematrix(KPC_d.Location, OutFile, "Delimiter",' ')