% Graham Davis and Tommy Walker made this script together 

% USER NOTE 
% Ctrl Enter this section first, then the second motion after 
% the first motion has been completely run. 
% Section break is at line 68

% First Motion - Hover above can 

rosshutdown 
clc
clear
rosinit('10.51.63.29'); % This is Grahams URL for the Arm Gazebo
% Change to your URL as needed 

goHome('qr')
resetWorld

grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory', 'DataFormat', 'struct')
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory') 
trajAct.FeedbackFcn = []; 
trajGoal = rosmessage(trajAct)
jointSub = rossubscriber("/joint_states")

UR5e = loadrobot('universalUR5e', DataFormat="row")
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e);
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];
jointStateMsg = receive(jointSub,3)
initialIKGuess = homeConfiguration(UR5e)

initialIKGuess(1) = jointStateMsg.Position(4);  % Shoulder Pan
initialIKGuess(2) = jointStateMsg.Position(3);  % Shoulder Tilt
initialIKGuess(3) = jointStateMsg.Position(1);  % Elbow
initialIKGuess(4) = jointStateMsg.Position(5);  % W1
initialIKGuess(5) = jointStateMsg.Position(6);  % W2
initialIKGuess(6) = jointStateMsg.Position(7);  % W3

gripperX = -0.032286;
gripperY = 0.79944;
gripperZ = 0.42939;

gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians


tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

trajGoal1 = packTrajGoal(UR5econfig,trajGoal)
sendGoal(trajAct,trajGoal1)

%% 
% Second motion - Move directly down to can 


grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory', 'DataFormat', 'struct')
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory') 
trajAct.FeedbackFcn = []; 
trajGoal = rosmessage(trajAct)
jointSub = rossubscriber("/joint_states")

UR5e = loadrobot('universalUR5e', DataFormat="row")
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e);
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];
jointStateMsg = receive(jointSub,3)
initialIKGuess = homeConfiguration(UR5e)

initialIKGuess(1) = jointStateMsg.Position(4);  % Shoulder Pan
initialIKGuess(2) = jointStateMsg.Position(3);  % Shoulder Tilt
initialIKGuess(3) = jointStateMsg.Position(1);  % Elbow
initialIKGuess(4) = jointStateMsg.Position(5);  % W1
initialIKGuess(5) = jointStateMsg.Position(6);  % W2
initialIKGuess(6) = jointStateMsg.Position(7);  % W3

gripperX = -0.032286;
gripperY = 0.79944;
gripperZ = 0.135;

gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians


tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

trajGoal2 = packTrajGoal(UR5econfig,trajGoal)
sendGoal(trajAct,trajGoal2)

trajGoal.Trajectory.Points.Positions

















