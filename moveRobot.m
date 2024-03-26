function position = moveRobot(x, y, z)    
    grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory', 'DataFormat', 'struct')   %This creates a client based on the action topic "/gripper_controller/follow_joint_trajectory and type control_msgs/FollowJointTrajectory as a struct"
    trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory') %Creates a trajectory of waypoints for the robot to follow
    trajAct.FeedbackFcn = [];
    trajGoal = rosmessage(trajAct)  %Instantiates the goal
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
    
    gripperX = x;
    gripperY = y;
    gripperZ = z;    
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
    
    trajGoal = packTrajGoal(UR5econfig,trajGoal)
    waitForServer(grip_client)
    sendGoalAndWait(trajAct,trajGoal)
    
    position = trajGoal.Trajectory.Points.Positions'
end