function moveGrip(p)
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                             'control_msgs/FollowJointTrajectory', ...
                             'DataFormat', 'struct');
gripGoal    = rosmessage(grip_client);
gripPos     = p; % 0.8 is fully closed, 0 is fully open 
gripGoal    = packGripGoal_struct(gripPos,gripGoal);
waitForServer(grip_client)
sendGoalAndWait(grip_client,gripGoal)
end