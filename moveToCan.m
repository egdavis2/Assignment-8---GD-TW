% Graham Davis and Tommy Walker made this script together 
    
    rosshutdown 
    clc
    clear
    rosinit('10.51.63.29'); % This is Graham's URL for the Arm Gazebo    
    % Change to your URL as needed     

    p1 = 0;
moveGrip(p1)
pause(1);


    goHome('qr')    %Putting the robot in its ready position
    resetWorld      %Resets the objects in the world

x1 = -0.032219;
y1 = 0.79954;
z1 = 0.135;

moveRobot(x1,y1,z1)


