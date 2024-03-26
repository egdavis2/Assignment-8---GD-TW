function position = Hover_Drop()
% Reset Grip to 0
p1 = 0
moveGrip(p1)
pause(1);

goHome('qr')    %Putting the robot in its ready position
    resetWorld      %Resets the objects in the world


% First Motion - Hover above can
x1 = -0.032219;
y1 = 0.775;
z1 = 0.487854;

position = moveRobot(x1, y1, z1)

% Second motion - Move directly down to can
x2 = -0.032219;
y2 = 0.79954;
z2 = 0.135;

moveRobot(x2, y2, z2)

end
