function atSetpoint = reachedSetpoint(currentPos, desJ)
    atSetpoint = false;
    disp('desired pos')
    desPos = calcJointPos(desJ)
    close = abs(currentPos-desPos(:,4))< [20;20;20];
    if(close == [1;1;1])
        atSetpoint = true;
    end
    disp('atSetpoint')
    disp(close)

end