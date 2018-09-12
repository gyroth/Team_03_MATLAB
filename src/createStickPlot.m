function R = createStickPlot( xPos, yPos, zPos )
%createStickPlot Creates a 3D Stick Plot of the Robot arm with the given parameters

R.handle= plot3(xPos,yPos,zPos,'DisplayName','Robot Links','MarkerFaceColor',[1 0 0],...
        'MarkerEdgeColor',[0 0 0],...
        'MarkerSize',10,...
        'Marker','square',...
        'LineWidth',4,...
        'Color',[0.850980401039124 0.325490206480026 0.0980392172932625]);
    grid on
    
    xlim([-200, 350]);
    ylim([-350, 350]);
    zlim([-75, 500]);
    
    
    % Create xlabel
    xlabel({'X-Distance(mm)'});
    
    % Create zlabel
    zlabel({'Z-Distance(mm)'});
    
    % Create title
    title({'Robotic Arm Stickplot'});
    
    % Create ylabel
    ylabel({'Y-Distance(mm)'});

end

