function [ Z ] = createXZStickPlot( xPos, zPos )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Z.handle = plot(xPos, zPos, 'DisplayName','Robot Links','MarkerFaceColor',[1 0 0],...
        'MarkerEdgeColor',[0 0 0],...
        'MarkerSize',10,...
        'Marker','square',...
        'LineWidth',4,...
        'Color',[0.850980401039124 0.325490206480026 0.0980392172932625]);
    grid on
    
    xlim([-10, 400]);
    ylim([-100, 500]);
        
    % Create xlabel
    xlabel({'X-Distance(mm)'});
    
    % Create ylabel
    ylabel({'Z-Distance(mm)'});

end

