if strcmp(experiment_name, 'walking')
yLimPos = 0.1 ;
yLimVel =  0.2;
end

if strcmp(experiment_name, 'com-sinusoid')
yLimPos = 0.1;
yLimVel =  0.15;
end

if strcmp(experiment_name, 'gazebo-v3-walking')
yLimPos = 0.35;
yLimVel =  0.5;
end


%% DILIGENT_KIO
if enable_diligent_kio
figure
jdx = 1;
title1 = {'error x (m)', 'error y (m)', 'error z (m)'};
title2 = {'error x (m/s)', 'error y (m/s)', 'error z (m/s)'};

for idx = [1, 7, 2, 8, 3, 9]
    subplot(3, 2, jdx)
    devLine = plot(estBaseTime(startIter:endIter), 3*sqrt(squeeze(outMap(diligent_kiomat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    hold on;
    plot(estBaseTime(startIter:endIter), -3*sqrt(squeeze(outMap(diligent_kiomat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    if idx == 1 || idx == 2 || idx == 3
        errLine = plot(estBaseTime(startIter:endIter), errors.diligent_kioL.posError(startIter:endIter, idx), ...
            'b', 'LineWidth', 2);
        ylabel(title1{idx}, 'FontSize', 22, 'FontWeight', 'bold')  
    elseif idx == 7 || idx == 8 || idx == 9
        errLine = plot(estBaseTime(startIter:endIter), errors.diligent_kioL.velError(startIter:endIter, idx-6), ...
            'b', 'LineWidth', 2);
        ylabel(title2{idx - 6}, 'FontSize', 22, 'FontWeight', 'bold')  
    end
    jdx = jdx+1;

    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 7 || idx == 8 || idx == 9)
        ylim([-yLimVel yLimVel])
    else
        ylim([-yLimPos yLimPos])
    end

    if (idx == 3 || idx == 9)
        xlabel('Time(s)', 'FontSize', 22, 'FontWeight', 'bold')
    end
    legend_line = [devLine errLine]; 
    legendtex = ["estimated 99% envelope", "error"];
%     if idx == 3
%         legend(legend_line,legendtex,'FontSize', 24)    
%     end
    set(gca,'FontSize',20)
    grid on
end
legend1 = legend(legend_line,legendtex);
set(legend1,...
    'Position',[0.415286463909627 0.0269914796199998 0.232109369423706 0.0345525283015656],...
    'Orientation','horizontal',...
    'FontSize',24);
sgtitle('DILIGENT-KIO: Position/Velocity error vs Uncertainty Envelope', ...
    'FontSize', 32, 'FontWeight', 'bold');
end

%% OCEKF
if enable_ocekf
figure
jdx = 1;
title1 = {'error x (m)', 'error y (m)', 'error z (m)'};
title2 = {'error x (m/s)', 'error y (m/s)', 'error z (m/s)'};

for idx = [1, 7, 2, 8, 3, 9]
    subplot(3, 2, jdx)
    devLine = plot(estBaseTime(startIter:endIter), 3*sqrt(squeeze(outMap(ocekfmat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    hold on;
    plot(estBaseTime(startIter:endIter), -3*sqrt(squeeze(outMap(ocekfmat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    if idx == 1 || idx == 2 || idx == 3
        errLine = plot(estBaseTime(startIter:endIter), errors.ocekfL.posError(startIter:endIter, idx), ...
            'b', 'LineWidth', 2);
        ylabel(title1{idx}, 'FontSize', 22, 'FontWeight', 'bold')  
    elseif idx == 7 || idx == 8 || idx == 9
        errLine = plot(estBaseTime(startIter:endIter), errors.ocekfL.velError(startIter:endIter, idx-6), ...
            'b', 'LineWidth', 2);
        ylabel(title2{idx - 6}, 'FontSize', 22, 'FontWeight', 'bold')  
    end
    jdx = jdx+1;

    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 7 || idx == 8 || idx == 9)
        ylim([-yLimVel yLimVel])
    else
        ylim([-yLimPos yLimPos])
    end

    if (idx == 3 || idx == 9)
        xlabel('Time(s)', 'FontSize', 22, 'FontWeight', 'bold')
    end
    legend_line = [devLine errLine]; 
    legendtex = ["estimated 99% envelope", "error"];
%     if idx == 3
%         legend(legend_line,legendtex,'FontSize', 24)    
%     end
    set(gca,'FontSize',20)
    grid on
end
legend1 = legend(legend_line,legendtex);
set(legend1,...
    'Position',[0.415286463909627 0.0269914796199998 0.232109369423706 0.0345525283015656],...
    'Orientation','horizontal',...
    'FontSize',24);
sgtitle('OCEKF: Position/Velocity error vs Uncertainty Envelope', ...
    'FontSize', 32, 'FontWeight', 'bold');
end

%% DILIGENT-KIO-RIE
if enable_diligent_kio_rie 
figure
jdx = 1;
title1 = {'error x (m)', 'error y (m)', 'error z (m)'};
title2 = {'error x (m/s)', 'error y (m/s)', 'error z (m/s)'};

for idx = [1, 7, 2, 8, 3, 9]
    subplot(3, 2, jdx)
    devLine = plot(estBaseTime(startIter:endIter), 3*sqrt(squeeze(outMap(diligent_kio_riemat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    hold on;
    plot(estBaseTime(startIter:endIter), -3*sqrt(squeeze(outMap(diligent_kio_riemat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    if idx == 1 || idx == 2 || idx == 3
        errLine = plot(estBaseTime(startIter:endIter), errors.diligent_kio_rieR.posError(startIter:endIter, idx), ...
            'b', 'LineWidth', 2);
        ylabel(title1{idx}, 'FontSize', 22, 'FontWeight', 'bold')  
    elseif idx == 7 || idx == 8 || idx == 9
        errLine = plot(estBaseTime(startIter:endIter), errors.diligent_kio_rieR.velError(startIter:endIter, idx-6), ...
            'b', 'LineWidth', 2);
        ylabel(title2{idx - 6}, 'FontSize', 22, 'FontWeight', 'bold')  
    end
    jdx = jdx+1;

    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 7 || idx == 8 || idx == 9)
        ylim([-yLimVel yLimVel])
    else
        ylim([-yLimPos yLimPos])
    end

    if (idx == 3 || idx == 9)
        xlabel('Time(s)', 'FontSize', 22, 'FontWeight', 'bold')
    end
    legend_line = [devLine errLine]; 
    legendtex = ["estimated 99% envelope", "error"];
%     if idx == 3
%         legend(legend_line,legendtex,'FontSize', 24)    
%     end  
    set(gca,'FontSize',20)
    grid on
end
legend1 = legend(legend_line,legendtex);
set(legend1,...
    'Position',[0.415286463909627 0.0269914796199998 0.232109369423706 0.0345525283015656],...
    'Orientation','horizontal',...
    'FontSize',24);
sgtitle('DILIGENT-KIO-RIE: Position/Velocity error vs Uncertainty Envelope', ...
    'FontSize', 32, 'FontWeight', 'bold');
end

%% CODILIGENT_KIO
if enable_codiligent_kio
figure
jdx = 1;
title1 = {'error x (m)', 'error y (m)', 'error z (m)'};
title2 = {'error x (m/s)', 'error y (m/s)', 'error z (m/s)'};

for idx = [1, 7, 2, 8, 3, 9]
    subplot(3, 2, jdx)
    devLine = plot(estBaseTime(startIter:endIter), 3*sqrt(squeeze(outMap(codiligent_kiomat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    hold on;
    plot(estBaseTime(startIter:endIter), -3*sqrt(squeeze(outMap(codiligent_kiomat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    if idx == 1 || idx == 2 || idx == 3
        errLine = plot(estBaseTime(startIter:endIter), errors.codiligent_kioL.posError(startIter:endIter, idx), ...
            'b', 'LineWidth', 2);
        ylabel(title1{idx}, 'FontSize', 22, 'FontWeight', 'bold')  
    elseif idx == 7 || idx == 8 || idx == 9
        errLine = plot(estBaseTime(startIter:endIter), errors.codiligent_kioL.velError(startIter:endIter, idx-6), ...
            'b', 'LineWidth', 2);
        ylabel(title2{idx - 6}, 'FontSize', 22, 'FontWeight', 'bold')  
    end
    jdx = jdx+1;

    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 7 || idx == 8 || idx == 9)
        ylim([-yLimVel yLimVel])
    else
        ylim([-yLimPos yLimPos])
    end

    if (idx == 3 || idx == 9)
        xlabel('Time(s)', 'FontSize', 22, 'FontWeight', 'bold')
    end
    legend_line = [devLine errLine]; 
    legendtex = ["estimated 99% envelope", "error"];
%     if idx == 3
%         legend(legend_line,legendtex,'FontSize', 24)    
%     end
    set(gca,'FontSize',20)
    grid on
end
legend1 = legend(legend_line,legendtex);
set(legend1,...
    'Position',[0.415286463909627 0.0269914796199998 0.232109369423706 0.0345525283015656],...
    'Orientation','horizontal',...
    'FontSize',24);
sgtitle('CODILIGENT-KIO: Position/Velocity error vs Uncertainty Envelope', ...
    'FontSize', 32, 'FontWeight', 'bold');
end
%% DILIGENT-KIO-RIE
if enable_codiligent_kio_rie
figure
jdx = 1;
title1 = {'error x (m)', 'error y (m)', 'error z (m)'};
title2 = {'error x (m/s)', 'error y (m/s)', 'error z (m/s)'};

for idx = [1, 7, 2, 8, 3, 9]
    subplot(3, 2, jdx)
    devLine = plot(estBaseTime(startIter:endIter), 3*sqrt(squeeze(outMap(codiligent_kio_riemat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    hold on;
    plot(estBaseTime(startIter:endIter), -3*sqrt(squeeze(outMap(codiligent_kio_riemat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    if idx == 1 || idx == 2 || idx == 3
        errLine = plot(estBaseTime(startIter:endIter), errors.codiligent_kio_rieR.posError(startIter:endIter, idx), ...
            'b', 'LineWidth', 2);
        ylabel(title1{idx}, 'FontSize', 22, 'FontWeight', 'bold')  
    elseif idx == 7 || idx == 8 || idx == 9
        errLine = plot(estBaseTime(startIter:endIter), errors.codiligent_kio_rieR.velError(startIter:endIter, idx-6), ...
            'b', 'LineWidth', 2);
        ylabel(title2{idx - 6}, 'FontSize', 22, 'FontWeight', 'bold')  
    end
    jdx = jdx+1;

    if (idx == 7 || idx == 8 || idx == 9)
        ylim([-yLimVel yLimVel])
    else
        ylim([-yLimPos yLimPos])
    end

    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 3 || idx == 9)
        xlabel('Time(s)', 'FontSize', 22, 'FontWeight', 'bold')
    end
    legend_line = [devLine errLine]; 
    legendtex = ["estimated 99% envelope", "error"];
%     if idx == 3
%         legend(legend_line,legendtex,'FontSize', 24)    
%     end 
    set(gca,'FontSize',20)
    grid on
end
legend1 = legend(legend_line,legendtex);
set(legend1,...
    'Position',[0.415286463909627 0.0269914796199998 0.232109369423706 0.0345525283015656],...
    'Orientation','horizontal',...
    'FontSize',24);
sgtitle('CODILIGENT-KIO-RIE: Position/Velocity error vs Uncertainty Envelope', ...
    'FontSize', 32, 'FontWeight', 'bold');
end

%% InvEKF-F
if enable_invfekf_f
figure
jdx = 1;
title1 = {'error x (m)', 'error y (m)', 'error z (m)'};
title2 = {'error x (m/s)', 'error y (m/s)', 'error z (m/s)'};

for idx = [1, 7, 2, 8, 3, 9]
    subplot(3, 2, jdx)
    devLine = plot(estBaseTime(startIter:endIter), 3*sqrt(squeeze(outMap(invfekf_fmat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    hold on;
    plot(estBaseTime(startIter:endIter), -3*sqrt(squeeze(outMap(invfekf_fmat).PextPoseBase(startIter:endIter, idx, idx))), ...
        '--r', 'LineWidth', 2);
    if idx == 1 || idx == 2 || idx == 3
        errLine = plot(estBaseTime(startIter:endIter), errors.invfekf_fL.posError(startIter:endIter, idx), ...
            'b', 'LineWidth', 2);
        ylabel(title1{idx}, 'FontSize', 22, 'FontWeight', 'bold')  
    elseif idx == 7 || idx == 8 || idx == 9
        errLine = plot(estBaseTime(startIter:endIter), errors.invfekf_fL.velError(startIter:endIter, idx-6), ...
            'b', 'LineWidth', 2);
        ylabel(title2{idx - 6}, 'FontSize', 22, 'FontWeight', 'bold')  
    end
    jdx = jdx+1;

    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 7 || idx == 8 || idx == 9)
        ylim([-yLimVel yLimVel])
    else
        ylim([-yLimPos yLimPos])
    end

    if (idx == 3 || idx == 9)
        xlabel('Time(s)', 'FontSize', 22, 'FontWeight', 'bold')
    end
    legend_line = [devLine errLine]; 
    legendtex = ["estimated 99% envelope", "error"];
%     if idx == 3
%         legend(legend_line,legendtex,'FontSize', 24)    
%     end
    set(gca,'FontSize',20)
    grid on
end
legend1 = legend(legend_line,legendtex);
set(legend1,...
    'Position',[0.415286463909627 0.0269914796199998 0.232109369423706 0.0345525283015656],...
    'Orientation','horizontal',...
    'FontSize',24);
sgtitle('InvEKF-F: Position/Velocity error vs Uncertainty Envelope', ...
    'FontSize', 32, 'FontWeight', 'bold');

end
