col = cell(6,1);
col{1} = colors.ocekf;
col{2} = colors.diligent_kio;
col{3} = colors.diligent_kio_rie; 
col{4} = colors.codiligent_kio; 
col{5} = colors.codiligent_kio_rie; 
col{6} = colors.invfekf_f;

figure
str= {'ATE Pos. [m]','RPE Pos. [m]','ATE Vel. [m/s]'};
X = categorical({'ATE Pos. [m]','RPE Pos. [m]','ATE Vel. [m/s]'});
X = reordercats(X,{'ATE Pos. [m]','RPE Pos. [m]','ATE Vel. [m/s]'});

Y = [errors.ocekfL.ATEpos  errors.diligent_kioL.ATEpos  errors.diligent_kio_rieR.ATEpos  errors.codiligent_kioL.ATEpos  errors.codiligent_kio_rieR.ATEpos  errors.invfekf_fL.ATEpos; 
     errors.ocekfL.RPEpos  errors.diligent_kioL.RPEpos  errors.diligent_kio_rieR.RPEpos  errors.codiligent_kioL.RPEpos  errors.codiligent_kio_rieR.RPEpos  errors.invfekf_fL.RPEpos;
     errors.ocekfL.ATEvel  errors.diligent_kioL.ATEvel  errors.diligent_kio_rieR.ATEvel  errors.codiligent_kioL.ATEvel  errors.codiligent_kio_rieR.ATEvel  errors.invfekf_fL.ATEvel; ];

b = bar(X, Y);
for idx = 1:6
b(idx).FaceColor = col{idx};
end
set(gca, 'XTickLabel', str, 'FontSize',20, 'FontWeight', 'Bold')
set(gca, 'GridAlpha', 0.5, 'LineWidth', 1.5)
grid on
grid minor
legend('OCEKF', 'DILIGENT-KIO', 'DILIGENT-KIO-RIE', 'CODILIGENT-KIO', 'CODILIGENT-KIO-RIE', 'InvEKF-F')

%%
figure
X1 = categorical({'ATE Rot. [deg] ', 'RPE Rot. [deg]'});
X1 = reordercats(X1,{'ATE Rot. [deg] ', 'RPE Rot. [deg]'});
Y1 = [errors.ocekfL.ATErot errors.diligent_kioL.ATErot  errors.diligent_kio_rieR.ATErot errors.codiligent_kioL.ATErot errors.codiligent_kio_rieR.ATErot errors.invfekf_fL.ATErot;
      errors.ocekfL.RPErot errors.diligent_kioL.RPErot  errors.diligent_kio_rieR.RPErot errors.codiligent_kioL.RPErot errors.codiligent_kio_rieR.RPErot errors.invfekf_fL.RPErot];
b1 = bar(X1,Y1);

for idx = 1:6
b1(idx).FaceColor = col{idx};
end
set(gca, 'FontSize',20, 'FontWeight', 'Bold')
set(gca, 'GridAlpha', 0.5, 'LineWidth', 1.5)
grid on
grid minor
legend('OCEKF', 'DILIGENT-KIO', 'DILIGENT-KIO-RIE', 'CODILIGENT-KIO', 'CODILIGENT-KIO-RIE', 'InvEKF-F')

%%
D1 = [errors.ocekfL.ATEvel errors.ocekfL.ATEpos errors.ocekfL.RPEpos errors.ocekfL.RPErot errors.ocekfL.ATErot];
D2 = [errors.diligent_kioL.ATEvel errors.diligent_kioL.ATEpos errors.diligent_kioL.RPEpos errors.diligent_kioL.RPErot errors.diligent_kioL.ATErot];
D3 = [errors.diligent_kio_rieR.ATEvel errors.diligent_kio_rieR.ATEpos errors.diligent_kio_rieR.RPEpos errors.diligent_kio_rieR.RPErot errors.diligent_kio_rieR.ATErot];
D4 = [errors.codiligent_kioL.ATEvel errors.codiligent_kioL.ATEpos errors.codiligent_kioL.RPEpos errors.codiligent_kioL.RPErot errors.codiligent_kioL.ATErot];
D5 = [errors.codiligent_kio_rieR.ATEvel errors.codiligent_kio_rieR.ATEpos errors.codiligent_kio_rieR.RPEpos errors.codiligent_kio_rieR.RPErot  errors.codiligent_kio_rieR.ATErot];
D6 = [errors.invfekf_fL.ATEvel errors.invfekf_fL.ATEpos errors.invfekf_fL.RPEpos errors.invfekf_fL.RPErot errors.invfekf_fL.ATErot];
figure
P = [D1; D2; D3; D4; D5; D6];

% Spider plot
s = spider_plot_class(P);
% s.AxesLimits = [1, 1, 1, 1, 1; 10, 10, 10, 10, 10];
s.AxesDisplay = 'all';
s.AxesLabels = {'ATE Velocity [m/s]', 'ATE Position [m]', 'RPE Position [m]', 'RPE Rotation [deg]', 'ATE Rotation [deg]'};
% s.AxesTickLabels = {'data', 'data', 'data', }
% s.AxesLabelsOffset = 0.1;
% s.AxesDataOffset = 2;
s.AxesPrecision = 3;
s.Color = [colors.ocekf; ...
    colors.diligent_kio; ...
    colors.diligent_kio_rie; ...
    colors.codiligent_kio; ...
    colors.codiligent_kio_rie; ...
    colors.invfekf_f];
s.AxesInterval = 1;
s.AxesFontSize = 60;
s.LabelFontSize = 60;
s.LineWidth = [6 6 6 6 6 6];
s.LineStyle = {'-.' , '--', '-.', '--', '-.', ':'};
s.AxesLimits = [min(Y(3, :)), min(Y(1, :)), min(Y(2, :)), min(Y1(2, :)), min(Y1(1, :)); max(Y(3, :)), max(Y(1, :)), max(Y(2, :)), max(Y1(2, :)), max(Y1(1, :))];
s.FillOption = 'on'; 
s.FillTransparency = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05];
% s.FillTransparency = [0.2, 0.1, 0.1];
s.AxesScaling =  {'linear', 'linear', 'linear', 'linear', 'linear'};

% Legend properties
s.LegendLabels = {'OCEKF', 'DILIGENT-KIO', 'DILIGENT-KIO-RIE', 'CODILIGENT-KIO', 'CODILIGENT-KIO-RIE', 'InvEKF-F'};
s.LegendHandle.Location = 'northeastoutside';
s.LegendHandle.FontSize = 60;
s.MinorGrid = 'on';
