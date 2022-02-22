%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
% distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% clear workspace
clc
clear 
close all

%% flags
enable_ocekf = true;
enable_diligent_kio = true;
enable_diligent_kio_rie = true;
enable_codiligent_kio = true;
enable_codiligent_kio_rie = true;
enable_invfekf_f = true;

%% colors
colors.ocekf = [0.9922    0.6824    0.1137];
colors.diligent_kio = [0    1    0];
colors.diligent_kio_rie = [0.2745    0.4275    0.1176];
colors.codiligent_kio = [0.3843    0.7725    0.8549];
colors.codiligent_kio_rie = [0.0745    0.2196    0.7412];
colors.invfekf_f = [182/255,96/255,205/255];

%% load dataset
experiment_name = 'com-sinusoid'; % walking | com-sinusoid 
experiment.mat_file = ['./resources/iCubGenova04/' experiment_name '.mat'];

load(experiment.mat_file);
experiment.robot_name='iCubGenova04'; %% Name of the robot
experiment.model_path = './resources/iCubGenova04/';  %% Path to the robot model
experiment.model_file = 'model.urdf';

experiment.joints_list = ["neck_pitch", "neck_roll", "neck_yaw", ...
                          "torso_pitch", "torso_roll", "torso_yaw", ...
                          "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", ...
                          "l_elbow", ...
                          "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", ...
                          "r_elbow", ...
                          "l_hip_pitch", "l_hip_roll", "l_hip_yaw", ...
                          "l_knee", "l_ankle_pitch", "l_ankle_roll", ...
                          "r_hip_pitch", "r_hip_roll", "r_hip_yaw", ...
                          "r_knee", "r_ankle_pitch", "r_ankle_roll"];
experiment.root_link = 'root_link'; 
%% Timing iterators
estimatorJointsTime = estimatorJointsTime - estimatorJointsTime(1);
dt = mean(diff(estimatorJointsTime));

startIter = 1;
endIter = length(estimatorJointsTime);
nrIters = endIter - startIter + 1;

%% load iDynTree KinDynComputations
experiment.kindyn_debug = false;
KinDynModel = iDynTreeWrappers.loadReducedModel(experiment.joints_list, ...
                                                experiment.root_link, ...
                                                experiment.model_path, ...
                                                experiment.model_file, ...
                                                experiment.kindyn_debug);
kinDyn = KinDynModel.kinDynComp;

%% Load ModelComputations
lfootLink = 'l_foot';
rfootLink = 'r_foot';
baseLink = 'root_link';
baseLinkIMU = 'root_link_imu_acc';
lfVertexNames = {'l_sole'};
rfVertexNames = {'r_sole'};

LFVertexIDs = -ones(length(lfVertexNames), 1);
for idx = 1:length(lfVertexNames)
    LFVertexIDs(idx) = kinDyn.model().getFrameIndex(lfVertexNames{idx});
end

RFVertexIDs = -ones(length(lfVertexNames), 1);
for idx = 1:length(lfVertexNames)
    RFVertexIDs(idx) = kinDyn.model().getFrameIndex(rfVertexNames{idx});
end

LSoleIds = [LFVertexIDs(1)];
RSoleIds = [RFVertexIDs(1)];
SoleIds = [LSoleIds; RSoleIds];

modelComp = Model.ModelComputations(kinDyn, baseLink, baseLinkIMU, ...
                                    lfootLink, rfootLink, ...
                                    LFVertexIDs, RFVertexIDs);

%% Load Estimator options
options = Estimation.Proprioception.EstimatorOptions();
options.nr_joints_est = nr_joints_est;
options.enable_bias_estimation = true;
options.debug_mode = true;

%% Setup initial states
rpy0 = linkBaseRot(startIter ,:);
R0 = Utils.rpy2rot(rpy0(1), rpy0(2), rpy0(3));
pos0 = linkBasePos(startIter ,:)';
pose0 = LieGroups.SE3.constructSE3(R0, pos0);

s = estimatorJointsPos(1, :)';
sDot = zeros(size(estimatorJointsPos(1, :)'));
modelComp.setRobotState(pose0, zeros(6, 1), s, sDot);

b_H_w0 = modelComp.kindyn.getWorldBaseTransform().inverse();
pW0 = b_H_w0.getPosition().toMatlab();
qW0 = b_H_w0.getRotation().asQuaternion().toMatlab();

w_H_IMU0 = modelComp.kindyn.getWorldTransform(modelComp.base_link_imu_idx);
pIMU0 = w_H_IMU0.getPosition().toMatlab();
qIMU0 = w_H_IMU0.getRotation().asQuaternion().toMatlab();

w_H_LF0 = modelComp.kindyn.getWorldTransform(LFVertexIDs(1));
pLF0 = w_H_LF0.getPosition().toMatlab();
qLF0 = w_H_LF0.getRotation().asQuaternion().toMatlab();
RotLF0 = w_H_LF0.getRotation().toMatlab();

w_H_RF0 = modelComp.kindyn.getWorldTransform(RFVertexIDs(1));
pRF0 = w_H_RF0.getPosition().toMatlab();
qRF0 = w_H_RF0.getRotation().asQuaternion().toMatlab();
RotRF0 = w_H_RF0.getRotation().toMatlab();

Nl = length(LFVertexIDs);
Nr = length(RFVertexIDs);

for idx = 1:Nl
    frameName = modelComp.kindyn.model.getFrameName(LFVertexIDs(idx));
    dl = modelComp.kindyn.getWorldTransform(frameName).getPosition().toMatlab();
end

for idx = 1:Nr
    frameName = modelComp.kindyn.model.getFrameName(RFVertexIDs(idx));
    dr = modelComp.kindyn.getWorldTransform(frameName).getPosition().toMatlab();
end

ba0 = zeros(3, 1);
bg0 = zeros(3, 1);

if (options.enable_bias_estimation)
    initial_state = zeros(30, 1);
else
    initial_state = zeros(24, 1);
end
initial_v = zeros(3, 1);
initial_state(1:4) = qIMU0; % imu orientation
initial_state(5:7) = pIMU0; % imu position
initial_state(8:10) = initial_v; % imu linear velocity
initial_state(11:14) = qLF0;  % left foot orientation
initial_state(15:17) = pLF0; % left foot position
initial_state(18:21) = qRF0;  % right foot orientation
initial_state(22:24) = pRF0; % right foot position


if (options.enable_bias_estimation)
    initial_state(25:27) = ba0; % acc bias
    initial_state(28:30) = bg0;  % gyro bias
end
    
initial_diligent_kio_state = Estimation.DILIGENT_KIO.State.construct(w_H_IMU0.getRotation().toMatlab(), initial_state(5:7), initial_v, ...
        RotLF0, initial_state(15:17), ...
        RotRF0, initial_state(22:24), ...
        ba0, bg0, options.enable_bias_estimation);
    
[initial_invfekf_f_state, initial_theta] = Estimation.InvEKF_F.State.construct(w_H_IMU0.getRotation().toMatlab(), initial_v, initial_state(5:7), ...
        initial_state(22:24), RotRF0(1:3, 1), RotRF0(1:3, 3), ...
        initial_state(15:17), RotLF0(1:3, 1), RotLF0(1:3, 3), ...
        bg0, ba0);

%% setup priors
priors = Config.iCubGenova04.configMatlabPriors();

%% setup sensor noise
sensors_dev = Config.iCubGenova04.configMatlabSensorDev(options.nr_joints_est);

%% setup schmitt trigger
[leftSchmittParams, rightSchmittParams] = Config.iCubGenova04.configSchmittParams;
leftFootContactStateMachine = Estimation.ContactHandler.ContactStateMachine(leftSchmittParams);
rightFootContactStateMachine = Estimation.ContactHandler.ContactStateMachine(rightSchmittParams);

%% Estimator collectors - collect outputs for plotting
outMap = containers.Map('KeyType','char','ValueType','any');

gt = 'gt';
outMap(gt) = EstCollector(gt);
outMap(gt).resizeBuffers(nrIters, Nl, Nr);

if enable_ocekf
    ocekfmat = 'ocekfmat';
    outMap(ocekfmat) = EstCollector(ocekfmat);
    outMap(ocekfmat).resizeBuffers(nrIters, Nl, Nr);
end

if enable_diligent_kio
    diligent_kiomat = 'diligent_kiomat';
    outMap(diligent_kiomat) = EstCollector(diligent_kiomat);
    outMap(diligent_kiomat).resizeBuffers(nrIters, Nl, Nr);
end


if enable_diligent_kio_rie
    diligent_kio_riemat = 'diligent_kio_riemat';
    outMap(diligent_kio_riemat) = EstCollector(diligent_kio_riemat);
    outMap(diligent_kio_riemat).resizeBuffers(nrIters, Nl, Nr);
end

if enable_codiligent_kio
    codiligent_kiomat = 'codiligent_kiomat';
    outMap(codiligent_kiomat) = EstCollector(codiligent_kiomat);
    outMap(codiligent_kiomat).resizeBuffers(nrIters, Nl, Nr);
end

if enable_codiligent_kio_rie
    codiligent_kio_riemat = 'codiligent_kio_riemat';
    outMap(codiligent_kio_riemat) = EstCollector(codiligent_kio_riemat);
    outMap(codiligent_kio_riemat).resizeBuffers(nrIters, Nl, Nr);
end

if enable_invfekf_f
    invfekf_fmat = 'invfekf_fmat';
    outMap(invfekf_fmat) = EstCollector(invfekf_fmat);
    outMap(invfekf_fmat).resizeBuffers(nrIters, Nl, Nr);
end

%% Initialize filters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ocekf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enable_ocekf
    ocekf = Estimation.OCEKF.Filter();
    ocekf.setup(priors, sensors_dev, modelComp, options);
    ocekf.initialize(initial_state, ba0, bg0);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% diligent_kio
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enable_diligent_kio
    diligent_kiom = Estimation.DILIGENT_KIO.Filter();
    diligent_kiom.setup(priors, sensors_dev, modelComp, options);
    diligent_kiom.initialize(initial_diligent_kio_state);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% diligent_kio_rie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enable_diligent_kio_rie
    diligent_kio_riem = Estimation.DILIGENT_KIO_RIE.Filter();
    diligent_kio_riem.setup(priors, sensors_dev, modelComp, options);
    diligent_kio_riem.initialize(initial_diligent_kio_state);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% codiligent_kio
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enable_codiligent_kio
    codiligent_kiom = Estimation.CODILIGENT_KIO.Filter();
    codiligent_kiom.setup(priors, sensors_dev, modelComp, options);
    codiligent_kiom.initialize(initial_diligent_kio_state);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% codiligent_kio_rie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enable_codiligent_kio_rie
    codiligent_kio_riem = Estimation.CODILIGENT_KIO_RIE.Filter();
    codiligent_kio_riem.setup(priors, sensors_dev, modelComp, options);
    codiligent_kio_riem.initialize(initial_diligent_kio_state);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% invfekf_f
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enable_invfekf_f
    invfekf_fm = Estimation.InvEKF_F.Filter();
    invfekf_fm.setup(priors, sensors_dev, modelComp, options);
    invfekf_fm.initialize(initial_invfekf_f_state, initial_theta);
end

%% init buffers
lfC = zeros(length(estimatorJointsTime), 1);
rfC = zeros(length(estimatorJointsTime), 1);

jPosDyn = iDynTree.JointPosDoubleArray(modelComp.kindyn.model);
XPrev = [];


%% Run filters (main loop)
for iter = startIter : endIter
    t = estimatorJointsTime(iter);
    s = estimatorJointsPos(iter, :)';
    sDot = estimatorJointsVel(iter, :)';
    fzl = lfForceZ(iter);
    fzr = rfForceZ(iter);
    acc = imuAcc(iter, :);
    gyro = imuOmega(iter, :);
       
    % update contact states using Schmitt Trigger
    leftFootContactStateMachine.contactMeasurementUpdate(fzl, t);
    rightFootContactStateMachine.contactMeasurementUpdate(fzr, t);
    contacts = [leftFootContactStateMachine.contactState(); ...
        rightFootContactStateMachine.contactState()];            
            
    lfC(iter) = contacts(1);
    rfC(iter) = contacts(2);    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ocekf
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if enable_ocekf
        [Xocekf, ~, ocekfBase, ocekfDebug] = ocekf.advance(t, gyro', acc', s, contacts);
        w_H_B_ocekf = Utils.idynPose(Utils.quat2rot(ocekfBase.q), ocekfBase.I_p);
        outMap(ocekfmat).updateBasePose(iter, w_H_B_ocekf.getRotation().asRPY().toMatlab(), ...
            ocekfBase.I_p, ocekfBase.I_pdot, ocekfBase.I_omega);
        [~,~,~, ...
            qLF_ocekf,p_LF_ocekf, ...
            qRF_ocekf,p_RF_ocekf, ...
            ba_ocekf, bg_ocekf] = Estimation.OCEKF.State.extract(Xocekf, ba0, bg0);
        outMap(ocekfmat).updateFootRotation('left',iter, Utils.rot2rpy(Utils.quat2rot(qLF_ocekf)));
        outMap(ocekfmat).updateFootRotation('right',iter, Utils.rot2rpy(Utils.quat2rot(qRF_ocekf)));
        outMap(ocekfmat).updateFootPosition('left',iter, 1, p_LF_ocekf);
        outMap(ocekfmat).updateFootPosition('right',iter, 1, p_RF_ocekf);
        outMap(ocekfmat).updateBias(iter, ba_ocekf, bg_ocekf);
        outMap(ocekfmat).updateCovPoseBase(iter, ocekfDebug.PextPoseBase);
    end
            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % diligent_kiomat
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if enable_diligent_kio
        [Xdil, Pk, diligent_kiomBase, diligent_kioDebug] = diligent_kiom.advance(t, gyro', acc', s, contacts);
        w_H_B_diligent_kiom = Utils.idynPose(Utils.quat2rot(diligent_kiomBase.q), diligent_kiomBase.I_p);
        outMap(diligent_kiomat).updateBasePose(iter, w_H_B_diligent_kiom.getRotation().asRPY().toMatlab(), ...
            diligent_kiomBase.I_p, diligent_kiomBase.I_pdot, diligent_kiomBase.I_omega);
        [~, ~, ~, ...
            Rdil_LF, pdil_LF, ...
            Rdil_RF, pdil_RF, ...
            bias_acc_dil, bias_gyro_dil] = Estimation.DILIGENT_KIO.State.extract(Xdil);
        outMap(diligent_kiomat).updateFootRotation('left',iter, Utils.rot2rpy(Rdil_LF));
        outMap(diligent_kiomat).updateFootRotation('right',iter, Utils.rot2rpy(Rdil_RF));
        outMap(diligent_kiomat).updateFootPosition('left',iter, 1, pdil_LF);
        outMap(diligent_kiomat).updateFootPosition('right',iter, 1, pdil_RF);
        outMap(diligent_kiomat).updateBias(iter, bias_acc_dil, bias_acc_dil);
        outMap(diligent_kiomat).updateCovPoseBase(iter, diligent_kioDebug.PextPoseBase);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % diligent_kio_riemat
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if enable_diligent_kio_rie
        [Xdilrie, Pkrie, diligent_kio_riemBase, diligent_kio_rieDebug] = diligent_kio_riem.advance(t, gyro', acc', s, contacts);
        w_H_B_diligent_kio_riem = Utils.idynPose(Utils.quat2rot(diligent_kio_riemBase.q), diligent_kio_riemBase.I_p);
        outMap(diligent_kio_riemat).updateBasePose(iter, w_H_B_diligent_kio_riem.getRotation().asRPY().toMatlab(), ...
            diligent_kio_riemBase.I_p, diligent_kio_riemBase.I_pdot, diligent_kio_riemBase.I_omega);
        [~, ~, ~, ...
            Rdilrie_LF, pdilrie_LF, ...
            Rdilrie_RF, pdilrie_RF, ...
            bias_acc_dilrie, bias_gyro_dilrie] = Estimation.DILIGENT_KIO_RIE.State.extract(Xdilrie);
        outMap(diligent_kio_riemat).updateFootRotation('left',iter, Utils.rot2rpy(Rdilrie_LF));
        outMap(diligent_kio_riemat).updateFootRotation('right',iter, Utils.rot2rpy(Rdilrie_RF));
        outMap(diligent_kio_riemat).updateFootPosition('left',iter, 1, pdilrie_LF);
        outMap(diligent_kio_riemat).updateFootPosition('right',iter, 1, pdilrie_RF);
        outMap(diligent_kio_riemat).updateBias(iter, bias_acc_dilrie, bias_acc_dilrie);
        outMap(diligent_kio_riemat).updateCovPoseBase(iter, diligent_kio_rieDebug.PextPoseBase);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % codiligent_kiomat
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if enable_codiligent_kio
        [Xcdlie, Pkcdlie, codiligent_kiomBase, codiligent_kioDebug] = codiligent_kiom.advance(t, gyro', acc', s, contacts);
        w_H_B_cdliem = Utils.idynPose(Utils.quat2rot(codiligent_kiomBase.q), codiligent_kiomBase.I_p);
        outMap(codiligent_kiomat).updateBasePose(iter, w_H_B_cdliem.getRotation().asRPY().toMatlab(), ...
            codiligent_kiomBase.I_p, codiligent_kiomBase.I_pdot, codiligent_kiomBase.I_omega);
        [~, ~, ~, ...
            Rcdlie_LF, pcdlie_LF, ...
            Rcdlie_RF, pcdlie_RF, ...
            bias_acc_cdlie, bias_gyro_cdlie] = Estimation.CODILIGENT_KIO.State.extract(Xcdlie);
        outMap(codiligent_kiomat).updateFootRotation('left',iter, Utils.rot2rpy(Rcdlie_LF));
        outMap(codiligent_kiomat).updateFootRotation('right',iter, Utils.rot2rpy(Rcdlie_RF));
        outMap(codiligent_kiomat).updateFootPosition('left',iter, 1, pcdlie_LF);
        outMap(codiligent_kiomat).updateFootPosition('right',iter, 1, pcdlie_RF);
        outMap(codiligent_kiomat).updateBias(iter, bias_acc_cdlie, bias_gyro_cdlie);
        outMap(codiligent_kiomat).updateCovPoseBase(iter, codiligent_kioDebug.PextPoseBase);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % codiligent_kio_riemat
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if enable_codiligent_kio_rie
        [Xcdrie, Pkcdrie, codiligent_kio_riemBase, codiligent_kio_rieDebug] = codiligent_kio_riem.advance(t, gyro', acc', s, contacts);
        w_H_B_cdriem = Utils.idynPose(Utils.quat2rot(codiligent_kio_riemBase.q), codiligent_kio_riemBase.I_p);
        outMap(codiligent_kio_riemat).updateBasePose(iter, w_H_B_cdriem.getRotation().asRPY().toMatlab(), ...
            codiligent_kio_riemBase.I_p, codiligent_kio_riemBase.I_pdot, codiligent_kio_riemBase.I_omega);
        [~, ~, ~, ...
            Rcdrie_LF, pcdrie_LF, ...
            Rcdrie_RF, pcdrie_RF, ...
            bias_acc_cdrie, bias_gyro_cdrie] = Estimation.CODILIGENT_KIO_RIE.State.extract(Xcdrie);
        outMap(codiligent_kio_riemat).updateFootRotation('left',iter, Utils.rot2rpy(Rcdrie_LF));
        outMap(codiligent_kio_riemat).updateFootRotation('right',iter, Utils.rot2rpy(Rcdrie_RF));
        outMap(codiligent_kio_riemat).updateFootPosition('left',iter, 1, pcdrie_LF);
        outMap(codiligent_kio_riemat).updateFootPosition('right',iter, 1, pcdrie_RF);
        outMap(codiligent_kio_riemat).updateBias(iter, bias_acc_cdrie, bias_gyro_cdrie);
        outMap(codiligent_kio_riemat).updateCovPoseBase(iter, codiligent_kio_rieDebug.PextPoseBase);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % invfekf_fmat
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if enable_invfekf_f
        [Xinvfekf_f, thetainvfekf_f, ~, invfekf_fmBase, invfekf_fDebug] = invfekf_fm.advance(t, gyro', acc', s, contacts);
        w_H_B_invfekf_fm = Utils.idynPose(Utils.quat2rot(invfekf_fmBase.q), invfekf_fmBase.I_p);
        outMap(invfekf_fmat).updateBasePose(iter, w_H_B_invfekf_fm.getRotation().asRPY().toMatlab(), ...
            invfekf_fmBase.I_p, invfekf_fmBase.I_pdot, invfekf_fmBase.I_omega);
        [~, ~, ~, ...
         p_RF_invfekf_f, x_RF_invfekf_f, z_RF_invfekf_f, ...
         p_LF_invfekf_f, x_LF_invfekf_f, z_LF_invfekf_f, ...
         ba_invfekf_f, bg_invfekf_f] = Estimation.InvEKF_F.State.extract(Xinvfekf_f, thetainvfekf_f);
        
        Rinvfekf_f_LF = [x_LF_invfekf_f Utils.skew(x_LF_invfekf_f)*z_LF_invfekf_f z_LF_invfekf_f];
        Rinvfekf_f_RF = [x_RF_invfekf_f Utils.skew(x_RF_invfekf_f)*z_RF_invfekf_f z_RF_invfekf_f];

        outMap(invfekf_fmat).updateFootRotation('left',iter, Utils.rot2rpy(Rinvfekf_f_LF));
        outMap(invfekf_fmat).updateFootRotation('right',iter, Utils.rot2rpy(Rinvfekf_f_RF));
        outMap(invfekf_fmat).updateFootPosition('left',iter, 1, p_LF_invfekf_f);
        outMap(invfekf_fmat).updateFootPosition('right',iter, 1, p_RF_invfekf_f);
        outMap(invfekf_fmat).updateBias(iter, ba_invfekf_f, bg_invfekf_f);
        outMap(invfekf_fmat).updateCovPoseBase(iter, invfekf_fDebug.PextPoseBase);
    end
   
%     pause
    if (mod(iter, 50) == 0)
        disp(['========> Iter: ' num2str(iter) '/' num2str(endIter-startIter)])
    end
end

%% Plot base rotation and position
estBaseTime = estimatorJointsTime;

figure
color = {'r', 'g', 'b'};
color2 = {'g', 'b', 'r'};

title1 = {'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'};
title2 = {'x (m)', 'y (m)', 'z (m)'};
for idx = 1:2
    subplot(2, 1, idx)
    simp = plot(estBaseTime(startIter:endIter), rad2deg(linkBaseRot(startIter:endIter, idx)), 'LineWidth', 4, 'Color', 'k', 'LineStyle', '--');
    legend_line = [];
    legendtex = {};
    legend_line = [legend_line simp];
    legendtex = [legendtex 'Vicon'];
    hold on
    if enable_ocekf
        ocekfmatline = plot(estBaseTime(startIter:endIter), rad2deg(outMap(ocekfmat).baseRPY(startIter:endIter, idx)),  'Color',colors.ocekf, 'LineWidth', 4, 'LineStyle', '-.');
        legend_line = [legend_line ocekfmatline];
        legendtex = [legendtex 'OCEKF'];
        
    end
    if enable_diligent_kio
        diligent_kiomatline = plot(estBaseTime(startIter:endIter), rad2deg(outMap(diligent_kiomat).baseRPY(startIter:endIter, idx)),  'Color',colors.diligent_kio, 'LineWidth', 4, 'LineStyle', '--');    
        legend_line = [legend_line diligent_kiomatline];
        legendtex = [legendtex 'DILIGENT-KIO'];
    end
    if enable_diligent_kio_rie
        diligent_kio_riematline = plot(estBaseTime(startIter:endIter), rad2deg(outMap(diligent_kio_riemat).baseRPY(startIter:endIter, idx)),  'Color',  colors.diligent_kio_rie, 'LineWidth', 4, 'LineStyle', '-.');    
        legend_line = [legend_line diligent_kio_riematline];
        legendtex = [legendtex 'DILIGENT-KIO-RIE'];
    end
    if enable_codiligent_kio
        codiligent_kiomatline = plot(estBaseTime(startIter:endIter), rad2deg(outMap(codiligent_kiomat).baseRPY(startIter:endIter, idx)), 'Color',  colors.codiligent_kio, 'LineWidth', 4, 'LineStyle', '--');    
        legend_line = [legend_line codiligent_kiomatline];
        legendtex = [legendtex 'CODILIGENT-KIO'];
    end
    if enable_codiligent_kio_rie
        codiligent_kio_riematline = plot(estBaseTime(startIter:endIter), rad2deg(outMap(codiligent_kio_riemat).baseRPY(startIter:endIter, idx)),  'Color',  colors.codiligent_kio_rie, 'LineWidth', 4, 'LineStyle', '-.');    
        legend_line = [legend_line codiligent_kio_riematline];
        legendtex = [legendtex 'CODILIGENT-KIO-RIE'];
    end
    if enable_invfekf_f
        invfekf_fmatline = plot(estBaseTime(startIter:endIter), rad2deg(outMap(invfekf_fmat).baseRPY(startIter:endIter, idx)), 'Color', colors.invfekf_f, 'LineWidth', 4, 'LineStyle', ':');
        legend_line = [legend_line invfekf_fmatline];
        legendtex = [legendtex 'InvEKF-F'];
    end
    ylabel(title1{idx}, 'FontSize', 24)    
    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 1)
    legend(legend_line,legendtex,'FontSize', 24, 'Orientation', 'Horizontal')    
    end
    if (idx == 2)
        xlabel('Time(s)', 'FontSize', 18)
    end
    set(gca,'FontSize',20, 'FontWeight', 'Bold')
    set(gca, 'GridAlpha', 0.5, 'LineWidth', 1.5)
    grid on
end
sgtitle('Base Orientation: Roll and Pitch', ...
    'FontSize', 24);
drawnow;

figure
for idx = 1:3
    subplot(3, 1, idx)
    simp = plot(estBaseTime(startIter:endIter), linkBasePos(startIter:endIter, idx), 'LineWidth', 4, 'Color', 'k', 'LineStyle', '--');
    legend_line = [];
    legendtex = {};
    legend_line = [legend_line simp];
    legendtex = [legendtex 'Vicon'];
    hold on
    if enable_ocekf
        ocekfmatline = plot(estBaseTime(startIter:endIter), outMap(ocekfmat).basePos(startIter:endIter, idx), 'Color',colors.ocekf, 'LineWidth', 4, 'LineStyle', '-.');
        legend_line = [legend_line ocekfmatline];
        legendtex = [legendtex 'OCEKF'];
    end
    if enable_diligent_kio
        diligent_kiomatline = plot(estBaseTime(startIter:endIter), outMap(diligent_kiomat).basePos(startIter:endIter, idx), 'Color',   colors.diligent_kio, 'LineWidth', 4, 'LineStyle', '--');
        legend_line = [legend_line diligent_kiomatline];
        legendtex = [legendtex 'DILIGENT-KIO'];
    end
    if enable_diligent_kio_rie
        diligent_kio_riematline = plot(estBaseTime(startIter:endIter), outMap(diligent_kio_riemat).basePos(startIter:endIter, idx),  'Color',  colors.diligent_kio_rie , 'LineWidth', 4, 'LineStyle', '-.');
        legend_line = [legend_line diligent_kio_riematline];
        legendtex = [legendtex 'DILIGENT-KIO-RIE'];
    end
    if enable_codiligent_kio
        codiligent_kiomatline = plot(estBaseTime(startIter:endIter), outMap(codiligent_kiomat).basePos(startIter:endIter, idx),  'Color',  colors.codiligent_kio, 'LineWidth', 4, 'LineStyle', '--');    
        legend_line = [legend_line codiligent_kiomatline];
        legendtex = [legendtex 'CODILIGENT-KIO'];
    end
    if enable_codiligent_kio_rie
        codiligent_kio_riematline = plot(estBaseTime(startIter:endIter), outMap(codiligent_kio_riemat).basePos(startIter:endIter, idx),  'Color',  colors.codiligent_kio_rie, 'LineWidth', 4, 'LineStyle', '-.');    
        legend_line = [legend_line codiligent_kio_riematline];
        legendtex = [legendtex 'CODILIGENT-KIO-RIE'];
    end
    if enable_invfekf_f
        invfekf_fmatline = plot(estBaseTime(startIter:endIter), outMap(invfekf_fmat).basePos(startIter:endIter, idx), 'Color',colors.invfekf_f, 'LineWidth', 4, 'LineStyle', ':');
        legend_line = [legend_line invfekf_fmatline];
        legendtex = [legendtex 'InvEKF-F'];
    end
    
    ylabel(title2{idx}, 'FontSize', 24)    
    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 1)
    legend(legend_line,legendtex,'FontSize', 24, 'Orientation', 'Horizontal')    
    end
    if (idx == 3)
        xlabel('Time(s)', 'FontSize', 18)
    end
    set(gca,'FontSize',20, 'FontWeight', 'Bold')
    set(gca, 'GridAlpha', 0.5, 'LineWidth', 1.5)
    grid on
end
sgtitle('Base Position', ...
    'FontSize', 24);
drawnow;

%% Compute Vicon linear velocity by numerical differentiatio
linkBaseLinVel = [];
for idx = 1:3    
    linkBaseLinVel(:, idx) = [0; smooth(diff(linkBasePos(:, idx))/dt, 'sgolay',  1)];
end

%% Plot Base Linear Velocity
title2 = {'v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)'};
figure
for idx = 1:3
    subplot(3, 1, idx)
    simp = plot(estBaseTime(startIter:endIter), linkBaseLinVel(startIter:endIter, idx), 'LineWidth', 4, 'Color', 'k', 'LineStyle', '--');
    legend_line = [];
    legendtex = {};
    legend_line = [legend_line simp];
    legendtex = [legendtex 'Vicon'];
    hold on
    if enable_ocekf
        ocekfmatline = plot(estBaseTime(startIter:endIter), outMap(ocekfmat).baseLinVel(startIter:endIter, idx), 'Color',colors.ocekf, 'LineWidth', 4, 'LineStyle', '-.');
        legend_line = [legend_line ocekfmatline];
        legendtex = [legendtex 'OCEKF'];
    end
    if enable_diligent_kio
        diligent_kiomatline = plot(estBaseTime(startIter:endIter), outMap(diligent_kiomat).baseLinVel(startIter:endIter, idx), 'Color',  colors.diligent_kio, 'LineWidth', 4, 'LineStyle', '--');
        legend_line = [legend_line diligent_kiomatline];
        legendtex = [legendtex 'DILIGENT-KIO'];
    end
    if enable_diligent_kio_rie
        diligent_kio_riematline = plot(estBaseTime(startIter:endIter), outMap(diligent_kio_riemat).baseLinVel(startIter:endIter, idx),   'Color',  colors.diligent_kio_rie, 'LineWidth', 4, 'LineStyle', '-.');
        legend_line = [legend_line diligent_kio_riematline];
        legendtex = [legendtex 'DILIGENT-KIO-RIE'];
    end
    if enable_codiligent_kio
        codiligent_kiomatline = plot(estBaseTime(startIter:endIter), outMap(codiligent_kiomat).baseLinVel(startIter:endIter, idx),  'Color',  colors.codiligent_kio, 'LineWidth', 4, 'LineStyle', '--');    
        legend_line = [legend_line codiligent_kiomatline];
        legendtex = [legendtex 'CODILIGENT-KIO'];
    end
    if enable_codiligent_kio_rie
        codiligent_kio_riematline = plot(estBaseTime(startIter:endIter), outMap(codiligent_kio_riemat).baseLinVel(startIter:endIter, idx), 'Color',  colors.codiligent_kio_rie, 'LineWidth', 4, 'LineStyle', '-.');    
        legend_line = [legend_line codiligent_kio_riematline];
        legendtex = [legendtex 'CODILIGENT-KIO-RIE'];
    end
    if enable_invfekf_f
        invfekf_fmatline = plot(estBaseTime(startIter:endIter), outMap(invfekf_fmat).baseLinVel(startIter:endIter, idx), 'Color', colors.invfekf_f, 'LineWidth', 4, 'LineStyle', ':');
        legend_line = [legend_line invfekf_fmatline];
        legendtex = [legendtex 'InvEKF-F'];
    end
    
    
    ylabel(title2{idx}, 'FontSize', 24)   
    xlim([estBaseTime(startIter) estBaseTime(endIter)])
    if (idx == 1)
    legend(legend_line,legendtex,'FontSize', 24, 'Orientation', 'Horizontal')    
    end
    if (idx == 3)
        xlabel('Time(s)', 'FontSize', 18)
    end
    set(gca,'FontSize',20, 'FontWeight', 'Bold')
    set(gca, 'GridAlpha', 0.5, 'LineWidth', 1.5)
    grid on
end
sgtitle('Base Linear Velocity', ...
    'FontSize', 32, 'FontWeight', 'Bold');
drawnow;

%% plot contact status
figure
subplot(2, 1, 1)
plot(estBaseTime(startIter:endIter), lfForceZ(startIter:endIter), 'r', 'LineWidth', 2, 'LineStyle', '--');
hold on
plot(estBaseTime(startIter:endIter), 300*lfC(startIter:endIter), 'b', 'LineWidth', 2, 'LineStyle', '-');
subplot(2, 1, 2)
plot(estBaseTime(startIter:endIter), rfForceZ(startIter:endIter), 'r', 'LineWidth', 2, 'LineStyle', '--');
hold on
plot(estBaseTime(startIter:endIter), 300*rfC(startIter:endIter), 'b', 'LineWidth', 2, 'LineStyle', '-');

%% Compute errors

RPEgap = 100;
align_yaw = false;
% [rotError, posError, velError, ATErot, ATEpos, ATEvel, RPErot, RPEpos]
if enable_ocekf
[errors.ocekfL.rotError, errors.ocekfL.posError, errors.ocekfL.velError, ...
    errors.ocekfL.ATErot, errors.ocekfL.ATEpos, errors.ocekfL.ATEvel,  ...
    errors.ocekfL.RPErot, errors.ocekfL.RPEpos, errors.ocekfL.rpyError] = Estimation.Utils.getLeftInvariantErrorMetrics(linkBasePos(startIter:endIter, :), ...
    linkBaseRot(startIter:endIter, :), ...
    linkBaseLinVel(startIter:endIter, :), ...
    outMap(ocekfmat).basePos(startIter:endIter, :), ...
    outMap(ocekfmat).baseRPY(startIter:endIter, :), ...
    outMap(ocekfmat).baseLinVel(startIter:endIter, :), ...
    RPEgap, ...
    align_yaw);
end
if enable_diligent_kio      
[errors.diligent_kioL.rotError, errors.diligent_kioL.posError, errors.diligent_kioL.velError, ...
    errors.diligent_kioL.ATErot, errors.diligent_kioL.ATEpos, errors.diligent_kioL.ATEvel, ... 
    errors.diligent_kioL.RPErot, errors.diligent_kioL.RPEpos, errors.diligent_kioL.rpyError] = Estimation.Utils.getLeftInvariantErrorMetrics(linkBasePos(startIter:endIter, :), ...
    linkBaseRot(startIter:endIter, :), ...
    linkBaseLinVel(startIter:endIter, :), ...
    outMap(diligent_kiomat).basePos(startIter:endIter, :), ...
    outMap(diligent_kiomat).baseRPY(startIter:endIter, :), ...
    outMap(diligent_kiomat).baseLinVel(startIter:endIter, :), ...
    RPEgap, ...
    align_yaw);
end
if enable_diligent_kio_rie
[errors.diligent_kio_rieR.rotError, errors.diligent_kio_rieR.posError, errors.diligent_kio_rieR.velError, ...
    errors.diligent_kio_rieR.ATErot, errors.diligent_kio_rieR.ATEpos, errors.diligent_kio_rieR.ATEvel, ... 
    errors.diligent_kio_rieR.RPErot, errors.diligent_kio_rieR.RPEpos, errors.diligent_kio_rieR.rpyError] = Estimation.Utils.getRightInvariantErrorMetrics(linkBasePos(startIter:endIter, :), ...
    linkBaseRot(startIter:endIter, :), ...
    linkBaseLinVel(startIter:endIter, :), ...
    outMap(diligent_kio_riemat).basePos(startIter:endIter, :), ...
    outMap(diligent_kio_riemat).baseRPY(startIter:endIter, :), ...
    outMap(diligent_kio_riemat).baseLinVel(startIter:endIter, :), ...
    RPEgap, ...
    align_yaw);
end
if enable_codiligent_kio
[errors.codiligent_kioL.rotError, errors.codiligent_kioL.posError, errors.codiligent_kioL.velError, ...
    errors.codiligent_kioL.ATErot, errors.codiligent_kioL.ATEpos, errors.codiligent_kioL.ATEvel, ... 
    errors.codiligent_kioL.RPErot, errors.codiligent_kioL.RPEpos, errors.codiligent_kioL.rpyError] = Estimation.Utils.getLeftInvariantErrorMetrics(linkBasePos(startIter:endIter, :), ...
    linkBaseRot(startIter:endIter, :), ...
    linkBaseLinVel(startIter:endIter, :), ...
    outMap(codiligent_kiomat).basePos(startIter:endIter, :), ...
    outMap(codiligent_kiomat).baseRPY(startIter:endIter, :), ...
    outMap(codiligent_kiomat).baseLinVel(startIter:endIter, :), ...
    RPEgap, ...
    align_yaw);
end
if enable_codiligent_kio_rie
[errors.codiligent_kio_rieR.rotError, errors.codiligent_kio_rieR.posError, errors.codiligent_kio_rieR.velError, ...
    errors.codiligent_kio_rieR.ATErot, errors.codiligent_kio_rieR.ATEpos, errors.codiligent_kio_rieR.ATEvel, ... 
    errors.codiligent_kio_rieR.RPErot, errors.codiligent_kio_rieR.RPEpos, errors.codiligent_kio_rieR.rpyError] = Estimation.Utils.getRightInvariantErrorMetrics(linkBasePos(startIter:endIter, :), ...
    linkBaseRot(startIter:endIter, :), ...
    linkBaseLinVel(startIter:endIter, :), ...
    outMap(codiligent_kio_riemat).basePos(startIter:endIter, :), ...
    outMap(codiligent_kio_riemat).baseRPY(startIter:endIter, :), ...
    outMap(codiligent_kio_riemat).baseLinVel(startIter:endIter, :), ...
    RPEgap, ...
    align_yaw);
end
if enable_invfekf_f
[errors.invfekf_fL.rotError, errors.invfekf_fL.posError, errors.invfekf_fL.velError, ...
    errors.invfekf_fL.ATErot, errors.invfekf_fL.ATEpos, errors.invfekf_fL.ATEvel, ...
    errors.invfekf_fL.RPErot, errors.invfekf_fL.RPEpos, errors.invfekf_fL.rpyError] = Estimation.Utils.getLeftInvariantErrorMetrics(linkBasePos(startIter:endIter, :), ...
    linkBaseRot(startIter:endIter, :), ...
    linkBaseLinVel, ...
    outMap(invfekf_fmat).basePos(startIter:endIter, :), ...
    outMap(invfekf_fmat).baseRPY(startIter:endIter, :), ...
    outMap(invfekf_fmat).baseLinVel(startIter:endIter, :), ...
    RPEgap, ...
    align_yaw);
end

%%
plotErrorCovariance 

if enable_ocekf && enable_diligent_kio_rie ...
                && enable_diligent_kio ...
                && enable_codiligent_kio ...
                && enable_codiligent_kio_rie ...
                && enable_invfekf_f 
plotErrorBars
end
