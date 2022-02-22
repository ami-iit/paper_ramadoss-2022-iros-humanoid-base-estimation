classdef Filter < handle
    properties (Access = private)
        timestamp_id (1, 1) uint64;
        
        X (:, :) double;
        theta (6, 1) double;
        P (:, :) double;
        
        t_prev (1, 1) double;
        X_prev (:, :) double;
        theta_prev (6, 1) double;
        P_prev (:, :) double;
        
        alpha_prev (3, 1) double;
        omega_prev (3, 1) double;
        encoders_prev (:, 1) double;
        contacts_prev (2, 1) logical;
        
        X0 (:, :) double;
        theta0 (6, 1) double;
        P0 (:, :) double;
        
        Qg (3, 3) double; % gyro variance
        Qa (3, 3) double; % accelerometer variance
        Qcontact (3, 3) double; % contact foot pose variance
        Qswing (3, 3) double; % swing foot pose variance
        QcontactAng (3, 3) double; % contact foot pose variance
        QswingAng (3, 3) double; % swing foot pose variance
        Qba (3, 3) double; % gyro bias variance
        Qbg (3, 3) double; % accelerometer bias variance
        
        Renc (:, :) double; % encoder variance
        R0fkin (6, 6) double; % forward kinematic pose variance
        
        model_comp ;
        options Estimation.Proprioception.EstimatorOptions;
        
        debugger Estimation.Proprioception.DebugOutputs;
        
        filter_configured (1, 1) logical;
        filter_initialized (1, 1) logical;
        bias_initialized (1, 1) logical;

        Rimu3, Rimuinv
    end
    
    properties (Access = private, Constant)
        g = [0; 0; -9.80665];
    end
    
    methods
        function obj = Filter()
            obj.timestamp_id = 0;
            obj.Qa = zeros(3);
            obj.Qg = zeros(3);
            obj.Qcontact = zeros(3);
            obj.Qswing = zeros(3);
            obj.QcontactAng = zeros(3);
            obj.QswingAng = zeros(3);
            obj.Qba = zeros(3);
            obj.Qbg = zeros(3);
           
            obj.R0fkin = zeros(6);
            obj.t_prev = 0.0;
            obj.alpha_prev = zeros(3, 1);
            obj.omega_prev = zeros(3, 1);
            obj.contacts_prev = false(2, 1);
            
            obj.options = Estimation.Proprioception.EstimatorOptions();
            obj.debugger = Estimation.Proprioception.DebugOutputs();
            
            obj.filter_configured = false;
            obj.filter_initialized = false;
            obj.bias_initialized = false;
        end
        
        function obj = setup(obj, prior_dev, sensors_dev, model_comp, options)
            arguments
                obj Estimation.InvEKF_F.Filter;
                prior_dev Estimation.Proprioception.PriorsStdDev;
                sensors_dev Estimation.Proprioception.SensorsStdDev;
                model_comp ;
                options Estimation.Proprioception.EstimatorOptions;
            end
            obj.model_comp = model_comp;
            obj.options = options;
            
            obj.Qa = diag(sensors_dev.accel_noise.^2);
            obj.Qg = diag(sensors_dev.gyro_noise.^2);
            obj.Qcontact = diag(sensors_dev.contact_foot_linvel_noise.^2);
            obj.Qswing = diag(sensors_dev.swing_foot_linvel_noise.^2);
            obj.QcontactAng = diag(sensors_dev.contact_foot_angvel_noise.^2);
            obj.QswingAng = diag(sensors_dev.swing_foot_angvel_noise.^2);
            obj.Qba = diag(sensors_dev.accel_bias_noise.^2);
            obj.Qbg = diag(sensors_dev.gyro_bias_noise.^2);
            
            assert(options.nr_joints_est == length(sensors_dev.encoders_noise), 'Mismatch #joints in options');
            obj.encoders_prev = zeros(options.nr_joints_est, 1);
            obj.Renc = diag(sensors_dev.encoders_noise.^2);
            
            obj.R0fkin = diag(prior_dev.forward_kinematics.^2);
            
            obj.P0 = blkdiag(diag(prior_dev.imu_orientation.^2), ...
                             diag(prior_dev.imu_linear_velocity.^2), ...
                             diag(prior_dev.imu_position.^2), ...
                             diag(prior_dev.right_foot_position.^2), ...
                             diag(prior_dev.right_foot_orientation.^2), ...
                             diag(prior_dev.right_foot_orientation.^2), ...
                             diag(prior_dev.left_foot_position.^2), ... 
                             diag(prior_dev.left_foot_orientation.^2), ... 
                             diag(prior_dev.left_foot_orientation.^2));
            if (options.enable_bias_estimation)
                obj.P0 = blkdiag(obj.P0, ...
                                 diag(prior_dev.gyro_bias.^2), ...
                                 diag(prior_dev.accel_bias.^2));
            end
            
            obj.P_prev = obj.P0;
            
            obj.filter_configured = true;
            obj.filter_initialized = false;
            obj.bias_initialized = false;

            Rimu = obj.model_comp.kindyn.model().getFrameTransform(obj.model_comp.base_link_imu_idx).getRotation().toMatlab();
            obj.Rimu3 = blkdiag(Rimu, Rimu, Rimu);
            obj.Rimuinv = blkdiag(Rimu',Rimu', Rimu');
        end % setup
        
        function obj = initialize(obj, X0, theta0)
            if (obj.filter_configured)
                if (obj.options.enable_bias_estimation)
                    assert(length(theta0) == 6, 'Bias parameter size mismatch');
                end
                
                obj.X0 = X0;
                obj.X = X0;
                obj.X_prev = X0;
                
                obj.theta0 = theta0;
                obj.theta = theta0;
                obj.theta_prev = theta0;
                
                obj.P = obj.P0;
                
                obj.filter_initialized = true;
            end
        end
        
        function [X, theta, P, BaseLinkState, DebugOut, obj] = advance(obj, t, omega, alpha, encoders, contacts)
            % TODO intialize bias static pose
            % obj.initializeBias(alpha, omega, obj.x0)
            obj.timestamp_id = obj.timestamp_id + 1;
            obj.X_prev = obj.X;
            obj.theta_prev = obj.theta;
            obj.P_prev = obj.P;
            
            if (obj.filter_initialized && obj.t_prev > 0.0)
                dt = t - obj.t_prev;
                obj = obj.predictState(obj.omega_prev, obj.alpha_prev, obj.encoders_prev, obj.contacts_prev, dt);
                
                if obj.options.enable_ekf_update
                    if obj.options.enable_kinematic_meas
                        obj = obj.updateKinematics(encoders, contacts, dt);
                    end
                    
                    if obj.options.debug_mode
                        obj.debugger.access = true;
                    end
                end
            end
            
            % store latest values
            obj.omega_prev = omega;
            obj.alpha_prev = alpha;
            obj.encoders_prev = encoders;
            obj.contacts_prev = contacts;
            obj.t_prev = t;
            
            % outputs
            X = obj.X;
            theta = obj.theta;
            P = obj.P;
            
            % extract base link state
            [R, v, p, ~, ~,~, ~, ~, ~, bg, ~] = Estimation.InvEKF_F.State.extract(obj.X, obj.theta);
            q = Utils.rot2quat(R);
            [BaseLinkState.q, BaseLinkState.I_p, BaseLinkState.I_pdot, BaseLinkState.I_omega] = obj.model_comp.getBaseStateFromIMUState(q, p, v, omega-bg);
            
            DebugOut = obj.debugger;
        end
    end
    
    methods (Access = private)
        function obj = predictState(obj, omega, alpha, encoders, contact, dt)
            [R, v, p, pr, xr, zr, pl, xl, zl, bg, ba] = Estimation.InvEKF_F.State.extract(obj.X, obj.theta);
            
            %- bias corrected IMU measurements
            alpha_unbiased = alpha - ba;
            omega_unbiased = omega - bg;
            acc = (R*alpha_unbiased + obj.g);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%-- non linear dynamics
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % base pose
            R_pred = LieGroups.SO3.compose(R, LieGroups.SO3.exphat(omega_unbiased*dt));
            v_pred = v + acc*dt;
            p_pred = p + v*dt + 0.5*acc*dt*dt;
            
            % feet position
            [IMU_q_LF, ~, ~] = obj.model_comp.relativeKinIMU_to_foot_contact(encoders, 'left');
            [IMU_q_RF, ~, ~] = obj.model_comp.relativeKinIMU_to_foot_contact(encoders, 'right');
            IMU_R_LF = Utils.quat2rot(IMU_q_LF);
            IMU_R_RF = Utils.quat2rot(IMU_q_RF);
            
%             pr_off = p_pred + R_pred*IMU_p_RF;
%             pl_off =  p_pred + R_pred*IMU_p_LF;
%             pr_pred = contact(2)*pr + (1 - contact(2))*pr_off;
%             pl_pred = contact(1)*pl + (1 - contact(1))*pl_off;
            
            pr_pred = pr;
            xr_pred = xr;
            zr_pred = zr;
            pl_pred = pl;
            xl_pred = xl;
            zl_pred = zl;

            % bias
            ba_pred = ba;
            bg_pred = bg;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%--Linearized invariant error dynamics
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            gCross = Utils.skew(obj.g);
            Fbase = [zeros(3) zeros(3) zeros(3); ...
                       gCross zeros(3) zeros(3); ...
                     zeros(3)   eye(3) zeros(3)];
            Fc = [Fbase  zeros(9, 18); ...
           zeros(18, 9) zeros(18, 18)];
            
            if obj.options.enable_bias_estimation
                Fc = blkdiag(Fc, zeros(6));
                vCross = Utils.skew(v);
                pCross = Utils.skew(p);
                prCross = Utils.skew(pr);
                xrCross = Utils.skew(xr);
                zrCross = Utils.skew(zr);
                plCross = Utils.skew(pl);
                xlCross = Utils.skew(xl);
                zlCross = Utils.skew(zl);
                
                Fc(1:27, end-5:end) = [        -R  zeros(3); ...
                                        -vCross*R        -R; ...
                                        -pCross*R  zeros(3); ...
                                       -prCross*R  zeros(3); ...
                                       -xrCross*R  zeros(3); ...
                                       -zrCross*R  zeros(3); ...
                                       -plCross*R  zeros(3); ...
                                       -xlCross*R  zeros(3); ...
                                       -zlCross*R  zeros(3)];
            end
            
            Fk = eye(size(Fc)) + (Fc*dt);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%-- Discrete system noise covariance
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Lc = Estimation.InvEKF_F.State.Adjoint(obj.X);
            if obj.options.enable_bias_estimation
                Lc = blkdiag(Lc, eye(6));
            end
            
            Qc = blkdiag(obj.Qg, obj.Qa, zeros(3), ...
                IMU_R_RF*(contact(2)*obj.Qcontact + (1-contact(2))*obj.Qswing)*IMU_R_RF', ...
                IMU_R_RF*(contact(2)*obj.QcontactAng + (1-contact(2))*obj.QswingAng)*IMU_R_RF', ...
                IMU_R_RF*(contact(2)*obj.QcontactAng + (1-contact(2))*obj.QswingAng)*IMU_R_RF', ...
                IMU_R_LF*(contact(1)*obj.Qcontact + (1-contact(1))*obj.Qswing)*IMU_R_LF', ...
                IMU_R_LF*(contact(1)*obj.QcontactAng + (1-contact(1))*obj.QswingAng)*IMU_R_LF', ...
                IMU_R_LF*(contact(1)*obj.QcontactAng + (1-contact(1))*obj.QswingAng)*IMU_R_LF');
            
            if obj.options.enable_bias_estimation
                Qc = blkdiag(Qc, obj.Qbg, obj.Qba);
            end
            
            Qk = Fk*Lc*Qc*Lc'*Fk'*dt;
            
            [obj.X, obj.theta] = Estimation.InvEKF_F.State.construct(R_pred, v_pred, p_pred,...
                                                                      pr_pred, xr_pred, zr_pred, ...
                                                                      pl_pred, xl_pred, zl_pred,...
                                                                      bg_pred, ba_pred);
            obj.P = Fk*obj.P*Fk'+ Qk;
            
            if obj.options.debug_mode
                obj.debugger.predicted_acc = acc;
                obj.debugger.F = Fk;
                obj.debugger.P_pred = obj.P;
                obj.debugger.Q = Qk;
            end
        end % predictState
        
        function obj = updateState(obj, Y, b, H, R, Pi)
            S = H*obj.P*H' + R;
            K = (obj.P*H')/S;
            
            % Copy X along the diagonals if more than one measurement
            X_cell = repmat({obj.X}, 1, length(Y)/size(obj.X,1));
            Z = blkdiag(X_cell{:}) * Y - b;
            
            % right invariant update
            delta = K*Pi*Z;
            
            deltaX = delta(1:27);
            if obj.options.enable_bias_estimation
                deltatheta = delta(end-5:end);
            else
                deltatheta = zeros(6, 1);
            end
            
            [dX, dtheta] = Estimation.InvEKF_F.State.exphat(deltaX, deltatheta);
            obj.X = dX * obj.X;
            obj.theta = obj.theta + dtheta;
            
            I = eye(size(obj.P));
            % perform Joseph update
            obj.P = (I - (K*H))*obj.P;
            obj.P = (obj.P + obj.P')/2;
            
            if obj.options.debug_mode
                eigval = eig(obj.P);
                condn = max(eigval)/min(eigval);
                
                tol = length(eigval)*eps(max(eigval));
                isposdef = all(eigval > tol);
                
                obj.debugger.traceP = trace(obj.P);
                obj.debugger.condP = condn;
                obj.debugger.isPsymmetric = issymmetric(obj.P);
                obj.debugger.isPpositivedefinite = isposdef;
                obj.debugger.isPSPD = (issymmetric(obj.P) && isposdef);
                
                obj.debugger.H = H;
                obj.debugger.R = R;
                obj.debugger.K = K;
                obj.debugger.deltay = Z;
                obj.debugger.deltax = deltaX;

                obj.debugger.PextPoseBase = obj.Rimu3*obj.P(1:9, 1:9)*obj.Rimuinv;
            end
            
        end % updateState
        
        function obj = updateKinematics(obj, encoders, contacts, dt)
            A_R_IMU = obj.X(1:3, 1:3);
            xR = obj.X(1:3, 7);
            zR = obj.X(1:3, 8);
            xL = obj.X(1:3, 10);
            zL = obj.X(1:3, 11);
            xRcross = Utils.skew(xR);
            zRcross = Utils.skew(zR);
            xLcross = Utils.skew(xL);
            zLcross = Utils.skew(zL);
            A_R_RF = [xR xRcross*zR zR];
            A_R_LF = [xL xLcross*zL zL];

            % Jacobian we get here allows us to express the feet velocities
            % in the feet frames, no need for additional transformations
            [y_LF, s_J_IMULF] = obj.model_comp.relativeKinIMU_to_foot_contactExplicit(encoders, 'left');
            [y_RF, s_J_IMURF] = obj.model_comp.relativeKinIMU_to_foot_contactExplicit(encoders, 'right');
            s_pR = y_RF(1:3, 4);
            s_xR = y_RF(1:3, 1);
            s_zR = y_RF(1:3, 3);
            s_pL = y_LF(1:3, 4);
            s_xL = y_LF(1:3, 1);
            s_zL = y_LF(1:3, 3);
            JLFlin = s_J_IMULF(1:3, :);
            JLFang = s_J_IMULF(4:6, :);
            JRFlin = s_J_IMURF(1:3, :);
            JRFang = s_J_IMURF(4:6, :);
            RFlinNoise = JRFlin*obj.Renc*JRFlin';
            RFangNoise = JRFang*obj.Renc*JRFang';
            LFlinNoise = JLFlin*obj.Renc*JLFlin';
            LFangNoise = JLFang*obj.Renc*JLFang';
            xRFRot = A_R_IMU'*xRcross*A_R_RF;
            zRFRot = A_R_IMU'*zRcross*A_R_RF;
            xLFRot = A_R_IMU'*xLcross*A_R_LF;
            zLFRot = A_R_IMU'*zLcross*A_R_LF;
        
            Z3 = zeros(3);
            Z311 = zeros(3, 11);
            Z38 = zeros(3, 8);
            I3 = eye(3);

            if (contacts(1) && contacts(2))
                %%%%%%%%%%%%%%%%%%%%%%%%
                %%-- double support
                %%%%%%%%%%%%%%%%%%%%%%%%
                % measurement model
                %      R   v  p  pr  xr  zr  pl   xl  zl
                Y = [s_pR; 0; 1; -1;  0;  0;  0;  0;  0; ...
                     s_xR; 0; 0;  0; -1;  0;  0;  0;  0; ...
                     s_zR; 0; 0;  0;  0; -1;  0;  0;  0; ...
                     s_pL; 0; 1;  0;  0;  0; -1;  0;  0; ...
                     s_xL; 0; 0;  0;  0;  0;  0; -1;  0; ...
                     s_zL; 0; 0;  0;  0;  0;  0;  0; -1];
                b = zeros(size(Y));
                
                %     R  v  p   pr  xr  zr  pl  xl  zl
                H = [Z3 Z3 -I3  I3  Z3  Z3  Z3  Z3  Z3; ...
                     Z3 Z3  Z3  Z3  I3  Z3  Z3  Z3  Z3; ...
                     Z3 Z3  Z3  Z3  Z3  I3  Z3  Z3  Z3; ...
                     Z3 Z3 -I3  Z3  Z3  Z3  I3  Z3  Z3; ...
                     Z3 Z3  Z3  Z3  Z3  Z3  Z3  I3  Z3; ...
                     Z3 Z3  Z3  Z3  Z3  Z3  Z3  Z3  I3];
                if  obj.options.enable_bias_estimation
                    H = [H zeros(18, 6)];
                end
                
                Rk = blkdiag(A_R_IMU*RFlinNoise*A_R_IMU' + obj.R0fkin(1:3, 1:3), ...
                             xRFRot*RFangNoise*xRFRot' + obj.R0fkin(4:6, 4:6), ...
                             zRFRot*RFangNoise*zRFRot' + obj.R0fkin(4:6, 4:6), ...
                             A_R_IMU*LFlinNoise*A_R_IMU' + obj.R0fkin(1:3, 1:3), ...
                             xLFRot*LFangNoise*xLFRot' + obj.R0fkin(4:6, 4:6), ...
                             zLFRot*LFangNoise*zLFRot' + obj.R0fkin(4:6, 4:6));
                Rk = Rk/dt;
                Pi = [   I3   Z38 Z311 Z311 Z311 Z311 Z311; ...
                       Z311    I3  Z38 Z311 Z311 Z311 Z311; ...
                       Z311  Z311   I3  Z38 Z311 Z311 Z311; ...
                       Z311  Z311 Z311   I3  Z38 Z311 Z311; ...
                       Z311  Z311 Z311 Z311   I3  Z38 Z311; ...
                       Z311  Z311 Z311 Z311 Z311   I3  Z38];
                
                obj = obj.updateState(Y, b, H, Rk, Pi);
            elseif (contacts(1))
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%-- single support left foot
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % measurement model
                %      R   v  p  pr  xr  zr  pl   xl  zl
                Y = [s_pL; 0; 1;  0;  0;  0; -1;  0;  0; ...
                     s_xL; 0; 0;  0;  0;  0;  0; -1;  0; ...
                     s_zL; 0; 0;  0;  0;  0;  0;  0; -1];
                b = zeros(size(Y));
                
                %     R  v  p   pr  xr  zr  pl  xl  zl
                H = [Z3 Z3 -I3  Z3  Z3  Z3  I3  Z3  Z3; ...
                     Z3 Z3  Z3  Z3  Z3  Z3  Z3  I3  Z3; ...
                     Z3 Z3  Z3  Z3  Z3  Z3  Z3  Z3  I3];
                if  obj.options.enable_bias_estimation
                    H = [H zeros(9, 6)];
                end
                
                Rk = blkdiag(A_R_IMU*LFlinNoise*A_R_IMU' + obj.R0fkin(1:3, 1:3), ...
                             xLFRot*LFangNoise*xLFRot' + obj.R0fkin(4:6, 4:6), ...
                             zLFRot*LFangNoise*zLFRot' + obj.R0fkin(4:6, 4:6));
                Rk = Rk/dt;
                Pi = [   I3   Z38 Z311 Z311; ...
                       Z311    I3  Z38 Z311; ...
                       Z311  Z311   I3  Z38];
                
                obj = obj.updateState(Y, b, H, Rk, Pi);
            elseif (contacts(2))
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%-- single support right foot
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % measurement model
                %      R   v  p  pr  xr  zr  pl   xl  zl
                Y = [s_pR; 0; 1; -1;  0;  0;  0;  0;  0; ...
                     s_xR; 0; 0;  0; -1;  0;  0;  0;  0; ...
                     s_zR; 0; 0;  0;  0; -1;  0;  0;  0];
                b = zeros(size(Y));
                
                %     R  v  p   pr  xr  zr  pl  xl  zl
                H = [Z3 Z3 -I3  I3  Z3  Z3  Z3  Z3  Z3; ...
                     Z3 Z3  Z3  Z3  I3  Z3  Z3  Z3  Z3; ...
                     Z3 Z3  Z3  Z3  Z3  I3  Z3  Z3  Z3];
                if  obj.options.enable_bias_estimation
                    H = [H zeros(9, 6)];
                end
                
                Rk = blkdiag(A_R_IMU*RFlinNoise*A_R_IMU' + obj.R0fkin(1:3, 1:3), ...
                             xRFRot*RFangNoise*xRFRot' + obj.R0fkin(4:6, 4:6), ...
                             zRFRot*RFangNoise*zRFRot' + obj.R0fkin(4:6, 4:6));
                Rk = Rk/dt;
                Pi = [   I3   Z38 Z311 Z311; ...
                       Z311    I3  Z38 Z311; ...
                       Z311  Z311   I3  Z38];
                
                obj = obj.updateState(Y, b, H, Rk, Pi);
            end                        
        end % updateKinematics
        
    end
    
end


