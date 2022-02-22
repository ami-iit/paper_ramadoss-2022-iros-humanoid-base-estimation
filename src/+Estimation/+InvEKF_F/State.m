classdef State
    methods (Static)
        % This state uses a different serialization than the one
        % in LieGroups.SE_Nplus2_3 in both states and velocity vector
        % This change in serialization mainly affects the Adjoint
        % matrix order (provides a lower triangular matrix)
        % We will use this state in the filter to be coherent with the original
        % implementation in
        % https://github.com/UMich-BipedLab/Contact-Aided-Invariant-EKF
        % Note that this state does not contain additional landmarks
        % it contains foot rotation unit vectors along x- and z-direction
        % see paper: https://ieeexplore.ieee.org/document/9233715
        function [R, v, p, prf, xrf, zrf, plf, xlf, zlf, bg, ba] = extract(X, theta)
            R = X(1:3, 1:3);
            v = X(1:3, 4);
            p = X(1:3, 5);
            prf = X(1:3, 6);
            xrf = X(1:3, 7);
            zrf = X(1:3, 8);
            plf = X(1:3, 9);
            xlf = X(1:3, 10);
            zlf = X(1:3, 11);
            
            bg = theta(1:3);
            ba = theta(4:6);
        end
        
        function  [X, theta] = construct(R, v, p, prf, xrf, zrf, plf, xlf, zlf, bg, ba)
            X = [         R    v   p   prf   xrf  zrf  plf xlf zlf; ...
                zeros(1, 3)    1   0     0     0    0    0   0   0; ...
                zeros(1, 3)    0   1     0     0    0    0   0   0; ...
                zeros(1, 3)    0   0     1     0    0    0   0   0; ...
                zeros(1, 3)    0   0     0     1    0    0   0   0; ...
                zeros(1, 3)    0   0     0     0    1    0   0   0; ...
                zeros(1, 3)    0   0     0     0    0    1   0   0; ...
                zeros(1, 3)    0   0     0     0    0    0   1   0; ...
                zeros(1, 3)    0   0     0     0    0    0   0   1];
            
            theta = [bg; ba];
        end
        
        function AdjX = Adjoint(X)
            dummy_theta = zeros(6, 1);
            [R, v, p, prf, xrf, zrf, plf, xlf, zlf, ~, ~] = Estimation.InvEKF_F.State.extract(X, dummy_theta);
            
            v_cross = Utils.skew(v);
            p_cross = Utils.skew(p);
            prf_cross = Utils.skew(prf);
            xrf_cross = Utils.skew(xrf);
            zrf_cross = Utils.skew(zrf);
            plf_cross = Utils.skew(plf);
            xlf_cross = Utils.skew(xlf);
            zlf_cross = Utils.skew(zlf);
            Z3 = zeros(3);
            
            AdjX =   [         R  Z3 Z3 Z3 Z3 Z3 Z3 Z3 Z3; ...
                       v_cross*R   R Z3 Z3 Z3 Z3 Z3 Z3 Z3; ...
                       p_cross*R  Z3  R Z3 Z3 Z3 Z3 Z3 Z3; ...
                     prf_cross*R  Z3 Z3  R Z3 Z3 Z3 Z3 Z3; ...
                     xrf_cross*R  Z3 Z3 Z3  R Z3 Z3 Z3 Z3; ...
                     zrf_cross*R  Z3 Z3 Z3 Z3  R Z3 Z3 Z3; ...
                     plf_cross*R  Z3 Z3 Z3 Z3 Z3  R Z3 Z3; ...
                     xlf_cross*R  Z3 Z3 Z3 Z3 Z3 Z3  R Z3; ...
                     zlf_cross*R  Z3 Z3 Z3 Z3 Z3 Z3 Z3  R];
        end
        
        
        function [X, theta] = exphat(v_X, v_theta)
            % use closed form solution instead of expm()
            [omega, alin, vlin, vrf, wxrf, wzrf, vlf, wxlf, wzlf] = Estimation.InvEKF_F.State.splitVector(v_X);            

            JlSO3 = LieGroups.SO3.leftJacobian(omega);
            
            R = LieGroups.SO3.exphat(omega);
            v = JlSO3*alin;
            p = JlSO3*vlin;
            prf = JlSO3*vrf;
            xrf = JlSO3*wxrf;
            zrf = JlSO3*wzrf;
            plf = JlSO3*vlf;
            xlf = JlSO3*wxlf;
            zlf = JlSO3*wzlf;
            
            bg = v_theta(1:3);
            ba = v_theta(4:6);
                       
            [X, theta] = Estimation.InvEKF_F.State.construct(R, v, p, prf, xrf, zrf, plf, xlf, zlf, bg, ba);
        end
        
        function [omega, alin, vlin, vrf, wxrf, wzrf, vlf, wxlf, wzlf] =  splitVector(v)
            assert( length(v) == 27, 'vector size mismatch');
            omega = v(1:3);
            alin = v(4:6);
            vlin = v(7:9);
            vrf = v(10:12);
            wxrf = v(13:15);
            wzrf = v(16:18);
            vlf = v(19:21);
            wxlf = v(22:24);
            wzlf = v(25:27);
        end
        
        
        function [Pbrot, Pvel, Ppos, Prfpos, Plfpos, Pbg, Pba] = extractStateVarSubBlockEvolutions(Ptraj, estimate_bias)
            Pbrot = zeros(length(Ptraj), 3, 3);
            Ppos = zeros(length(Ptraj), 3, 3);
            Pvel = zeros(length(Ptraj), 3, 3);
            Plfpos = zeros(length(Ptraj), 3, 3);
            Prfpos = zeros(length(Ptraj), 3, 3);
            
            Pbg = zeros(length(Ptraj), 3, 3);
            Pba = zeros(length(Ptraj), 3, 3);
            
            for iter_idx = 1: length(Ptraj)
                P = squeeze(Ptraj{iter_idx});
                Pbrot(iter_idx, :, :) = P(1:3, 1:3);
                Pvel(iter_idx, :, :) = P(4:6, 4:6);
                Ppos(iter_idx, :, :) = P(7:9, 7:9);
                Prfpos(iter_idx, :, :) = P(10:12, 10:12);
                Plfpos(iter_idx, :, :) = P(19:21, 19:21);
                
                if estimate_bias
                    Pbg(iter_idx, :, :) = P(28:30, 28:30);
                    Pba(iter_idx, :, :) = P(31:33, 31:33);
                end
            end
        end
              
    end
end

