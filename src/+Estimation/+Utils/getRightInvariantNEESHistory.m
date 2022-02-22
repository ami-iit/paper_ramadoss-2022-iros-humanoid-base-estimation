function [aneespos, aneesrot] = getRightInvariantNEESHistory(pos, rpy, poshat, rpyhat, Pt, align_yaw)
nrIter = length(pos);
aneespos = zeros(nrIter, 1);
aneesrot = zeros(nrIter, 1);
dim = 3;
for idx = 1:nrIter
    R_i = Utils.rpy2rot(rpy(idx, 1), rpy(idx, 2), rpy(idx, 3));
    p_i = pos(idx, :)';
    
    if ~align_yaw
        R_i_hat = Utils.rpy2rot(rpyhat(idx, 1), rpyhat(idx, 2), rpyhat(idx, 3));
    else
        R_i_hat = Utils.rpy2rot(rpyhat(idx, 1), rpyhat(idx, 2), rpy(idx, 3));
    end
    p_i_hat = poshat(idx, :)';
    
    % right invariant error ate
    deltaR = R_i*R_i_hat';
    deltaPhi = LieGroups.SO3.logvee(deltaR);
    deltap = p_i - deltaR*p_i_hat;
    e = [deltap; deltaPhi];
    P = squeeze(Pt(idx, :, :));
    if (trace(P) < 1e-14)
        continue
    end

    if (cond(P) < 1e-16)
        continue
    end

    neespos(idx) = (e(1:3)'*inv(P(1:3, 1:3))*e(1:3));
    neesrot(idx) = (e(4:6)'*inv(P(4:6, 4:6))*e(4:6));
end

for idx = 1:nrIter
    aneespos(idx) = sum(neespos(1:idx))/(idx*dim);
    aneesrot(idx) = sum(neesrot(1:idx))/(idx*dim);
end

end