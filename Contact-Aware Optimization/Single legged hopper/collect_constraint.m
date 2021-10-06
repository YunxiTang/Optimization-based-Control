function [c, ceq] = collect_constraint(ztraj, p, rbt)
%DYNAMIC_CONSTRAINT dynamic constraint (nonlinear constraint)
% rbt: hopper model
% ztraj: decision variable
% p: problem
    Nx = p.Nx;
    Nu = p.Nu;
    Nt = p.Nt;
    h = p.h;
    non_con = zeros(p.m_nlp, 1);
    
    z = reshape(ztraj(1:end-Nx), [Nx+Nu, Nt-1]);
    xtraj = [z(1:Nx,:) ztraj(end-(Nx-1):end)];
    utraj = z(Nx+1:Nx+Nu,:);
    %% Equality constraints
    % 1. initial/terminal state constraint 
    non_con(p.c_init_inds,:) = ztraj(1:Nx) - p.xref(:,1);
    non_con(p.c_term_inds,:) = ztraj((end-(Nx-1)):end) - p.xref(:,end);
    % 2. Dynamical Constraint
    % defects
    d = zeros(p.Nx, p.Nt-1);
    for k=1:(p.Nmodes-1)
        if mod(k, 2) == 1
            % stance phase
            for j=1:p.Nm
                s = (k-1)*p.Nm + j;
                d(:,s) = rbt.stance_dynamics_rk(xtraj(:,s), utraj(:,s)) - xtraj(:,s+1);
            end
        else
            % flight phase
            for j = 1:(p.Nm-1)
                s = (k-1)*p.Nm + j;
                d(:,s) = rbt.flight_dynamics_rk(xtraj(:,s), utraj(:,s)) - xtraj(:,s+1);
            end
            % phase switch (flight2stance)
            s = k*p.Nm;
            x_n = rbt.flight_dynamics_rk(xtraj(:,s),utraj(:,s));
            d(:,s) = rbt.jump_map(x_n) - xtraj(:,s+1);
        end
    end
    if mod(p.Nmodes,2) == 1
        % the last phase is a stance phase
        for j = 1:(p.Nm-1)
            s = (p.Nmodes-1)*p.Nm + j;
            d(:,s) = rbt.stance_dynamics_rk(xtraj(:,s),utraj(:,s)) - xtraj(:,s+1);
        end
    else
        % the last phase is a flight phase
        for j = 1:(p.Nm-1)
            s = (p.Nmodes-1)*p.Nm + j;
            d(:,s) = rbt.flight_dynamics_rk(xtraj(:,s),utraj(:,s)) - xtraj(:,s+1);
        end
    end
    non_con(p.c_dyn_inds,:) = reshape(d, [p.Nx*(p.Nt-1), 1]);
    
    % 3. Stance constraint
    contact = zeros(length(p.c_stance_inds), 1);
    t = 1;
    for k=1:p.Nmodes
        if mod(k,2) == 1
            for j=1:p.Nm
                s = (k-1)*p.Nm + j;
                contact(t,:) = xtraj(4,s);
                t = t + 1;
            end
        end
    end
    non_con(p.c_stance_inds,:) = contact;
    
    %% Inequality Constraints
    len_lim = zeros(p.Nt * 2, 1);
    ct = 1;
    for k = 1:1:p.Nt
        len_lim(ct,:) = norm(xtraj(1:2,k) - xtraj(3:4,k)) - rbt.l_max;
        len_lim(ct+1,:) = rbt.l_min -  norm(xtraj(1:2,k) - xtraj(3:4,k));
        ct = ct + 2;
    end
    non_con(p.c_length_inds,:) = len_lim;
    % flight phase
    gap = zeros(length(p.c_flight_inds), 1);
    q = 1;
    for k=1:p.Nmodes
        if mod(k,2) == 0
            for j=1:p.Nm
                s = (k-1)*p.Nm + j;
                gap(q,:) = 0.0001-xtraj(4,s);
                q = q + 1;
            end
        end
    end
    non_con(p.c_flight_inds, :) = gap;
    %% split all the constraints into Eq_constraints and Ineq_constraints
    ceq = non_con(1:end-(2*p.Nt+length(p.c_flight_inds)),:); 
    c = non_con(end-(2*p.Nt+length(p.c_flight_inds))+1:end,:);
end

