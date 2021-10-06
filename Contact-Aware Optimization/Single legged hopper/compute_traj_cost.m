function [J] = compute_traj_cost(ztraj,p)
%COMPUTE_TRAJ_COST compute trajectory cost
%   z: Nx * Nt + Nu * (Nt - 1) dimensional vector
    Nx = p.Nx;
    Nu = p.Nu;
    Nt = p.Nt;
    h = p.h;
    
    z = reshape(ztraj(1:end-Nx), Nx+Nu, Nt-1);
    xtraj_p = z(1:Nx,:);
    utraj = z(Nx+1:Nx+Nu,:);
    J_stage = stage_cost(xtraj_p, utraj, p) * 1;
    x_f = reshape(ztraj(end-(p.Nx-1):end), Nx, 1);
    J_terminal = terminal_cost(x_f, p) * 1;
    J = J_stage + J_terminal;
end

