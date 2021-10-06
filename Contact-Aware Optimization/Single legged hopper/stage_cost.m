function [J] = stage_cost(x, u, p)
%STAGE_COST stage cost
    J = 0.0;
    for k=1:p.Nt-1
        J = J + 0.5*(x(:,k)-p.xref(:,k))'*p.Q*(x(:,k)-p.xref(:,k)) ...
              + 0.5*(u(:,k)-p.uref(:,k))'*p.R*(u(:,k)-p.uref(:,k));
    end
end

