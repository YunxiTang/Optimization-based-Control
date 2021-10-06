function [lf_cost] = terminal_cost(xf,p)
%TERMINAL_COST 
    lf_cost = 0.5*(xf-p.xref(:,end))'*p.Q_f*(xf-p.xref(:,end));
end

