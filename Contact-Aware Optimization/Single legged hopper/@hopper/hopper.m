classdef hopper
    %HOPPER Trajectory optimization for a plannar hopper
    % More like a slip model
    
    properties
        %% hoppper dynmaics
        g = 9.81
        m1 = 1.5        % body mass
        m2 = 0.2        % foot mass
        l_min = 0.5     % minimum length
        l_max = 1.5     % maximum length
        Nx = 8;         % Number of state
        Nu = 2;         % Number of control
        h = 0.05;        % time step
    end
    
    methods
        function obj = hopper()
            %HOPPER 
            disp("[INFO]: Construct a hopper.");
        end
        
        function dx = flight_dynamics(obj, x, u)
            % Flight dynamics
            m_b = obj.m1;
            m_f = obj.m2;
            q = x(1:obj.Nx/2);
            dq = x(obj.Nx/2+1:obj.Nx);
            M = obj.autogen_M(m_b,m_f);
            C = obj.autogen_C();
            G = obj.autogen_G(obj.g,m_b,m_f);
            B = obj.autogen_B(q(1),q(2),q(3),q(4));
            ddq = M \ (B*u - G - C*dq);
            dx = [dq;ddq];
        end
        
        function dx = stance_dynamics(obj, x, u)
            % Flight dynamics
            m_b = obj.m1;
            m_f = obj.m2;
            q = x(1:obj.Nx/2);
            dq = x(obj.Nx/2+1:obj.Nx);
            M = obj.autogen_M(m_b,m_f);
            C = obj.autogen_C();
            G = obj.autogen_G(obj.g,m_b,m_f);
            B = obj.autogen_B(q(1),q(2),q(3),q(4));
            B(3,1) = 0;
            B(3,2) = 0;
            B(4,1) = 0;
            B(4,2) = 0;
            ddq = M \ (B*u - G - C*dq);
            dx = [dq;ddq];
        end
        
        function x_next = flight_dynamics_rk(obj, x, u)
            dt = obj.h;
            k1 = obj.flight_dynamics(x,             u);
            k2 = obj.flight_dynamics(x + 0.5*dt*k1, u);
            k3 = obj.flight_dynamics(x + 0.5*dt*k2, u);
            k4 = obj.flight_dynamics(x +     dt*k3, u);
            x_next = x + dt/6*(k1+2*k2+2*k3+k4);
        end
        
        function x_next = stance_dynamics_rk(obj, x, u)
            dt = obj.h;
            k1 = obj.stance_dynamics(x,             u);
            k2 = obj.stance_dynamics(x + 0.5*dt*k1, u);
            k3 = obj.stance_dynamics(x + 0.5*dt*k2, u);
            k4 = obj.stance_dynamics(x +     dt*k3, u);
            x_next = x + dt/6*(k1+2*k2+2*k3+k4);
        end
        
        function xn = jump_map(obj, x)
            % Assume the foot experiences inelastic collisions (pined model)
            xn = [x(1:6); 0.0; 0.0];
        end
    end
    
    methods (Static)
        M = autogen_M(m_b,m_f);
        G = autogen_G(g,m_b,m_f);
        B = autogen_B(q1,q2,q3,q4);
        C = autogen_C();
        P = autogen_energy_P(g,m_b,m_f,q2,q4);
        K = autogen_energy_T(dq1,dq2,dq3,dq4,m_b,m_f);
    end
end

