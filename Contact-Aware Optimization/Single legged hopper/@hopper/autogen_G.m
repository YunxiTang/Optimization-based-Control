function G = autogen_G(g,m_b,m_f)
%AUTOGEN_G
%    G = AUTOGEN_G(G,M_B,M_F)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    06-Oct-2021 10:57:35

G = [0.0;g.*m_b;0.0;g.*m_f];
