function x = modelStateTransitionFcn(x,u)

m_cart = 0.558;                                                            % 0.558 Kg
m_pend = 1.267;                                                            % 1.267 Kg
I_pend = 0.055;                                                            % 0.055 Kg*m^2
l = 0.159;                                                                 % 0.159 m
mu_drag = 0.00002;                                                         % drag_coefficient
g = 9.81;                                                                  % gravity acceleration
                                                       
dt = 0.01;                                                                 % sampling time

m_tot = m_cart + m_pend;

dx1 = x(2);
dx2 = (u - mu_drag*x(2) - m_pend*l*(((-m_pend*g*l*sin(x(3))*cos(x(3)))/(I_pend + m_pend*l^2)) - (x(4)^2)*sin(x(3))))/(m_tot*(1 - (((m_pend^2)*(l^2)*(cos(x(3))^2))/(I_pend + m_pend*l^2))));
dx3 = x(4);
dx4 = (-m_pend*g*l*sin(x(3)) -m_pend*l*dx2*cos(x(3)))/(I_pend + m_pend*l^2);

xddot = [
  dx1;
  dx2;
  dx3;
  dx4
];

x = x + xddot*dt;                                                          % computation of discrete time approximation

end