function dq = odefun(t,q,tau,params)
  
  H_bar = Five_Bar_Mass_Matrix(t, q, tau, params.masses, params.lengths,params.g);
  D_bar = Five_Bar_CN_Matrix(t, q, tau, params.masses, params.lengths,params.g);

  dq = [q(5:8);H_bar\(D_bar)];
  dq = dq(1:8);
end
