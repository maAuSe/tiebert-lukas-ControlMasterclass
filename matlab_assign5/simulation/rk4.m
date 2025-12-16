function x1 = rk4(f,x0,u0,dt)
    
    k1 = dt * (f(x0,        u0));
    k2 = dt * (f(x0 + k1/2, u0));
    k3 = dt * (f(x0 + k2/2, u0));
    k4 = dt * (f(x0 + k3,   u0));
    
    x1 = x0 + 1/6 * (k1 + 2 * k2 + 2 * k3 + k4);    
end