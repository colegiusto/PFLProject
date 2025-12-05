function u = linearizedPD(x, x_ref, p, kp, kd)
    terms = calculateSpong(x,p);

    hbar = terms.h2 - terms.d22 * terms.h1/  terms.d12;
    phibar = terms.phi2 - terms.d22 * terms.phi1 / terms.d12;
    
    d1bar = terms.d21 - terms.d22 * terms.d11 / terms.d12;
    
    pd = kp*(x(1)-x_ref(1))+kd*(x(3)-x_ref(3));

    u = hbar + phibar - d1bar*pd;

end

