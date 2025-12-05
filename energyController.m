function u = energyController(x,p, kp, kd, alpha)
    terms = calculateSpong(x,p);

    dbar = terms.d22 - terms.d21 * terms.d12 / terms.d11;

    hbar = terms.h2 - terms.d21 * terms.h1 / terms.d11;

    phibar = terms.phi2 - terms.d21 * terms.phi1 / terms.d11;

    q2d = alpha * atan(x(3));

    v = -(x(2)-q2d)*kp - x(4)*kd;

    u = hbar + phibar + dbar * v;
end

