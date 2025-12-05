function u = switchingController(x,p,margin, kp, kd, K)
    
    
    error = x(1)+x(2) - pi/2

    if abs(error) > margin
        u = linearizedPD(x, [pi/2; 0; 0; 0], p, kp, kd);
        return
    end
    
    u = - K *(x - [pi/2; 0; 0; 0]);
end

