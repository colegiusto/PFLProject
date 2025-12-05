function u = switchingController(x,p,margin, K, swingup)
    
    
    error = x(1)+x(2) - pi/2;

    if abs(error) > margin
        u = swingup(x,p);
        return
    end
    
    u = - K *(x - [pi/2; 0; 0; 0]);
end

