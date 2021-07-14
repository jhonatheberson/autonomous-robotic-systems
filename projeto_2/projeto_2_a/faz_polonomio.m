function [X, Y, a, b, MAT] = faz_polonomio (position,point_end,teta_i,teta_f,incremento)
    %position = [-1.9500e+00, +1.6750e+00, 0.0000];
    %point_end = [+1.5250e+00, -1.4750e+00, 0.0000];
    %teta_i = 0;
    %teta_f = 0;
    %incremento = 0.01;

    delta = 1;
    deltaX = point_end(1) - position(1);
    deltaY = point_end(2) - position(2);
    teta1 = teta_i;
    teta2 = teta_f;
    alfa_init = tan(teta_i);
    alfa_end = tan(teta_f);

    if ((((pi/2)-delta) <= teta1) && (teta1 <= ((pi/2)+delta)) && (((pi/2)-delta) <= teta2) && (teta2 <= ((pi/2)+delta)))
        %printf ('i');
        b1 = deltaY;
        b2 = 0;
        a0 = position(1);
        a1 = 0;
        a2 = 3*deltaX;
        a3 = -2*deltaX;
        b0 = position(2);
        b3 = deltaY-b1-b2;

    elseif ((((pi/2)-delta) <= teta1) && (teta1 <= ((pi/2)+delta)))
        %printf ('ii');
        a3 = -(deltaX/2);
        b3 = 1;
        a0 = position(1);
        a1 = 0;
        a2 = deltaX-a3;
        b0 = position(2);
        b1 = 2*(deltaY-alfa_end*deltaX) - alfa_end*a3 + b3;
        b3 = (2*alfa_end*deltaX-deltaY) + alfa_end*a3 - 2*b3;
        
    elseif ((((pi/2)-delta) <= teta2) && (teta2 <= ((pi/2)+delta)))
        %printf ('iii');
        a1 = 3*(deltaX/2);
        b2 = 1;
        a0 = position(1);
        a2 = 3*deltaX - 2*a1;
        a3 = a1 - 2*deltaX;
        b0 = position(2);
        b1 = alfa_init*a1;
        b3 = deltaY - alfa_init*a1 - b2;
    else
        %printf ('iv');
        a1 = deltaX;
        a2 = 0;
        a0 = position(1);
        a3 = deltaX - a1 - a2;
        b0 = position(2);
        b1 = alfa_init*a1;
        b2 = 3*(deltaY-alfa_end*deltaX) + 2*(alfa_end-alfa_init)*a1 + alfa_end*a2;
        b3 = 3*alfa_end*deltaX - 2*deltaY - (2*alfa_end-alfa_init)*a1 -  alfa_end*a2;

    a = [a0 a1 a2 a3];
    b = [b0 b1 b2 b3];
   
    X = zeros((1/incremento),1);
    Y = zeros((1/incremento),1);
    Z = zeros((1/incremento),1);
    p = 0;
    for i = 0:incremento:1
        x = a0 + a1*i + a2*(i^2) + a3*(i^3);
        y = b0 + b1*i + b2*(i^2) + b3*(i^3);
        z = 0.05; 
        X(p+1) = x;
        Y(p+1) = y;
        Z(p+1) = z;
        p = p+1;
    end
        
    MAT = [X Y Z]; 
    
    %writematrix(MAT,'Matriz de pontos.csv')

end