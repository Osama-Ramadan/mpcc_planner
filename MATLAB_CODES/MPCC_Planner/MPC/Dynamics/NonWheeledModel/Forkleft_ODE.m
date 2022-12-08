function [F] = Forkleft_ODE(l, vd1, vd2, x)

    
    F(1,1) = vd1*cos(x(3))*cos(x(6));
    F(2,1) = vd1*sin(x(3))*cos(x(6));
    F(3,1) = -vd1*sin(x(6))/l;
    F(4,1) =  0;
    F(5:12,1) = 0;
end