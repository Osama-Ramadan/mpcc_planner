function [F] = Forkleft_ODE(l, vd1, vd2, x)

    
    F(1,1) = vd1*cos(x(3))*cos(x(4));
    F(2,1) = vd1*sin(x(3))*cos(x(4));
    F(3,1) = -vd1*sin(x(4))/l;
    F(4,1) =  vd2;
end