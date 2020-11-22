function [y] = converta(angle)
    if angle <= pi
        y = abs(angle-pi);
    else 
        y = abs(angle-3*pi);
    end
end
