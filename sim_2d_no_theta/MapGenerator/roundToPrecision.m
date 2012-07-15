function x_rounded  = roundToPrecision(x, decimalplaces)
    temp = x*(10^decimalplaces);
    if ( abs(temp- floor(temp)) < 0.5)
        x_rounded = floor(temp)/(10^decimalplaces);
    else
        x_rounded = ceil(temp)/(10^decimalplaces);
    end
    
end