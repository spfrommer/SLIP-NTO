function [ type, gridPoint ] = getParamType( index, ntoParams )
%GET Returns the type of parameter located at that index
    p = size(ntoParams.phases, 1);
    cnt = ntoParams.gridn * p;
    
    if     any(1:p == index)
        type = 'stanceT';
        gridPoint = NaN;
    elseif any(p+1:p*2-1 == index)
        type = 'flightT';
        gridPoint = NaN;
    elseif any(p*2:p*2-1+cnt == index)
        type = 'xtoe';
        gridPoint = index - p*2 + 1;
    elseif any(p*2+cnt:p*2-1+cnt*2 == index)
        type = 'xtoedot';
        gridPoint = index - (p*2 + cnt) + 1;
    elseif any(p*2+cnt*2:p*2-1+cnt*3 == index)
        type = 'x';
        gridPoint = index - (p*2 + cnt*2) + 1;
    elseif any(p*2+cnt*3:p*2-1+cnt*4 == index)
        type = 'xdot';
        gridPoint = index - (p*2 + cnt*3) + 1;
    elseif any(p*2+cnt*4:p*2-1+cnt*5 == index)
        type = 'y';
        gridPoint = index - (p*2 + cnt*4) + 1;
    elseif any(p*2+cnt*5:p*2-1+cnt*6 == index)
        type = 'ydot';
        gridPoint = index - (p*2 + cnt*5) + 1;
    elseif any(p*2+cnt*6:p*2-1+cnt*7 == index)
        type = 'ra';
        gridPoint = index - (p*2 + cnt*6) + 1;
    elseif any(p*2+cnt*7:p*2-1+cnt*8 == index)
        type = 'radot';
        gridPoint = index - (p*2 + cnt*7) + 1;
    elseif any(p*2+cnt*8:p*2-1+cnt*9 == index)
        type = 'raddot';
        gridPoint = index - (p*2 + cnt*8) + 1;
    elseif any(p*2+cnt*9:p*2-1+cnt*10 == index)
        type = 'torque';
        gridPoint = index - (p*2 + cnt*9) + 1;
    else
        type = 'none';
        gridPoint = NaN;
    end
end

