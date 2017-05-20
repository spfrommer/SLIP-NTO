function [ type, limit, gridPoint ] = getIneqConstraint( ntoParams, index )
%GETCONSTRAINTTYPE Returns the type of inequality constraint located at that index
    for p = 1 : size(ntoParams.phases, 1)
        % Offset in the inequality parameter vector due to phase
        picOffset = 4 * (ntoParams.gridn) * (p - 1);
        
        for i = 1 : ntoParams.gridn - 1
            if index == picOffset + (i-1)*4 + 1
                type = 'maxLen';
                limit = ntoParams.maxlen;
                gridPoint = (p-1) * ntoParams.gridn + i;
            elseif index == picOffset + (i-1)*4 + 2
                type = 'minLen';
                limit = ntoParams.minlen;
                gridPoint = (p-1) * ntoParams.gridn + i;
            elseif index == picOffset + (i-1)*4 + 3
                type = 'minGrf';
                limit = 0;
                gridPoint = (p-1) * ntoParams.gridn + i;
            elseif index == picOffset + (i-1)*4 + 4
                type = 'maxGrf';
                limit = ntoParams.maxgrf;
                gridPoint = (p-1) * ntoParams.gridn + i;
            end
        end
        
        if p == size(ntoParams.phases, 1)
            if index == picOffset+(ntoParams.gridn-1)*4+1
                type = 'maxLen';
                limit = ntoParams.maxlen;
                gridPoint = p * ntoParams.gridn;
            elseif index == picOffset+(ntoParams.gridn-1)*4+2
                type = 'minLen';
                limit = ntoParams.minlen;
                gridPoint = p * ntoParams.gridn;
            elseif index == picOffset+(ntoParams.gridn-1)*4+3
                type = 'minGrf';
                limit = 0;
                gridPoint = p * ntoParams.gridn;
            elseif index == picOffset+(ntoParams.gridn-1)*4+4
                type = 'maxGrf';
                limit = ntoParams.maxgrf;
                gridPoint = p * ntoParams.gridn;
            end
        else 
            if index == picOffset+(ntoParams.gridn-1)*4+1
                type = 'maxLen';
                limit = ntoParams.maxlen;
                gridPoint = p * ntoParams.gridn;
            elseif index == picOffset+(ntoParams.gridn-1)*4+2
                type = 'minLen';
                limit = ntoParams.minlen;
                gridPoint = p * ntoParams.gridn;
            elseif index == picOffset+(ntoParams.gridn-1)*4+3
                type = 'none';
                limit = NaN;
                gridPoint = p * ntoParams.gridn;
            elseif index == picOffset+(ntoParams.gridn-1)*4+4
                type = 'none';
                limit = NaN;
                gridPoint = p * ntoParams.gridn;
            end
        end
    end
end

