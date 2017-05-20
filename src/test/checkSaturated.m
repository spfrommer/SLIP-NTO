function [] = checkSaturated( funParams, constraintsFun, ntoParams )
%CHECKSATURATED Checks if any inequality constraints/limits are
%active/saturated
    [lb, ub] = bounds(ntoParams);
    
    for i = 1:length(lb)
        [type, gridPoint] = getParamType(ntoParams, i);
        if abs(funParams(i)-lb(i)) < 1e-9
            fprintf('%s hitting lower bound (%f) at element %d and grid point %d\n', ...
                    type, lb(i), i, gridPoint);
        elseif abs(funParams(i)-ub(i)) < 1e-9
            fprintf('%s hitting upper bound (%f) at element %d and grid point %d\n', ...
                    type, ub(i), i, gridPoint);
        end
    end
    
    [ci, ~, ~, ~] = constraintsFun(funParams);
    
    for i = 1:length(ci)
        [type, limit, gridPoint] = getIneqConstraint(ntoParams, i);
        if ci(i) + 1e-9 >= 0
            fprintf('%s constraint active (limit %f, violation %f) at element %d and grid point %d\n', ...
                    type, limit, ci(i), i, gridPoint);
        end
    end
end

