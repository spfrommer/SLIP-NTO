function [varargout] = call(fun, x, nout)
    % Calls the specific function, specifying number of output args
    varargout = cell(nout, 1);
    [varargout{:}] = fun(x);
end