function [ vq ] = phaseInterp( x, v, xq, method )
    % Interpolates the vector values, while allowing for discontinuities
    % on the start/stop of all-zero phases.
    
    % Force all to be row vectors
    x = x(:)';
    v = v(:)';
    xq = xq(:)';
    
    ne0 = find(v);
    ix0 = unique([ne0(1) ne0(diff([0 ne0])>1)]); 
    ix1 = ne0([find(diff([0 ne0])>1)-1 length(ne0)]);
    
    vq = [];
    for i = 1:length(ix0)
        vq = [vq interp1(x(ix0(i):ix1(i)), v(ix0(i):ix1(i)), ...
              xq(xq >= min(x(ix0(i)), x(ix1(i))) & xq <= max(x(ix0(i)), x(ix1(i)))), method)];
        % Handle flight phase interpolation
        if i < length(ix0)
            vq = [vq zeros(1, length(xq(xq > x(ix1(i)) & xq < x(ix0(i+1)))))];
        end
    end
end

