classdef (Sealed) Resource < handle
   methods (Access = public)
      function n = numDataFiles(obj)
         n = 0;
         while exist(obj.getDataPath(n + 1), 'file') == 2
            n = n + 1;
         end
      end
      
      function path = projRoot(~)
         path = mfilename('fullpath');
         idx = strfind(path, filesep);
         path = path(1:idx(end-2));
      end
      
      function path = getDataPath(obj, simNum)
         path = strcat(obj.projRoot(), 'data/sim', num2str(simNum), '.txt');
      end
      
      function [fpath, bpath] = getFigurePaths(obj, simNum)
         fpath = strcat(obj.projRoot(), 'figures/fig', num2str(simNum), 'f.png');
         bpath = strcat(obj.projRoot(), 'figures/fig', num2str(simNum), 'b.png');
      end
   end
   methods (Access = private)
      function obj = Resource()
      end
   end
   methods (Static)
      function singleObj = instance()
         persistent localObj
         if isempty(localObj) || ~isvalid(localObj)
            localObj = Resource();
         end
         singleObj = localObj;
      end
   end
end