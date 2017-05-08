classdef (Sealed) Resource < handle
   methods (Access = public)
      function path = getDataPath(~, simNum)
         path = strcat('../data/sim', num2str(simNum), '.txt');
      end
      
      function [fpath, bpath] = getFigurePaths(~, simNum)
         fpath = strcat('../figures/fig', num2str(simNum), 'f.png');
         bpath = strcat('../figures/fig', num2str(simNum), 'b.png');
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