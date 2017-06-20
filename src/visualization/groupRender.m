% Generates starting images for all runs
for i=21:Resource.instance().numDataFiles()
   results = readSim(i);
   vp = VisParams();
   vp.figSize = 1000;
   results.saveRenderImage(i, vp);
   close all;
end