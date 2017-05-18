% Generates starting images for all runs
for i=1:Resource.instance().numDataFiles()
   disp(i);
   results = readSim(i);
   results.render();
   close all;
end