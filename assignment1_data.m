csvfile = 'log_2910_65_2cycles.csv';
labels = strsplit(fileread(csvfile), '\n'); % Split file in lines
labels = strsplit(labels{:, 2}, ', '); % Split and fetch the labels (they are in line 2 of every reco
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels

timeVector= data(801:3600,1); 
omegaA= data(801:3600,3); % OUTPUT 1
omegaB= data(801:3600,4); % OUTPUT 2

voltage= data(801:3600,5); % INPUT



