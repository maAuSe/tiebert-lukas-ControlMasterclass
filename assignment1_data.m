csvfile = 'C:\Users\campa\QRCrecords\log_gpio_20-10-2025_14-26-20.csv';
labels = strsplit(fileread(csvfile), '\n'); % Split file in lines
labels = strsplit(labels{:, 2}, ', '); % Split and fetch the labels (they are in line 2 of every reco
data = dlmread(csvfile, ',', 2, 0); % Data follows the labels

timeVector= data(368:4565,1);
omegaA= data(368:4565,3);
omegaB= data(368:4565,4);

voltage= data(368:4565,5);



