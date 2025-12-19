classdef KalmanExperiment 
% KALMANEXPERIMENT An experiment of the Kalman filter
%
% This class contains all data of an experiment with the Kalman filter and
% provides convenient methods to extract some interesting data to assess 
% the quality of the estimation and the behavior of the model. 

% (c) 2020 - Laurens Jacobs, MECO Research Team @ KU Leuven
 
    %% Properties
    properties
        t % the time vector, N samples (1D matrix)
        x % the state vectors, nx number of states x N samples (2D matrix)
        P % the covariance matrices of the states, nx number of states x nx number of states x N samples (3D matrix)
        R % the covariance matrices of the measurements, ny number of measurements x ny number of measurements x N samples (3D matrix)
        y % the measurements, ny number of measurements x N samples (2D matrix)
        u % the inputs, nu number of inputs x N samples (2D matrix) 
        nu % the innovations, nx number of innovations x N samples (2D matrix) 
        S  % the innovation covariance, ny number of measurements x ny number of measurements x N samples (3D matrix) 
    end
    
    %% Publicly accessible methods 
    methods(Access=public) 
        
        function obj = KalmanExperiment(t,x,P,y,R,u,nu,S)
        % KALMANEXPERIMENT Constructor
        % Gather the data of a Kalman filter experiment in a
        % dedicated object.
        % 
        % exp = KALMANEXPERIMENT(t,x,P,y,R,u) takes the following input arguments:
        %   t - the time vector, N samples (1D matrix)
        %   x - the state vectors, nx number of states x N samples (2D matrix)
        %   P - the covariance matrices of the states, nx number of states x nx number of states x N samples (3D matrix)
        %   y - the measurements, ny number of measurements x N samples (2D matrix)
        %   R - the covariance matrices of the measurements, ny number of measurements x ny number of measurements x N samples (3D matrix)
        %   u - the inputs, nu number of inputs x N samples (2D matrix) 
        %   nu - the innovations, ny number of innovations x N samples (2D matrix)
        %   S - the innovation covariance matrix, ny number of innovations x ny number of innovations x N samples (3D matrix)
        
            assert(nargin>=6, 'At least 6 input arguments are required.'); 
        
            obj.t = t;
            obj.x = x;
            obj.P = P;
            obj.y = y;
            obj.R = R;
            obj.u = u;
            
            if nargin>=8
                obj.nu = nu; 
                obj.S = S; 
            else
                obj.nu = zeros(size(obj.y)); 
                obj.S = zeros([size(obj.y,1) size(obj.y)]);
            end
            checkInput(obj); 
            if verLessThan('matlab','8.4'); warning(['You can create ' ...
            'a KalmanExperiment object, but the plot functionality is ' ...
            'only supported from MATLAB R2014b. You are using an ' ...
            'older version.']); end

        end
        
        function plotstates(obj, states, confidence)
        % PLOTSTATES Generates a plot of the evolution of the states and their uncertainty
        %
        % PLOTSTATES(KE) returns a separate figure with the evolution of 
        % every state together with its 95% confidence interval.
        %
        % PLOTSTATES(KE, states) returns a separate figure with the
        % evolution of every state of which the index is in 'states', e.g.
        % PLOTSTATES(KE, [1 3]) plots the evolution of the first and third
        % state. 
        %
        % PLOTSTATES(KE, states, confidence) returns the same of the
        % previous syntax, but uses 'confidence' as cumulative probability
        % (confidence interval), e.g. PLOTSTATES(KE, [1 2], 0.99) plots the
        % 99% confidence interval of states 1 and 2.
        
            % 0. Check MATLAB version (>= R2014b)
            assert(~verLessThan('matlab','8.4'),'Uncertainty plots are only supported from MATLAB R2014b.'); 
        
            % 1. Generate state indices
            if nargin>=3
                assert(confidence > 0 && confidence < 1, 'Cumulative probability must be between 0 and 1.'); 
            end
            if nargin>=2
                assert(all(states<=obj.nx()), ['There are only ' num2str(obj.nx()) ' states.']); 
                assert(all(mod(states,1)==0), 'State indices should be integers.');
            end
            if nargin<=2
            	confidence = 0.95; 
            end
            if nargin==1
               states = 1:obj.nx();
            end

            % 2. Prepare plot
            box = stateboundingbox(obj, confidence); 
            bound = [box(1,:)            fliplr(box(1,:))          ;
                     obj.x+box(2:end,:)  fliplr(obj.x-box(2:end,:))];

            % 3. Plot
            h = ishold(); 
            figure(gcf());
            for i=1:length(states)
                j = states(i); 
                p = plot(obj.t,obj.x(j,:));
                p.DisplayName = ['state ' num2str(j) ' (' num2str(confidence*100) '% interval)'];
                hold on;
                f = fill(bound(1,:),bound(j+1,:),p.Color,'FaceAlpha',0.15,'LineStyle','none');
                f.Annotation.LegendInformation.IconDisplayStyle = 'off';
                legend(); 
                xlabel('time'); 
                if ~h; hold off; end
                if i<length(states)
                    figure(); 
                end
            end
        end
        
        function plotmeasurements(obj, measurements, confidence)
        % PLOTMEASUREMENTS Generates a plot of the evolution of the measurements and their uncertainty
        %
        % PLOTMEASUREMENTS(KE) returns a separate figure with the evolution 
        % of every measurement together with its 95% confidence interval.
        %
        % PLOTMEASUREMENTS(KE, measurements) returns a separate figure with 
        % the evolution of every measurement of which the index is in 
        % 'measurements', e.g. PLOTMEASUREMENTS(KE, [1 3]) plots the
        % evolution of the first and third measurement. 
        %
        % PLOTMEASUREMENTS(KE, states, confidence) returns the same of the
        % previous syntax, but uses 'confidence' as cumulative probability
        % (confidence interval), e.g. PLOTMEASUREMENTS(KE, [1 2], 0.99) 
        % plots the 99% confidence interval of measurements 1 and 2.
        
            % 0. Check MATLAB version (>= R2014b)
            assert(~verLessThan('matlab','8.4'),'Uncertainty plots are only supported from MATLAB R2014b.'); 
        
            % 1. Generate state indices
            if nargin>=3
                assert(confidence > 0 && confidence < 1, 'Cumulative probability must be between 0 and 1.'); 
            end
            if nargin>=2
                assert(all(measurements<=obj.nout()), ['There are only ' num2str(obj.nout()) ' measurements.']); 
                assert(all(mod(measurements,1)==0), 'Measurement indices should be integers.');
                confidence = 0.95; 
            end
            if nargin<=2
            	confidence = 0.95; 
            end
            if nargin==1
               measurements = 1:obj.nout();
            end

            % 2. Prepare plot
            box = measurementboundingbox(obj, confidence); 
            bound = [box(1,:)            fliplr(box(1,:))          ;
                     obj.y+box(2:end,:)  fliplr(obj.y-box(2:end,:))];

            % 3. Plot
            h = ishold(); 
            figure(gcf());
            for i=1:length(measurements)
                j = measurements(i); 
                p = plot(obj.t,obj.y(j,:));
                p.DisplayName = ['measurement ' num2str(j) ' (' num2str(confidence*100) '% interval)'];
                hold on;
                f = fill(bound(1,:),bound(j+1,:),p.Color,'FaceAlpha',0.15,'LineStyle','none');
                f.Annotation.LegendInformation.IconDisplayStyle = 'off';
                legend(); 
                xlabel('time'); 
                if ~h; hold off; end
                if i<length(measurements)
                    figure(); 
                end
            end
        end
        
        function plotstateellipses(obj, states, confidence, n, log)
        % PLOTSTATEELLIPSES Generates a plot of the evolution of the confidence ellipses of two states
        %
        % PLOTSTATEELLIPSES(KE) returns a figure with the projection of
        % the 95% confidence hyperellipsoid on the plane spanned by the two
        % states of the model. This syntax only works if the model has
        % two states. 
        %
        % PLOTSTATEELLIPSES(KE, states) returns a figure with the projection
        % of the 95% confidence hyperellipsoid on the plane spanned by the 
        % two states of which the indices are in 'states', e.g.
        % plotstateelipses(KE, [1 3]) returns the ellipses that are the
        % projection of the hyperellipsoids on the plane (x1,x3). 
        %
        % PLOTSTATEELLIPSES(KE, states, confidence) returns the same as the
        % previous syntax, but uses 'confidence' as cumulative probability
        % (confidence interval), e.g. PLOTMEASUREMENTS(KE, [1 3], 0.99) 
        % returns the same as the previous syntax, but instead plots the
        % 99% confidence ellipse. 
        %
        % PLOTSTATEELLIPSES(KE, states, confidence, n) returns the same as 
        % the previous syntax, but allows to change the number of ellipses
        % 'n' that are being plotted. The default value is 10.
        %
        % PLOTSTATEELLIPSES(KE, states, confidence, n, log) returns the same 
        % as the previous syntax, but allows to set the boolean value 'log'
        % to 'false'. By default, the evolution of the ellipsoids is
        % plotted on a logarithmic time scale to be more informative, 
        % since the convergence typically has an exponential(-like) rate.
        % If you would like to see the evolution on a linear time scale,
        % set 'log' as 'false'. 
        
            % 0. Check MATLAB version (>= R2014b)
            assert(~verLessThan('matlab','8.4'),'Uncertainty plots are only supported from MATLAB R2014b.'); 
        
            % 1. Generate state indices
            if nargin>=4
                assert(n>=2, 'The number of ellipses should be greater or equal than 2 (beginning and end).'); 
                assert(mod(n,1)==0, 'The number of ellipses should be an integer.');
            end
            if nargin>=3
                assert(confidence>0 && confidence<1, 'Cumulative probability must be between 0 and 1.'); 
            end
            if nargin>=2
                assert(all(states<=obj.nx()), ['There are only ' num2str(obj.nx()) ' states.']); 
                assert(all(mod(states,1)==0), 'State indices should be integers.');
                assert(length(states)==2, 'You have to provide two state indices.'); 
            end
            if nargin<=4
                log = true;
            end
            if nargin<=3
                n = 10; 
            end
            if nargin<=2
            	confidence = 0.95; 
            end
            if nargin<2
                if obj.nx()==2
                    states = 1:2;
                else
                    error('You have to provide two state indices.'); 
                end
            end

            % 2. Prepare plot
            alpha = chi2inv(confidence, obj.nx()); 
            if log
                idx = round(logspace(0,log10(length(obj.t)),n)); 
            else
                idx = round(linspace(1,length(obj.t),n)); 
            end
            R = linspace(0,1,n);
            G = zeros(1,n);
            B = linspace(1,0,n);
            colormap([R(:), G(:), B(:)]);
            
            % 3. Plot
            h = ishold(); 
            figure(gcf());
            for i=1:length(idx)
                j = idx(i); 
                [V,D] = eig(obj.P(states,states,j));
                t = linspace(0,2*pi,100); 
                z1 = sqrt(alpha*D(1,1))*cos(t);
                z2 = sqrt(alpha*D(2,2))*sin(t);
                l = V*[z1(:)';z2(:)'];
                hold on; 
                p = plot(l(1,:),l(2,:));
                p.Color = [R(i) G(i) B(i)];
                p.DisplayName = ['t = ' num2str(obj.t(j))];
            end
            if log
                c = colorbar('YTick',log10(idx),'YTickLabel',obj.t(idx)); 
                caxis(log10([idx(1) idx(end)]));
            else
                c = colorbar('YTick',obj.t(idx),'YTickLabel',obj.t(idx)); 
                caxis([obj.t(idx(1)) obj.t(idx(end))]);
            end
            c.Label.String = 'time'; 
            if ~h; hold off; end
        end
        
        function [prob_nis, prob_snis] = analyzeconsistency(obj, confidence, M)
        % ANALYZECONSISTENCY Analyzes the consistency of the Kalman filter
        %
        % [probNIS,probSNIS] = ANALYZECONSISTENCY(KE) analyzes the
        % consistency of the Kalman filter during the experiment and
        % returns a visualization of the NIS and SNIS check. The default
        % confidence interval is chosen as 95% and the number of NIS
        % samples for calculation of the SNIS is 5. 
        %
        % [probNIS,probSNIS] = ANALYZECONSISTENCY(KE, confidence) returns
        % the same as the previous syntax, but allows to
        % choose the confidence interval as a probability between 0 and 1.
        %
        % [probNIS,probSNIS] = ANALYZECONSISTENCY(KE, confidence, M) 
        % returns the same as the previous syntax, but additionally allows
        % to change the number of subsequent NIS samples M to calculate 
        % the SNIS. 
        
            % 0. Check MATLAB version (>= R2014b), presence of innovations
            %    and parse input
            assert(~verLessThan('matlab','8.4'),'Uncertainty plots are only supported from MATLAB R2014b.'); 
            if nargin>=3
                assert(isnumeric(M) && mod(M,1)==0 && isscalar(M), 'The number of subsequent NIS samples should be a positive integer.'); 
                assert(confidence>0 && confidence<1, 'Cumulative probability must be between 0 and 1.'); 
            end
            if nargin>=2
                assert(confidence>0 && confidence<1, 'Cumulative probability must be between 0 and 1.'); 
            end
            if nargin<=2
                M = 5;
            end
            if nargin<=1
                confidence = 0.95; 
            end
           
            % 1. Calculate NIS and SNIS
            NIS = zeros(obj.N(),1);
            SNIS = zeros(obj.N()-M+1,1);
            for i=1:obj.N()
                NIS(i) = obj.nu(:,i)'/obj.S(:,:,i)*obj.nu(:,i);
            end
            for i=1:obj.N()-M+1
                SNIS(i) = sum(NIS(i:i+M-1)); 
            end
            
            % 2. Calculate and check distribution of NIS and SNIS
            min_nis = chi2inv((1-confidence)/2, obj.nout()); 
            max_nis = chi2inv(1-(1-confidence)/2, obj.nout()); 
            min_snis = chi2inv((1-confidence)/2, M*obj.nout()); 
            max_snis = chi2inv(1-(1-confidence)/2, M*obj.nout());
            prob_nis = sum(NIS>min_nis & NIS<max_nis)/length(NIS); 
            prob_snis = sum(SNIS>min_snis & SNIS<max_snis)/length(SNIS); 
            
            % 3. Plot the evolution of NIS and SNIS
            h = ishold(); 
            figure(); 
            subplot(211); 
            hold on;
            plot(obj.t,NIS,'k.');
            plot(obj.t,min_nis*ones(size(obj.t)),'g--');
            p = plot(obj.t,max_nis*ones(size(obj.t)),'g--');
            bound = [obj.t(1) min_nis ; obj.t(end) min_nis ; obj.t(end) max_nis ; obj.t(1) max_nis]; 
            f = fill(bound(:,1),bound(:,2),p.Color,'FaceAlpha',0.15,'LineStyle','none');
            f.Annotation.LegendInformation.IconDisplayStyle = 'off';
            title(['\bfNIS\rm: ' num2str(100*prob_nis) '% is in ' num2str(confidence*100) '% confidence interval']);
            xlabel('time'); 
            ylabel('NIS'); 
            subplot(212); 
            hold on;
            plot(obj.t(1:(end-M+1)),SNIS,'k.');
            plot(obj.t,min_snis*ones(size(obj.t)),'g--');
            plot(obj.t,max_snis*ones(size(obj.t)),'g--');
            bound = [obj.t(1) min_snis ; obj.t(end) min_snis ; obj.t(end) max_snis ; obj.t(1) max_snis]; 
            f = fill(bound(:,1),bound(:,2),p.Color,'FaceAlpha',0.15,'LineStyle','none');
            f.Annotation.LegendInformation.IconDisplayStyle = 'off';
            title(['\bfSNIS\rm: ' num2str(100*prob_snis) '% is in ' num2str(confidence*100) '% confidence interval']);
            xlabel('time'); 
            ylabel('SNIS'); 
            if ~h; hold off; end
        end
        
    end
    
   %% Private helper methods
    methods(Access=private)
        
        function checkInput(obj)
        % CHECKINPUT Checks the consistency of the input arguments
            assert(length(obj.t)>=2, 'At least two time samples are required.'); 
            assert(all(diff([size(obj.t,1), size(obj.x,2), size(obj.P,3), size(obj.R,3), size(obj.y,2), size(obj.u,2), size(obj.nu,2), size(obj.S,3)])==0), 'All inputs, outputs and states should have the same number of time samples.');
            assert(all(diff([size(obj.x,1), size(obj.P,1), size(obj.P,2)])==0), 'State covariance matrix size does not match the number of states.');
            assert(all(diff([size(obj.y,1), size(obj.nu,1)])==0), 'Number of innovations does not match the number of measurements.');
            assert(all(diff([size(obj.y,1), size(obj.R,1), size(obj.R,2)])==0), 'Measurement covariance matrix size does not match the number of measurements.');
            assert(all(diff([size(obj.nu,1), size(obj.S,1), size(obj.S,2)])==0), 'Innovation covariance matrix size does not match the number of innovations.');
        end
        
        function ny = nout(obj)
        % NOUT Returns the number of measurements
            ny = size(obj.y,1);
        end
        
        function nu = nin(obj)
        % NIN Returns the number of inputs
            nu = size(obj.u,1);
        end
        
        function nx = nx(obj)
        % NX Returns the number of states
            nx = size(obj.x,1);
        end
        
        function N = N(obj)
        % N Returns the number of timesamples
            N = size(obj.t,1);
        end
        
        function box = stateboundingbox(obj, confidence)
        % STATEBOUNDINGBOX Returns the axis-aligned bounding box (AABB) of the confidence (hyper)ellipsoid
        %
        % Due to the symmetry of the (hyper)ellipsoids, only one number per 
        % axis is returned (positive). 
        
            alpha = chi2inv(confidence, obj.nx()); 
            box = zeros(obj.nx()+1, obj.N());
            box(1,:) = obj.t'; 
            for i=1:obj.N()
                P = inv(obj.P(:,:,i)); 
                for j=1:obj.nx()
                    A = P; A(j,:) = []; A(:,j) = [];
                    b = P(:,j); b(j,:) = []; s = -A\b; 
                    sub = zeros(obj.nx(),1);
                    sub(1:j-1) = s(1:j-1); sub(j) = 1; sub(j+1:end) = s(j:end);
                    box(j+1,i) = sqrt(alpha/(sub'*P*sub));
                end
            end
        end
        
        function box = measurementboundingbox(obj, confidence)
        % MEASUREMENTBOUNDINGBOX Returns the axis-aligned bounding box (AABB) of the confidence (hyper)ellipsoid
        %
        % Due to the symmetry of the (hyper)ellipsoids, only one number per 
        % axis is returned (positive). 
        
            alpha = chi2inv(confidence, obj.nout()); 
            box = zeros(obj.nout()+1, obj.N());
            box(1,:) = obj.t'; 
            for i=1:obj.N()
                R = inv(obj.R(:,:,i)); 
                for j=1:obj.nout()
                    A = R; A(j,:) = []; A(:,j) = [];
                    b = R(:,j); b(j,:) = []; s = -A\b; 
                    sub = zeros(obj.nout(),1);
                    sub(1:j-1) = s(1:j-1); sub(j) = 1; sub(j+1:end) = s(j:end);
                    box(j+1,i) = sqrt(alpha/(sub'*R*sub));
                end
            end
        end
        
    end
    
    methods(Static)
        function obj = createfromQRC3()
        % CREATEFROMQRC3 Creates the KalmanExperiment data from the CSV file export in QRoboticsCenter for Control Theory assignments 4 and 5. 
        %
        % KE = CREATEFROMQRC3() opens an interactive interface to load data
        % from a CSV file recorded with QRoboticsCenter. This function is
        % particularly useful for assignment 3 of the Control Theory
        % assignments. 
       
            % label order: P, x, y, nu, S
            labels = {'P','x','y','nu','S'}; 
            
            % 1. Start with a message box explaining how the import
            % function works and select the CSV file
            nl = char(10); 
            msg = [ '\fontsize{11}' ...
                    'We will now import and parse the data from your Kalman filter experiment. ' ...
                    'Make sure the labels of your signals are as follows (without quotes):' nl nl ...
                    '-  The covariance on the state: ''' labels{1} '''' nl  ...
                    '-  The state: ''' labels{2} '''' ...
                    '-  The measurement: ''' labels{3} '''' nl ...
                    '-  The innovation: ''' labels{4} '''' nl ...
                    '-  The innovation covariance: ''' labels{5} '''' nl ...
                    '\bfNotes\rm:' nl ...
                    'If you forgot to set the correct labels and don''t want to redo your experiment, you can edit the labels manually by changing the appropriate line of the CSV file in a text editor.' ];
            opts = struct('WindowStyle', 'replace', 'Interpreter', 'tex'); 
            uiwait(msgbox(msg,'Importing data from QRoboticsCenter (assignment 3)','none',opts));
            [file,path] = uigetfile('*.csv'); 
            if ~file; return; end
            
            % 2. Parse the CSV
            datastartline = 3;  % depends on QRC version
            delimiter = ',';    % depends on QRC version
            readlabels = strsplit(fileread([path file]),{'\r','\n'}); readlabels = strsplit(readlabels{:,datastartline-1},[delimiter ' ']); % labels
            readdata = dlmread([path file], delimiter, datastartline-1, 0); % data
            [lia,locb] = ismember(labels,readlabels);
            assert(all(lia), ['The following labels were not found in your CSV file: ' strjoin(labels(~lia),', ') '.']); 
            readdata = readdata(:,[1 locb]);
                
            % 3. Request the measurement noise matrix
            if ~evalin('base','exist(''R'',''var'')')
                msg = '\fontsize{11}The import was successful! You now still have to provide the measurement covariance matrix. Click ''OK'' and type the covariance in the command window. If you define R in your workspace before running this function, this message will not appear anymore.';
                opts = struct('WindowStyle', 'replace', 'Interpreter', 'tex'); 
                uiwait(msgbox(msg,'Adding the measurement covariance','none',opts));
                R = input(['Type measurement covariance matrix here (''R ='')  and press enter:' nl '>> ']); 
            else
                R = evalin('base','R');
            end
            assert(isnumeric(R) && R>0 && isscalar(R), 'R should be a positive scalar.');
            R = repmat(R,[1 1 size(readdata,1)]);
            
            % 4. No control inputs - The MAVLink communication is not
            % fast enough to send more than 12 floats simultaneously at 100
            % Hz, so control signals have be calculated offline if
            % interested.
            u = zeros(0,size(readdata,1));
            
            % 5. Cast into a KalmanExperiment object
            obj = KalmanExperiment(readdata(:,1)/1000, readdata(:,3)', permute(readdata(:,2),[2 3 1]), readdata(:,4)', R, u, readdata(:,5)', permute(readdata(:,6),[2 3 1])); 
        
        end
        
        function obj = createfromQRC45()
        % CREATEFROMQRC45 Creates the KalmanExperiment data from the CSV file export in QRoboticsCenter for Control Theory assignments 4 and 5. 
        %
        % KE = CREATEFROMQRC45() opens an interactive interface to load data
        % from a CSV file recorded with QRoboticsCenter. This function is
        % particularly useful for assignments 4 and 5 of the Control Theory
        % assignments. 
        
            % label order: P11, P12, P13, P22, P23, P33, x1, x2, x3, y1, y2
            labels = {'P11','P12','P13','P22','P23','P33','x1','x2','x3','y1','y2'}; 
            
            % 1. Start with a message box explaining how the import
            % function works and select the CSV file
            nl = char(10); 
            msg = [ '\fontsize{11}' ...
                    'We will now import and parse the data from your Kalman filter experiment. ' ...
                    'Make sure the labels of your signals are as follows (without quotes):' nl nl ...
                    '-  The covariance matrix components:' nl  ...
                    '   \itP\rm_{11}:  ''' labels{1} ''',  \itP\rm_{12}:  ''' labels{2} ''',  \itP\rm_{13}:  ''' labels{3} ''',' nl ...
                    '   \itP\rm_{22}:  ''' labels{4} ''',  \itP\rm_{23}:  ''' labels{5} ''',  \itP\rm_{33}:  ''' labels{6} '''' nl ...
                    '   (The remaining 3 components follow from symmetry.)' nl nl ...
                    '-  The states:' nl ...
                    '   \itx\rm_{1}:  ''' labels{7} ''',  \itx\rm_{2}:  ''' labels{8} ''',  \itx\rm_{3}:  ''' labels{9} '''' nl nl ...
                    '-  The measurements:' nl ...
                    '   \ity\rm_{1}:  ''' labels{10} ''',  \ity\rm_{2}:  ''' labels{11} '''' nl nl ...
                    '\bfNotes\rm:' nl ...
                    ' 1. If you are using just one measurement, then the label ''' labels{11} ''' is still required and will be parsed, but you can of course ignore it yourself in the postprocessing. In that case, choose your measurement covariance matrix [\itR\rm_{\ity\rm_1} 0 ; 0 0].' nl ...
                    ' 2. Similarly, we always assume there are 3 states. If you do not use a state, set the corresponding row and column of your covariance matrix 0.' ...
                    ' 3. If you forgot to set the correct labels and don''t want to redo your experiment, you can edit the labels manually by changing the appropriate line of the CSV file in a text editor.' ];
            opts = struct('WindowStyle', 'replace', 'Interpreter', 'tex'); 
            uiwait(msgbox(msg,'Importing data from QRoboticsCenter (assignment 4-5)','none',opts));
            [file,path] = uigetfile('*.csv'); 
            if ~file; return; end
            
            % 2. Parse the CSV
            datastartline = 3;  % depends on QRC version
            delimiter = ',';    % depends on QRC version
            readlabels = strsplit(fileread([path file]),{'\r','\n'}); readlabels = strsplit(readlabels{:,datastartline-1},[delimiter ' ']); % labels
            readdata = dlmread([path file], delimiter, datastartline-1, 0); % data
            [lia,locb] = ismember(labels,readlabels);
            assert(all(lia), ['The following labels were not found in your CSV file: ' strjoin(labels(~lia),', ') '.']); 
            readdata = readdata(:,[1 locb]);
                
            % 3. Request the measurement noise matrix
            if ~evalin('base','exist(''R'',''var'')')
                msg = '\fontsize{11}The import was successful! You now still have to provide the 2 x 2 measurement covariance matrix. Click ''OK'' and type the covariance matrix in the command window. If you define R in your workspace before running this function, this message will not appear anymore.';
                opts = struct('WindowStyle', 'replace', 'Interpreter', 'tex'); 
                uiwait(msgbox(msg,'Adding the measurement covariance matrix','none',opts));
                R = input(['Type the 2 x 2 measurement covariance matrix here (''R ='')  and press enter:' nl '>> ']); 
            else
                R = evalin('base','R');
            end
            assert(isnumeric(R) && issymmetric(R) && all(eig(R)>=0) && ismatrix(R) && size(R,1)==size(R,2) && size(R,1)==2, 'R should be a 2 x 2 positive semidefinite matrix.');
            R = repmat(R,[1 1 size(readdata,1)]);
            
            % 4. No control inputs - The MAVLink communication is not
            % fast enough to send more than 12 floats simultaneously at 100
            % Hz, so control signals have be calculated offline if
            % interested.
            u = zeros(0,size(readdata,1));
            
            % 5. Cast into a KalmanExperiment object
            P = reshape([readdata(:,2:4) readdata(:,3) readdata(:,5:6) readdata(:,4) readdata(:,6) readdata(:,7)]', [3,3,size(readdata,1)]);
            obj = KalmanExperiment(readdata(:,1)/1000, readdata(:,8:10)', P, readdata(:,11:12)', R, u); 
            
        end
            
    end
        
end 
