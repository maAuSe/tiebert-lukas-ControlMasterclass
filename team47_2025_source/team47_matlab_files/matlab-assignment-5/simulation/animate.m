function animate(xsim, params, colors, axislimits)
    
      N = params.N;
      d = params.d;
      fpz = params.fpz;
      r = params.r;
      Ts = params.Ts;
      steps_per_frame = 5;
  
      figure(1);
      xlabel('x position (m)')
      ylabel('z position (m)')  
      axis(axislimits);
      grid on    
  
      for i=2:N
  
          % Measurements
          % ------------
          px    = xsim(1, i);
          pz    = xsim(2, i);
          pitch = xsim(5, i);
  
  
          % Visualization
          % -------------
          if not(mod(i,steps_per_frame))  % Display only subset of all time data.
  
              Ry = [ cos(pitch), sin(pitch);
                    -sin(pitch), cos(pitch)];
              R = Ry;
  
              fp = R*[[-d; fpz] [-d; 0] [d; 0] [d; fpz]];  % framepoints
  
              frame = {fp(1,:)', fp(2,:)'};
  
              t = -r:0.001:r;
              rp1 = R*[-d+t; fpz*ones(1,length(t))];
              rp2 = R*[ d+t; fpz*ones(1,length(t))];
              rotors = {{rp1(1,:),rp1(2,:)},...
                        {rp2(1,:),rp2(2,:)}};
              figure(1)
              clf
              xlabel('x position (m)')
              ylabel('z position (m)')          
              axis(axislimits);
  
              grid on
              hold on
              plot(px+frame{1}, pz+frame{2}, 'Color', colors.red)
              plot(px+[rotors{1}{1};rotors{2}{1}]',...
                   pz+[rotors{1}{2};rotors{2}{2}]', 'Color', colors.red)
              hold off
          end
          pause(Ts)  
      end
  
  end