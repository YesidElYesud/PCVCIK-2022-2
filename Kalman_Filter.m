classdef Kalman_Filter < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime
    % Filtro Kalman para estimar los estados del modelo UVA/Padova

    % Public, tunable properties
    properties
        Q
        R         
    end
    
    properties(DiscreteState)
        P
        x
    end

    % Pre-computed constants
    properties(Access = private)
        Gm
        Hm
        Cm
    end
      

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [y_est, dy_est] = stepImpl(obj,y,u)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            % Corrección
            K = obj.P*obj.Cm'/(obj.Cm*obj.P*obj.Cm'+obj.R);                  % Innovación           
            x_est = obj.x + K*(y - obj.Cm*obj.x);                            % x[n|n]
            P_est = (eye(13)-K*obj.Cm)*obj.P*(eye(13)-K*obj.Cm)'+K*obj.R*K'; % P[n|n]
            P_est = (P_est + P_est')/2;
            
            % Salida
            y_est = obj.Cm*x_est;                               % Salida estimada
            dy_est = obj.Cm*((obj.Gm-eye(13))*x_est + obj.Hm*[u;1]);
            
            % Predicción
            obj.x = obj.Gm*x_est + obj.Hm*[u(2);u(1);1];                      % x[n+1|n]
            obj.P = obj.Gm*P_est*obj.Gm' + obj.Q;   % P[n+1|n]
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties

            obj.Gm = [0.943039537893160   0.111486794890384  -0.000501899484855  -0.000023619404189   0.000000209519221   0.000017574017788   0.001620890142795  -0.008038239882059  -0.000037322516442  -0.002055831090970  -0.000000378252043  -0.000001956916345                   0
                      0.053246307909252   0.872504282681993  -0.011871908148131  -0.000736262023126   0.000000003037851   0.000000339457323   0.000047183399219  -0.000228271823708  -0.000000711417929  -0.033058984622272  -0.000012128703262  -0.000062504451405                   0
                                      0                   0   0.732021109305409   0.134073283703363                   0                   0                   0                   0                   0                   0   0.002556237988320   0.012915082144601                   0
                                      0                   0   0.158497813952848   0.618734839580989                   0                   0                   0                   0                   0                   0   0.000269616522359   0.001379188363795                   0
                                      0                   0                   0                   0   0.965246245705849                   0                   0                   0                   0                   0                   0                   0                   0             
                                      0                   0                   0                   0   0.034388313703668   0.979165649399429                   0                   0                   0                   0                   0                   0                   0             
                                      0                   0                   0                   0   0.000346578047674   0.019244192425563   0.851830520551867                   0                   0                   0                   0                   0                   0             
                                      0                   0   0.000625375772286   0.000038265816349                   0                   0                   0   0.990831932023669   0.009125912214459                   0   0.000000629620088   0.000003245229041                   0
                                      0                   0   0.129279350956946   0.011666327187569                   0                   0                   0                   0   0.990831932023669                   0   0.000201263686759   0.001030635459362                   0                
                                      0                   0   0.647058806006423   0.058764850782706                   0                   0                   0                   0                   0   0.954067603105833   0.001014422181767   0.005194240504648                   0
                                      0                   0                   0                   0                   0                   0                   0                   0                   0                   0   0.982315327167075                   0                   0
                                      0                   0                   0                   0                   0                   0                   0                   0                   0                   0   0.014702640154226   0.984905053783262                   0
                      0.343878888987482   0.023754478575008  -0.000060048869009  -0.000002345995977   0.000000024754073   0.000002660586335   0.000346343776909  -0.001681203434848  -0.000005584791541  -0.000311391981263  -0.000000036946457  -0.000000191588103   0.367879441171442];              
                                                                                                                                                                                        
            obj.Hm = [-0.000000076923931   0.000000053107382   3.520247014621363
                      -0.000003098974086   0.000000000616071  -0.359265557750322
                       0.001330242294224                   0                   0
                       0.000095619478280                   0                   0
                                       0   0.982520682409587                   0
                                       0   0.017356930401615                   0
                                       0   0.000117620591994                   0
                       0.000000159515068                   0                   0
                       0.000068519551190                   0                   0
                       0.000346488599213                   0  -4.856652831876819
                       0.991131368213438                   0                   0
                       0.007391984015882                   0                   0
                      -0.000000006372546   0.000000005152860   0.736919658425075];  
            
            obj.Cm = [0     0     0     0     0     0     0     0     0     0     0     0     1];                         
            obj.Q
            obj.P = obj.Q;
            obj.x = [216.6795   86.8199    6.4165    3.0342         0         0         0  105.7348  105.7348         0   85.9527   84.4699  122.0211]';    

        end
        
        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(~,name)
            if strcmp(name,'P')
                sz = [13 13];
                dt = 'double';
                cp = false;
            elseif strcmp(name,'x')
                sz = [13 1];
                dt = 'double';
                cp = false;
%             elseif strcmp(name,'x_past')
%                 sz = [10 1];
%                 dt = 'double';
%                 cp = false;
             else
                error(['Error: Incorrect State Name: ', name.']);
            end
        end        
               
        function [o1, o2] = getOutputDataTypeImpl(~)
            o1 = 'double';
            o2 = 'double';
%             o3 = 'double';
        end
        
        function [o1, o2] = getOutputSizeImpl(~)
            o1 = 1;
            o2 = 1;
%             o3 = 1;
        end
        
        function [o1, o2] = isOutputComplexImpl(~)
            o1 = false;
            o2 = false;
%             o3 = false;
        end
        
        function [o1, o2] = isOutputFixedSizeImpl(~)
            o1 = true;
            o2 = true;
%             o3 = true;
        end
        
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete',...
            'SampleTime',1,'OffsetTime',0);
        end
       
    end
end