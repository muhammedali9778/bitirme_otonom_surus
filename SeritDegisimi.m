classdef SeritDegisimi < matlab.System
    % Untitled3 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties

    end     

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        t_i
        t_f
        a0
        a1
        a2
        a3
        a4
        a5
        overtake
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function y_Hiz = stepImpl(obj, flag, sollama_flag, clock, y)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            if flag
                
                if isempty(obj.a5)
                    obj.t_i = clock;
                    
                    d_i =         [1  obj.t_i   obj.t_i^2   obj.t_i^3    obj.t_i^4    obj.t_i^5];
                    d_dot_i =     [0  1   2*obj.t_i   3*obj.t_i^2  4*obj.t_i^3  5*obj.t_i^4];
                    d_ddot_i =    [0  0   2     6*obj.t_i    12*obj.t_i^2  20*obj.t_i^3]; 
                    
                    obj.t_f = obj.t_i + 10; 
                    
                    d_f =         [1  obj.t_f   obj.t_f^2   obj.t_f^3    obj.t_f^4    obj.t_f^5];
                    d_dot_f =     [0  1   2*obj.t_f   3*obj.t_f^2  4*obj.t_f^3  5*obj.t_f^4];
                    d_ddot_f =    [0  0   2     6*obj.t_f    12*obj.t_f^2  20*obj.t_f^3]; 
                    
                    A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];
                    
                    B = [y; 0; 0; y+4; 0; 0];
                    
                    X = linsolve(A,B);
                    
                    obj.a0 = X(1);
                    obj.a1 = X(2);
                    obj.a2 = X(3);
                    obj.a3 = X(4);
                    obj.a4 = X(5);
                    obj.a5 = X(6);
                end
                
                if sollama_flag && isempty(obj.overtake)
                        obj.t_i = clock;
                        obj.overtake = true;
                        
                        d_i =         [1  obj.t_i   obj.t_i^2   obj.t_i^3    obj.t_i^4    obj.t_i^5];
                        d_dot_i =     [0  1   2*obj.t_i   3*obj.t_i^2  4*obj.t_i^3  5*obj.t_i^4];
                        d_ddot_i =    [0  0   2     6*obj.t_i    12*obj.t_i^2  20*obj.t_i^3];
                        
                        
                        obj.t_f = obj.t_i + 10; 
                        
                        d_f =         [1  obj.t_f   obj.t_f^2   obj.t_f^3    obj.t_f^4    obj.t_f^5]; 
                        d_dot_f =     [0  1   2*obj.t_f   3*obj.t_f^2  4*obj.t_f^3  5*obj.t_f^4];
                        d_ddot_f =    [0  0   2     6*obj.t_f    12*obj.t_f^2  20*obj.t_f^3]; 
                        
                        A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];
                        
                        B = [y; 0; 0; y-4; 0; 0];
                        
                        X = linsolve(A,B);
                        
                        obj.a0 = X(1);
                        obj.a1 = X(2);
                        obj.a2 = X(3);
                        obj.a3 = X(4);
                        obj.a4 = X(5);
                        obj.a5 = X(6);
                end

                if clock> obj.t_f
                    y_Hiz=0;
                else
                    t = clock;
                    
                    y_Hiz =  obj.a1  + 2*obj.a2*t +  3*obj.a3*t.^2  +  4*obj.a4*t.^3  +  5*obj.a5*t.^4;
     
                end
                
            else
                y_Hiz = 0;
            end

        end

        function resetImpl(obj)
        end

        function out = getOutputSizeImpl(obj)
            out = [1 1];
        end
        function out = getOutputDataTypeImpl(obj)
            out = "double";
        end

        function out = isOutputComplexImpl(obj)        
            out = false;
        end

        function out = isOutputFixedSizeImpl(obj)
            out = true;
        end
    end
end
