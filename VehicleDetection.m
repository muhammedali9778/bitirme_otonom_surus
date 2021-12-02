classdef VehicleDetection < matlab.System 
    % Untitled2 Add summary here
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
        sensor
        frontcardetector
        sidecardetector
        front_vehicle_cache
        side_vehicle_cache
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            Sensor = load('Sensor.mat');
            FrontCarDetector = load('FrontCarDetector.mat');
            SideCarDetector = load('SideCarDetector.mat');
            obj.sensor = Sensor.sensor;
            obj.frontcardetector = FrontCarDetector.frontcardetector;
            obj.sidecardetector = SideCarDetector.sidecardetector;
            obj.front_vehicle_cache = zeros(1, 10);
            obj.side_vehicle_cache = zeros(1, 10)-99;
        end

        function [vehicles_bboxes, vehicles_distances, cache] = stepImpl(obj,current_frame)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            % Detection the Front Vehicle
            [bboxes_front_vehicle, scores_front] = detect(obj.frontcardetector, current_frame);
            bboxes_front_vehicle = bboxes_front_vehicle(scores_front==max(scores_front), :);
            if ~isempty(bboxes_front_vehicle)
                front_vehicle_location_img = [min(bboxes_front_vehicle(1)+0.5*bboxes_front_vehicle(3), 1279),...
                    min(bboxes_front_vehicle(2)+bboxes_front_vehicle(4), 959)];
                front_vehicle_location_veh = imageToVehicle(obj.sensor, front_vehicle_location_img);
                front_vehicle_distance = sqrt((front_vehicle_location_veh(1))^2+...
                   (front_vehicle_location_veh(2))^2)*1.25+6;
            else
                bboxes_front_vehicle = [0, 0, 0, 0];
                front_vehicle_distance = -99;
            end
            for m = 1:9
                obj.front_vehicle_cache(m) = obj.front_vehicle_cache(m+1);
            end
            obj.front_vehicle_cache(end) = front_vehicle_distance;
            
                
            % Detection the Side Vehicle
            [bboxes_side_vehicle, scores_side] = detect(obj.sidecardetector, current_frame);
            bboxes_side_vehicle = bboxes_side_vehicle(scores_side==max(scores_side), :);
            if ~isempty(bboxes_side_vehicle)
                side_vehicle_location_img = [min(bboxes_side_vehicle(1)+0.5*bboxes_side_vehicle(3), 1279),...
                    min(bboxes_side_vehicle(2)+bboxes_side_vehicle(4), 959)];
                side_vehicle_location_veh = imageToVehicle(obj.sensor, side_vehicle_location_img);
                side_vehicle_location_Ydirection = side_vehicle_location_veh(2);
            else
                bboxes_side_vehicle = [0, 0, 0, 0];
                side_vehicle_location_Ydirection = -99;
            end
            for m = 1:9
                obj.side_vehicle_cache(m) = obj.side_vehicle_cache(m+1);
            end
            obj.side_vehicle_cache(end) = side_vehicle_location_Ydirection;
            vehicles_bboxes = [bboxes_front_vehicle; bboxes_side_vehicle];
            vehicles_distances = [front_vehicle_distance; side_vehicle_location_Ydirection];
            cache = [obj.front_vehicle_cache; obj.side_vehicle_cache];
        end
        % -----------------------------------------------------------------
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon(".\img\acf.PNG");
        end

        function [out,out2,out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [2 4];
            out2 = [2 1];
            out3 = [2, 10];
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2,out3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out,out2,out3] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out,out2,out3] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
