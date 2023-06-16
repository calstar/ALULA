

% Define parameters
N = 10;
radius = 0.5; % meters
height = 1.0; % meters
speed_of_sound = 340.29; % meters/second (at 20°C and sea level)
BCr = 'D'; % 'D' for Dirichlet, 'N' for Neumann (radial boundary condition)
BCz = 'D'; % 'D' for Dirichlet, 'N' for Neumann (axial boundary condition)

% Calculate resonant frequencies and mode shapes
[resonant_frequencies, mode_shapes] = cylindrical_resonant_modes(N, radius, height, speed_of_sound, BCr, BCz);

% Display results
disp('Resonant Frequencies:');
disp(resonant_frequencies);

disp('Mode Shapes:');
disp(mode_shapes);



function [resonant_frequencies, mode_shapes] = cylindrical_resonant_modes(N, radius, height, speed_of_sound, BCr, BCz)
    % Input parameters:
    % N: number of resonant modes to compute
    % radius: combustion chamber radius
    % height: combustion chamber height
    % speed_of_sound: speed of sound in the combustion chamber
    % BCr: radial boundary condition ('D' for Dirichlet, 'N' for Neumann)
    % BCz: axial boundary condition ('D' for Dirichlet, 'N' for Neumann)

    % Initialize variables
    resonant_frequencies = zeros(N, 1);
    mode_shapes = cell(N, 1);
    k = speed_of_sound / (2 * pi);
    modes = [];
    
    % Calculate resonant frequencies and mode shapes
    for m = 0:N
        % Find the first N zeros of the Bessel function J_m(x) or its derivative depending on the radial boundary condition
        if BCr == 'D'
            radial_zeros = besselzero(m, N, 1);
        elseif BCr == 'N'
            radial_zeros = besselzero(m, N, 1, 'Neumann');
        else
            error('Invalid radial boundary condition. Use "D" for Dirichlet or "N" for Neumann.');
        end
        
        for radial_index = 1:length(radial_zeros)
            lambda = radial_zeros(radial_index) / radius;
            
            for kz_index = 1:N
                % Check axial boundary condition
                if BCz == 'D'
                    k_z = (kz_index * pi) / height;
                elseif BCz == 'N'
                    k_z = ((2 * kz_index - 1) * pi) / (2 * height);
                else
                    error('Invalid axial boundary condition. Use "D" for Dirichlet or "N" for Neumann.');
                end
                
                k_r = sqrt(lambda^2 - k_z^2);
                
                % Calculate frequency
                frequency = k * sqrt(lambda^2) / (2 * pi);
                modes = [modes; frequency, m, radial_index, kz_index];
            end
        end
    end
    
    % Sort modes according to their frequencies
    sorted_modes = sortrows(modes, 1);
    
    % Store the first N resonant frequencies and mode shapes
    for mode_index = 1:N
        resonant_frequencies(mode_index) = sorted_modes(mode_index, 1);
        mode_shapes{mode_index} = struct('T', sorted_modes(mode_index, 2), 'R', sorted_modes(mode_index, 3), 'L', sorted_modes(mode_index, 4));
    end
end
