function DH = jaco2DH(configuration)
%
% JACO2DH This function returns the Denavit-Hartenberg's table for Kinova
% Jaco2 given an initial joint configuration.
%
%   INPUT :
%           1. configuration: 7x1 array.
%   
%   OUTPUT:
%           1. DH: 7x4 matrix [a, alfa, d, theta].
%

    a = zeros(7,1);
    alfa = zeros(7,1);
    alfa(1:6) = pi/2;
    d = [0.2755 0 -0.41 -0.0098 -0.3111 0 0.2638]';

    DH = [a alfa d configuration];

end