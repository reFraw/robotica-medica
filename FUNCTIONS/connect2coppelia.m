  function connection = connect2coppelia( ...
    q, ...
    simulationTime, ...
    samplingTime)
%
% CONNECT2COPPELIA This function automates the connection with CoppeliaSim.
%
%   INPUT:
%           1. q : Matrix whose columns represent the joint variables for each sampling step.
%           2. SimulationTime : Simulation time in the CoppeliaSim
%           environment.
%           3. SamplingTime : Sampling time in the CoppeliaSim enviroment.
%
%   OUTPUT:
%           1. connection : Object that maintains the connection.
%

    porta = 19997;
    
    % Individuo durata della simulazione MATLAB
    sizeQ = size(q);
    planningPoints = sizeQ(2);
    
    % Individuo durata della simulazione CoppeliaSim
    t = 0 : samplingTime : simulationTime;
    N = length(t);
    
    if planningPoints > N
        % Se il Coppelia SimulationTime è inferiore del MATLAB
        % SimulationTime forza la durata della simulazione Coppelia a
        % MATLAB SimulationTime.
        N = planningPoints;
    else
        % Se il Coppelia SimulationTime è maggiore del MATLAB
        % SimulationTime, aggiunge colonne alla matrice q (ultimo valore assunto) fino a coprire
        % tutto il tempo di simulazione.
        addPoint = N - planningPoints;
        addQ = repmat(q(:,end), 1 ,addPoint);
        q = [q addQ];
    end

    connection = 1;
    
    clc
    fprintf('----------------------');
    fprintf('\n simulation started ');
    fprintf('\n trying to connect...\n');
    [clientID, vrep] = StartVrep(porta, samplingTime);
    
    handle_joint = my_get_handle_Joint(vrep,clientID); 

    for i=1:N
    
        % DH -> Kinova conversion
        q_jaco(:,i) = mask_q_DH2Jaco(q(:,i));
        my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,i));
       
        pause(samplingTime);

    end

    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
    DeleteVrep(clientID, vrep);
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [clientID, vrep ] = StartVrep(porta, Ts)

vrep = remApi('remoteApi');   
vrep.simxFinish(-1);        
clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);

if (clientID>-1)
    disp('remote API server connected successfully');
else
    %disp('failed connecting to remote API server');
    DeleteVrep(clientID, vrep);
end
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, Ts, vrep.simx_opmode_oneshot_wait);
vrep.simxSetBooleanParameter(clientID, vrep.sim_boolparam_realtime_simulation, true, vrep.simx_opmode_oneshot_wait);
vrep.simxSetBooleanParameter(clientID, vrep.sim_boolparam_dynamics_handling_enabled, false, vrep.simx_opmode_oneshot_wait);

vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
end

function DeleteVrep(clientID, vrep)

vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);  
vrep.delete();              
disp('simulation ended');

end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)

[m,n] = size(q);
for i=1:n
    for j=1:m
        err = vrep.simxSetJointPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
        if (err ~= vrep.simx_error_noerror)
            fprintf('failed to send joint angle q %d \n',j);
        end
    end
end

end

function handle_joint = my_get_handle_Joint(vrep,clientID)

[~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_1',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_2',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_3',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_4',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_5',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_6',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)

[~,n] = size(q);

for i=1:n
    joints_positions = vrep.simxPackFloats(q(:,i)');
    [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);
    
    if (err~=vrep.simx_return_ok)
        fprintf('failed to send the string signal of iteration %d \n',i);
    end
end
pause(8);

end

function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)

for j=1:n
    vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
end

pause(0.05);

for j=1:n
    [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
end

if (err(j)~=vrep.simx_return_ok)
    fprintf(' failed to get position of joint %d \n',j);
end

end

function q_Jaco = mask_q_DH2Jaco(q)
   
  
    q_Jaco(1) = -q(1);
    q_Jaco(2) = q(2) + pi;
    q_Jaco(3) = q(3);
    q_Jaco(4) = q(4);
    q_Jaco(5) = q(5);
    q_Jaco(6) = q(6) + pi;
    q_Jaco(7) = -q(7);
    

end

function q_DH = mask_q_Jaco2DH(q)
    
    
    q_DH(1) = -q(1);
    q_DH(2) = q(2) - pi;
    q_DH(3) = q(3);
    q_DH(4) = q(4);
    q_DH(5) = q(5);
    q_DH(6) = q(6) - pi;
    q_DH(7) = -q(7);
    

end