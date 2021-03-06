%% Robot data

N = 2;
L = [1 1];  % link length m
m = [1 1];  % link mass in kg
r(:,:,1) = [L(1) 0 0]; % link center of mass wrt link coordinate frame [3x1]
r(:,:,2) = [L(2) 0 0]; % link center of mass wrt link coordinate frame [3x1]
I(:,:,1) = eye(3)*m(1)*L(1)^2; % link inertia matrix, symmetric [3x3], about link c. of mass
I(:,:,2) = eye(3)*m(2)*L(2)^2; % link inertia matrix, symmetric [3x3], about link c. of mass
Jm = [0 0]; % motor inertia (motor referred)
GR = [1 1]; % gear ratio
B  = [1 1]; % link viscous friction (motor referred) Nm/(rad/s)

%% Modified DH parameters

DH(1) = struct( 'd',        0,          ...
                'a',        0,          ...
                'alpha',    0,          ... 
                'theta',    [],         ...
                'type',     'R',        ...
                'm',        m(1),       ...
                'r',        r(:,:,1),   ...
                'I',        I(:,:,1),   ...
                'Jm',       Jm(1),      ...
                'G',        GR(1),      ...
                'B',        B(1));
            
DH(2) = struct( 'd',        0,          ...
                'a',        L(1),       ...
                'alpha',    0,          ...
                'theta',    [],         ...
                'type',     'R',        ...
                'm',        m(2),       ...
                'r',        r(:,:,2),   ...
                'I',        I(:,:,2),   ...
                'Jm',       Jm(2),      ...
                'G',        GR(2),      ...
                'B',        B(2));


%% Robot

for i = 1:N
    links{i} = Link('d', DH(i).d, 'a', DH(i).a, 'alpha', DH(i).alpha, 'm', DH(i).m, 'r', DH(i).r, ...
        'I', DH(i).I, 'Jm', DH(i).Jm, 'G', DH(i).G, 'B', DH(i).B, 'modified');
end


tool = transl([L(end), 0, 0]);
robot = SerialLink([links{:}], 'tool', tool, 'name', 'robot');

