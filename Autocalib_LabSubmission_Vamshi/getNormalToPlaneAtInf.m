function findnormal = getNormalToPlaneAtInfinity(PPM, A)
%% PPM - Camera matrix
%   A - initial guess of intrinsics

%% Getting w
w = A * A';

%defining symbolic variables nx, ny and nz
nx = sym('nx', 'real'); ny = sym('ny', 'real'); nz = sym('nz', 'real');
l2 = sym('l2', 'real');

nor = [nx; ny; nz];             % Normal to plane at Inf.

% Now Forming Dual Absolute Quadric =====
Q = [w, (w * nor); (nor' * w), (nor' * w * nor)]; 

M2 = PPM(:, :, 2);              % getting one camera matrix
m2 = M2 * Q * M2';              % for autocalibration

% solving linear equn..
solution = solve(m2(1, 1) == (l2 * w(1, 1)), m2(2, 2) == (l2 * w(2, 2)), m2(3, 3) == (l2 * w(3, 3)), m2(1, 3) == (l2 * w(1, 3)));

findnormal = [double(solution.nx(1)) double(solution.ny(1)) double(solution.nz(1)) double(solution.l2(1))];
end