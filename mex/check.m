%CHECK script to compare M-file and MEX-file versions of RNE

% load the model and remove non-linear friction
clear
puma560
puma560akb
rdh = nofriction(p560, 'coulomb');
rmdh = nofriction(p560m, 'coulomb');

% number of trials
n = 10;

fprintf('************************ normal case *****************************\n')
args = {};
fprintf('DH:  ')
check2(rdh, n, args);
fprintf('MDH: ')
check2(rmdh, n, args);

fprintf('************************ no gravity *****************************\n')
args = {[0 0 0]};
fprintf('DH:  ')
check2(rdh, n, args);
fprintf('MDH: ')
check2(rmdh, n, args);

fprintf('************************ ext force *****************************\n')
args = {[0 0 9.81], [10 10 10 10 10 10]'};
fprintf('DH:  ')
check2(rdh, n, args);
fprintf('MDH: ')
check2(rmdh, n, args);
