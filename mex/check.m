%CHECK script to compare M-file and MEX-file versions of RNE

% load the model and remove non-linear friction
puma560akb
p560 = nofriction(p560, 'coulomb');

% number of trials
n = 10;

% create random points in state space
q = rand(n, 6);
qd = rand(n, 6);
qdd = rand(n, 6);

args = {[0 0 0]', [1 2 3 4 5 6]};

% test M-file
tic;
tau = rne(p560m, q, qd, qdd, args{:});
t = toc;

% test MEX-file
tic;
tau_f = frne(p560m, q, qd, qdd, args{:});
t_f = toc;

% print comparative results
fprintf('Speedup is %.0f\n', t/t_f);
fprintf('Worst case error is %f\n', max(max(abs(tau-tau_f))));
