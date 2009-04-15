# test harness for jacob_dot

if exist('p560') == 0,
    puma560
end

e = 1e-4;
dq=[0 0 0 0 1 0];

jacob_dot(p560, qn, dq)
(jacob0(p560, qn+dq*e)-jacob0(p560, qn))/e

