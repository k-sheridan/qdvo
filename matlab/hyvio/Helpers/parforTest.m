n = 1e7;
a = zeros(n, 1);

disp('par start')

parfor (i = 1:n, 100)
    a(i) = rand();
end

disp('par fin')
disp('consec start')

for i = (1:n)
    a(i) = rand();
end
disp('consec end')