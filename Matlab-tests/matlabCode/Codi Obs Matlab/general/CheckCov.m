a = -1;
b = 1;
noiseSignal = a + (b-a).*rand(1,500);
test =[ 1 0.1 1 1 1 1]';

for i = 1:numel(test)
 m(i) = cov(noiseSignal * test(i));
end

m