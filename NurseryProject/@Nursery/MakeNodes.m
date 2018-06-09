function [nodes, DMAT] = MakeNodes(C)
  %implement defining the locations of each node and the cost of traveling
  %between them

X = 1;
Y = 2;
K = C.K + 1;
endI = 2 * K + 2;
swX = C.swX; swY = C.swY;
xs = ((C.W / 2):C.W:(C.W / 2 + C.W * (K - 1))) + swX - C.W;
nodes = [C.start(X), xs, xs, C.start(X);
  C.start(Y), (zeros(1, K) + swY), (zeros(1, K) + C.RL + swY), C.start(Y)];

fprintf('Start and End: (%0.2f, %0.2f) (%0.2f, %0.2f)\n', nodes(X, 1), nodes(Y, 1),...
  nodes(X, endI), nodes(Y, endI))
fprintf('Bottom Nodes: ')
for i = 2:(K + 1)
  fprintf('(%0.2f, %0.2f) ', nodes(X, i), nodes(Y, i))
end
fprintf('\nTop Nodes: ')
for i = (K + 2):(endI - 1)
  fprintf('(%0.2f, %0.2f) ', nodes(X, i), nodes(Y, i))
end
fprintf('\n')

DMAT = zeros(endI);

for i = 2:(K + 1)               %all lower nodes
  for j = (K + 2):(2 * K + 1) %visit upper nodes
    if (j - i) == K             %nodes are in same row
      DMAT(i, j) = -C.MULT;       %cost to go through row is 0
      DMAT(j, i) = -C.MULT;
    else                    %diagonal traversal not allowed
      DMAT(i, j) = C.HUGE;  %cost is unreasonably large
      DMAT(j, i) = C.HUGE;
    end
  end
end

%headlands turning costs
for i = 2:K
  for j = (i + 1):(K + 1)   
    d = abs(i - j);
    if d * C.W >= 2 * C.Rmin   %Pi turn
      DMAT(i, j) = C.MULT * (d * C.W + (pi - 2) * C.Rmin);
    else                       %Omega turn
      DMAT(i, j) = C.MULT * (3 * pi * C.Rmin - 2 * C.Rmin *...
        acos(1 - (2 * C.Rmin + d * C.W) ^ 2 / (8 * C.Rmin ^ 2)));
    end
    
    DMAT(j, i) = DMAT(i, j);
    DMAT(i + K, j + K) = DMAT(i, j);
    DMAT(j + K, i + K) = DMAT(i, j);
  end
end

%Start and end points
for j = 2:(endI - 1)
  DMAT(1, j) = abs(nodes(X, 1) - nodes(X, j)) + abs(nodes(Y, 1) - nodes(Y, j));
  DMAT(j, 1) = DMAT(1, j);
  DMAT(endI, j) = abs(nodes(X, endI) - nodes(X, j)) + ...
    abs(nodes(Y, endI) - nodes(Y, j));
  DMAT(j, endI) = DMAT(endI, j);
end

DMAT(1, endI) = C.HUGE;
DMAT(endI, 1) = C.HUGE;


end