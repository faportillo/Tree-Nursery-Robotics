function badPairs = ValidPath(route, C)
  badPairs = 0;
  for i = 3:2:(length(route) - 1)
    a = route(i - 1);
    b = route(i);
    if abs(a - b) ~= C.N
      fprintf('BAD: %d(%d) and %d(%d)\n', i-1, a, i, b);
      badPairs = badPairs + 1;
    end
  end
end