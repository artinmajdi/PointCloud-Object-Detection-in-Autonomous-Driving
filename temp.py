import numpy as np
from timeit import timeit



items = [20, 18, 14, 17, 20, 21, 15]


mn, mx = items[0], 0

for it in items[1:]:
    mn = min(mn, it)
    mx = max(it - mn , mx)


print(mx)