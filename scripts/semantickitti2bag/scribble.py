import numpy as np


#
# prs = [0.85502, 0.86829, 0.93980]
# rrs = [0.99354, 0.90617, 0.97081]
#
# prs = [0.94221, 0.95815, 0.91487]
# rrs = [0.93608, 0.57077, 0.95383]

# prs = [0.76319, 0.83293, 0.87731]
# rrs = [0.96799, 0.88371, 0.97008]

# prs = [0.869, 0.88170, 0.88406]
# rrs = [0.87880, 0.79981, 0.98248]

prs = [0.80689, 0.82038, 0.90624]
rrs = [0.98822, 0.95504, 0.99271]

count = 0
for pr, rr in zip(prs, rrs):
    print(count, 2*pr*rr/(pr+rr))
    count += 1
