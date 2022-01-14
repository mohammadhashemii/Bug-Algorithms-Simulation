import numpy as np
from initialization import * 
import math

def run(runfile):
    with open(runfile, 'r') as rnf:
        exec(rnf.read())

algo = 'BUG0'
print('RUNNING {}...'.format(algo))
if algo == 'BUG0':
	run('BUG0_controller.py')
	
elif algo == 'BUG1':
	run('BUG1_controller.py')
	
elif algo == 'BUG2':
	run('BUG2_controller.py')
	
else:
	print('UNKNOWN ALGORITHM ENTERD!')