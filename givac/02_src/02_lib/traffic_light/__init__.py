from .intelligent_intersection import *
from .intelligent_intersection_with_modification import *
from .intelligent_intersection_with_dynamic_platoons import *

import datetime 
t_start = datetime.datetime.now()

print('Tempo inicial da execucao: ' + str(t_start))

def executionTime():
	"""Prints the time difference between start of the execution and current point
	"""
	print('Tempo de execucao ate este ponto: ' + str(datetime.datetime.now() - t_start))