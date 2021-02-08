import numpy as np
lbda = 1.01


def detect_collision(acc):
	"""Détecte une collision (booleen) si un 'pic' est detecte dans la courbe de l'acceleration"""
	return np.linalg.norm(acc) > 600


