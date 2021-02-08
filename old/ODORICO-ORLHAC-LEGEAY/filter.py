def low_pass_filter(N, a):
	"""Se debarrasse des vibrations captees par l'accelerometre
	N liste des normes d'acceleration
	a norme mesuree"""
	lbda = 0.9 # Coefficient compris entre 0 et 1
	N.append(lbda*N[-1]+(1-lbda)*a)
	return N

