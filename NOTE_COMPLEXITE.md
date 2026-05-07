# Livrable 3 — Note de Complexité

## 1) Limite de l'approche naïve

Avec l'approche double boucle, on compare chaque drone avec tous les autres:

\[
\sum_{i=1}^{n-1} (n-i) = \frac{n(n-1)}{2} \in \Theta(n^2)
\]

Pour \(n = 10\,000\):

\[
\frac{10000 \times 9999}{2} = 49\,995\,000
\]

Presque 50 millions de distances à calculer par cycle radar: non viable en temps réel embarqué.

## 2) Récurrence de l'algorithme Divide and Conquer

Le module applique:

- Tri initial: \(O(n \log n)\)
- Découpage récursif en deux sous-problèmes de taille \(n/2\)
- Coût linéaire de combinaison (partition + bande centrale): \(O(n)\)

Récurrence:

\[
T(n) = 2T(n/2) + O(n)
\]

Par le théorème maître (cas 2):

\[
T(n) \in \Theta(n \log n)
\]

## 3) Justification opérationnelle pour l'essaim

- Le passage de \(O(n^2)\) à \(O(n \log n)\) réduit drastiquement le nombre d'opérations.
- Pour \(n=10\,000\), \(\log_2(n)\approx 13.29\), donc l'échelle effective est de l'ordre de \(10^4 \times 13\), très inférieure à \(10^8\).
- Cette réduction asymptotique enlève la saturation CPU et permet de garder la décision d'évitement dans la fenêtre temps critique.

## 4) Conclusion

La preuve par récurrence + théorème maître valide mathématiquement que l'architecture choisie est en \(O(n \log n)\), ce qui répond à la contrainte anti-timeout et protège l'essaim contre le risque de crash en chaîne.
