This code can find the shortest path (geodesic) for any 3D surface.
The examples below for are 3 simple surfaces: sphere, hyperbola, and parabola. This code operates as so:

1) create the shortest path using a euclidean distance from current step to final step.
2) Go along this path in 3D space for a short distance
3) Find the optimal point along the 3D surface via lagrange multipliers
4) Continue until sufficiently close to the end.

Example 1: Hyperbola

![Hyperbola](https://raw.githubusercontent.com/jah-26603/numerical_geodesics/main/Hyperbola.jpg)


Example 2: Sphere

![Hyperbola](https://raw.githubusercontent.com/jah-26603/numerical_geodesics/main/trajecotryplotted.jpg)


Example 3: Parabaloid

![Hyperbola](https://raw.githubusercontent.com/jah-26603/numerical_geodesics/main/parabaloid%20example.jpg)
