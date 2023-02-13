<h1 align="center"> Modified RRT Path Finding </h1>

<div align="center">
  <img width="500" src="https://user-images.githubusercontent.com/39476147/218465217-fb1fb7b0-9d08-4bb0-a63d-3a51b23a8f39.gif" />
</div>

<div>&nbsp&nbsp</div>
<p align="center"> Robot path planning algorithm based on RRT. </p>
<p align="center"> With ellipsoidal heuristics. </p>
<p align="center"> Modified version for pushing objects. </p>

<div>&nbsp&nbsp&nbsp</div>

### How to run
1. Clone to local repository
2. Execute:

```
python3 src/main.py
```

Alternatively, also run
```
python3 src/RRT.py
```

Or
```
python3 src/RRT_star.py
```

<div>&nbsp&nbsp&nbsp</div>

### Caveats
This repo demonstrates ideas for optimising path finding algorithm.

It is not refined for reliability.

This will not be maintained any further due to these reasons:
* Python is slow and resource intensive compared cpp implementation
* There is no check for unsolvable maps
