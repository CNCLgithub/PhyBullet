# PhyBullet - Bullet backend for PhySMC

> NOTE!: This is a work in progress and the API is NOT stable

Implements the core interface of [PhySMC](https://github.com/CNCLgithub/PhySMC) for `BulletSim` and `BulletState`.

Currently the only `Element{BulletSim}` implemented is `RigidBody` but others (such as those with complex joints) are possible. 

See `examples/` and `src/` for docstrings. 


## Installation

Assumes that you have installed `pybullet` (e.g., `pip install pybullet`) and that the python path is exposed to `PyCall`. For more info see https://github.com/JuliaPy/PyCall.jl#python-virtual-environments


With the proper python env activated simply run 

``` julia-repl
pkg> add https://github.com/CNCLgithub/PhyBullet.git
```


