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


### Example setup

> NOTE: this is just an example, there are many roads to nirvana

First step, create python virtualenv to install pybullet 

``` sh
# run this at the root of this repo 
virtualenv .venv
source .venv/bin/activate

```


Install pybullet
``` sh
python -m pip install --upgrade pip
pip install pybullet
```


After `pybullet` is installed, active the virtualenv whenever you need to, you should not need to re-install pybullet
``` sh
source .venv/bin/activate
```

Need some exports for Julia's `PyCall` to use the right python environment
``` sh
export PYCALL_JL_RUNTIME_PYTHON="python"
export PYCALL_JL_RUNTIME_PYTHON="${PWD}/.venv/bin/python3"
```

``` sh
julia --project=.
```
