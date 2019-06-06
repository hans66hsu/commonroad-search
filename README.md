# Efficient Online Planning Using Maneuver Automata

The software is written in Python 3.7 and tested on Linux and macOS.

## Before you start
We recommand to use Anaconda to manage your environment so that even if you mess up something, you can always restart safely.
(https://conda.io/docs/user-guide/tasks/manage-environments.html)

To create a new environment:
```sh
  $ conda create -n commonroad-py37 python=3.7
```
Always activate your env before you do anything related:
```sh
  $ source activate commonroad-py37
```
Make your Jupyter Notebook aware of this new env:
```sh
  $ conda install ipykernel
  $ conda install ipython-autotime
  $ conda install ipywidgets
  $ conda install sphinx
  $ python -m ipykernel install --user --name commonroad-py37 --display-name "Python (commonroad-py37)"
```

If you need superuser rights for the last step try:
```sh
  $ sudo /<root path to>/anaconda3/envs/commonroad-py37/bin/python -m ipykernel install --user --name commonroad-py37 --display-name "Python (commonroad-py37)"
```


Later in the Jupyter Notebook, you should select this kernel so that everything runs in this particular environment.


## How to install
This software has three main dependencies to install.

### 1. CommonRoad
Go to folder commonRoad_root/tools/commonroad_io/Python and install the commonroad environment:
```sh
  $ conda install networkx
  $ python setup.py install
```


### 2. CommonRoad-Collision-Checker
Go to folder commonRoad_root/tools/commonroad-collision-checker/ and follow the Readme file.

### 3. Boundary
Go to folder commonRoad_root/tools/ and follow the Readme file. Alternatively, you can also use the second method suggested in the troubleshoot.

## How to use

Go to motionAutomata/notebooks and start Jupyter:
```sh
  $ jupyter notebook
```

If you are using Anaconda environments like we suggested, remember to switch to the kernel of the Jupyter Notebook: click kernel -> change kernel -> Python(commonroad-py37).

In case that the jupyter widget is not working (section: Visualize planned trajectory):
```sh
  $ jupyter nbextension install --py widgetsnbextension --user
  $ jupyter nbextension enable widgetsnbextension --user --py
```

## Implement your own search algorithm

Go to motionAutomata/Automata and open MotionPlanner.py. Insert the search algorithm of your choice in the function:

```python
  def search_alg(self, startSuccessor, maxTreeDepth, status)
```

There are already 2 search algorithms (A star and Greedy-best-first search) implemented as examples for you. Please check the first cell of the notebook to see how to use them.



## Troubleshoot:

**1**
If you get errors when executing "python setup.py install", try the following equivalent command:
```
  $ sudo /<root path to>/anaconda3/envs/commonroad-py37/bin/python setup.py install
```

**2**
If there are errors related to not finding the boundary library (e.g. can not find module construction) when using the jupyter notebook, try to manually move all the contents of boundary folder (not simply copy the whole folder but to copy all files directly under this folder) into /path/to/your/anaconda/envs/lib/python3.7/site-packages.

The path typically looks like this: /home/USER-NAME/anaconda3/envs/commonroad-py37/lib/python3.7/site-packages
