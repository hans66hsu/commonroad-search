.. _install-index:

===============
Installation
===============

This part gives the instructions on how to install the commonroad_search tool. The software is written in Python 3.7 and tested on Linux and macOS. The commonroad_search release 2019a is compatible with commonroad framework of release 2019. The usage of the Anaconda_ Python distribution is recommended. 

Hint
=====

During the using of commonroad_search, if there is any function or class that you can not find an explaination in this documentation, you may need to check the documentation of Commonroad-io_, Commonroad-collision-checker_ or Boundary, as this software is developed based on them.

.. _Anaconda: http://www.anaconda.com/download/#download

.. _Commonroad-io: https://commonroad-io.readthedocs.io/en/latest/ 

.. _Commonroad-collision-checker: https://commonroad.in.tum.de/static/docs/collision-checker/index.html

Setup of the conda environment
===============================
We recommand to use Anaconda to manage your environment so that even if you mess up something, you can always restart safely.
(https://conda.io/docs/user-guide/tasks/manage-environments.html)

To create a new environment:

.. code-block:: console

 　　　　　　　conda create -n commonroad-py37 python=3.7

Always activate your environment before you do anything related:

.. code-block:: console

  　　　　　 source activate commonroad-py37

Make your Jupyter Notebook aware of this new environment:

.. code-block:: console

	conda install ipykernel
	conda install ipython-autotime
	conda install ipywidgets
	conda install sphinx
	python -m ipykernel install --user --name commonroad-py37 --display-name "Python (commonroad-py37)"

If you need superuser rights for the last step try:

.. code-block:: console

  　　　　　　sudo /<root path to>/anaconda3/envs/commonroad-py37/bin/python -m ipykernel install --user --name commonroad-py37 --display-name "Python (commonroad-py37)"


Later in the Jupyter Notebook, you should select this kernel so that everything runs in this particular environment.

How to install
===============

This software has three main dependencies to install.

1. CommonRoad_io
==================

Go to folder commonRoad_root/tools/commonroad_io/Python and install the commonroad environment:

.. code-block:: console

 　　　　　　 conda install networkx
  　　　　　　python setup.py install


2. CommonRoad_Collision_Checker
===============================

Go to folder commonRoad_root/tools/commonroad-collision-checker/ and follow the Readme.

3. Boundary
============

Go to folder commonRoad_root/tools/ and follow the Readme file. Alternatively, you can also use the second method suggested in the trouble shooting.
 
How to use
============

Go to motionAutomata/notebooks and start Jupyter:

.. code-block:: python

  jupyter notebook



If you are using Anaconda environments like we suggested, remember to switch to the kernel of the Jupyter Notebook: click kernel -> change kernel -> Python(commonroad-py37).

In case that the jupyter widget is not working (section: Visualize planned trajectory):

.. code-block:: python

  jupyter nbextension install --py widgetsnbextension --user
 　jupyter nbextension enable widgetsnbextension --user --py


Implement your own search algorithm
====================================

Go to /primitive-planner/motionAutomata/Automata and open MotionPlanner.py. Insert the search algorithm of your choice in the function:

.. code-block:: python

  def search_alg(self, startSuccessor: List[MotionPrimitive], maxTreeDepth: int, status: dict)

And you can define your own cost function in the function:

.. code-block:: python

   def calc_heuristic_cost(self, path: List[State], curPos: State):

There are already 2 search algorithms (A star and Greedy-best-first search) implemented as examples for you. Please check the first cell of the notebook (primitive_planner/motionAutomata/notebooks/demo.ipynb) to see how to use them. 



Trouble Shooting
=================

Here are several suggestions for possible problems in the installation process.


1.

If you get errors when executing "python setup.py install", try the following equivalent command:

.. code-block:: console

  sudo /<root path to>/anaconda3/envs/commonroad-py37/bin/python setup.py install


2.

If there are errors related to not finding the boundary library (e.g. can not find module construction) when using the jupyter notebook, try to manually move all the contents of boundary folder (not simply copy the whole folder but to copy all files directly under this folder) into /path/to/your/anaconda/envs/lib/python3.7/site-packages.

The path typically looks like this: /home/USER-NAME/anaconda3/envs/commonroad-py37/lib/python3.7/site-packages

