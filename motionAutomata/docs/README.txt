
CommonRoad-search
==================

Structure
---------

Sphinx and ReST is used to write the documentation. All source files are in the folder source. The structure of the documentation is:

* source/install - the installation guide

* source/api - API documentation

* source/commonroad_search.rst - documentation master file

* source/troubleshooting.rst - several tips for trouble shooting

* source/conf.py - the sphinx configuration

* Makefile - entry points for building the docs


Dependency installation:
'''
conda install -c astropy sphinx-automodapi
conda install sphinx-autodoc-typehints
'''

To build the documentation, just do:

'''
mkdir build
make html
'''
and the documentation will be generated in build/html/commonroad-search.html
