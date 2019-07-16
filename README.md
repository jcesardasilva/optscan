# optscan
Optmization of scan grid for scanning microscopy using a solver for the travelling salesman program implemented in Google OR-Tools 
[[https://developers.google.com/optimization/routing/tsp]]

** Installation
~python3 setup.py install --user~

** Running examples
You can edit the file round_roi_optim.py and run:
#+begin_SRC shell
python examples/basic_example.py
#+end_SRC

** Credits
Based on a code snippet originally posted here [[https://developers.google.com/optimization/routing/tsp]]
And also here [[https://github.com/google/or-tools/blob/stable/ortools/constraint_solver/samples/tsp_cities.py]]

The implementation here is published in the following work: 

da Silva, J.C., Guilloud, C., Hignette, O., Jarnias, C., Ponchut, C., Ruat, M., Labiche, J.-C., Pacureanu, A., Yang, Y., 
Salome, M. et al. (2019). J. Synchrotron Rad. 26, [[https://doi.org/10.1107/S1600577519006301]]
