# optscan
Optmization of scan grid for scanning microscopy using a solver for the travelling salesman program implemented in Google OR-Tools 
<https://developers.google.com/optimization/routing/tsp>
The implementation here is published here [[1]](https://doi.org/10.1107/S1600577519006301). We will be happy with your citation if you use it. 

Installation
------------

Installation should be as simple as :

   `sudo python3 setup.py install`

or, for local installation, using the flag --user :

   `python3 setup.py install --user`
   
Running examples
----------------

You can edit the file round_roi_optim.py and run:

  `python3 examples/round_roi_optim.py`

Dependencies
------------

Optscan depends on standard python packages:
 * numpy
 * matplotlib
 * ortools
 * python >=3.6

Credits
-------

Based on a code snippet originally posted here <https://developers.google.com/optimization/routing/tsp>

And also here <https://github.com/google/or-tools/blob/stable/ortools/constraint_solver/samples/tsp_cities.py>

References
----------

[1]: da Silva, J.C., Guilloud, C., Hignette, O., Jarnias, C., Ponchut, C., Ruat, M., Labiche, J.-C., Pacureanu, A., Yang, Y., Salome, M. et al. (2019). J. Synchrotron Rad. 26, <https://doi.org/10.1107/S1600577519006301>
