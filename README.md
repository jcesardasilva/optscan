# optscan
Optmization of scan grid for scanning microscopy using a solver for the travelling salesman program implemented in Google OR-Tools 
<https://developers.google.com/optimization/routing/tsp>

Installation
------------

Installation should be as simple as :

   `sudo python setup.py install`

or, as a user, :

   `python setup.py install --user`
   
Running examples
----------------

You can edit the file round_roi_optim.py and run:

  `python examples/basic_example.py`


Credits
-------

Based on a code snippet originally posted here <https://developers.google.com/optimization/routing/tsp>
And also here <https://github.com/google/or-tools/blob/stable/ortools/constraint_solver/samples/tsp_cities.py>

The implementation here is published in the following work, which we will be happy with your citation if you use it: 

da Silva, J.C., Guilloud, C., Hignette, O., Jarnias, C., Ponchut, C., Ruat, M., Labiche, J.-C., Pacureanu, A., Yang, Y., 
Salome, M. et al. (2019). J. Synchrotron Rad. 26, <https://doi.org/10.1107/S1600577519006301>
