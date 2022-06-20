# LCSS-control
This repository contains the MATLAB source code used for reproducing the trajectory planning simulations in the LCSS papers titled:

*K. Ren, H. Ahn, and M. Kamgarpour, “Chance-Constrained Trajectory Planning with Multimodal Environmental Uncertainty,” IEEE Control Systems Letters, 2022.*

*H. Ahn, C. Chen, I. M. Mitchell, and M. Kamgarpour, “Safe Motion Planning Against Multimodal Distributions Based on a Scenario Approach,” IEEE Control Systems Letters, vol. 6, p. 1142–1147, 2022.*

## Dependencies
The following dependencies need to be installed/configured and must be on the MATLAB path:

[YALMIP](https://yalmip.github.io/) (configured with an optimization solver such as [CPLEX](https://www.ibm.com/analytics/cplex-optimizer))

[MagInset](https://www.mathworks.com/matlabcentral/fileexchange/49055-maginset)

## Instructions

After installing the dependencies, simply run **main.m**.

## References of the work
Please cite the original paper when using any part of this code. BibTeX citation data:
```
@article{Ahn_2022_LCSS,
   title={{Safe Motion Planning Against Multimodal Distributions Based on a Scenario Approach}},
   volume={6},
   ISSN={2475-1456},
   DOI={10.1109/lcsys.2021.3089641},
   journal={IEEE Control Systems Letters},
   publisher={Institute of Electrical and Electronics Engineers (IEEE)},
   author={Ahn, Heejin and Chen, Colin and Mitchell, Ian M. and Kamgarpour, Maryam},
   year={2022},
   pages={1142–1147}
}

@article{Ren_2022_LCSS,
  author={Ren, Kai and Ahn, Heejin and Kamgarpour, Maryam},
  journal={IEEE Control Systems Letters}, 
  title={{Chance-Constrained Trajectory Planning with Multimodal Environmental Uncertainty}}, 
  year={2022}}
```
