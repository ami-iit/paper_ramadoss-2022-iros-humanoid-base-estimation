## Installation

### Code Dependencies

The software has been tested with MATLAB R2021B on an Ubuntu 20.04 LTS OS. The required dependencies are,

- [iDynTree MATLAB Bindings](https://github.com/robotology/idyntree#bindings)
- MATLAB Curve Fitting Toolbox

### Easier way to install dependencies
Instead of manually installing the iDynTree MATLAB bindings, we provide a script that installs the bindings using a `Mambaforge` installer without modifying anything in the PC.

One can install these bindings by launching the script `install_idyntree_bindings` on the MATLAB console while being in the `scripts` folder. This process installs the bindings in `paper_ramadoss-2022-ral-humanoid-base-estimation/deps/idyntree-matlab/mex` directory.

Additionally, a configuration script `idyntree_bindings_setup.m` is created in the `scripts` folder. Running this script, adds the iDynTree bindings in the MATLAB path.

Once the bindings are installed, it is possible to run the filters, as described in the section below.

If one wants to uninstall the iDynTree bindings, they may simply delete the folder where it was installed.



## Usage

This repository includes sample MATLAB implementation of so-called **flat foot filters** for the floating bae estimation of  humanoid robots with flat feet. These filters are based on an Extended Kalman Filter (EKF) based framework.

| Method                                                       | Implementation                                               | Please cite                                                  | Remark                                                       |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Observability Constrained quaternion based EKF (**OCEKF**)   | [`Estimation.OCEKF.Filter`](./../src/+Estimation/+OCEKF/Filter.m) | N. Rotella, M. Bloesch, L. Righetti and S. Schaal, "State estimation for a humanoid robot," 2014 IEEE/RSJ International Conference on Intelligent Robots and Systems, 2014, pp. 952-958, doi: 10.1109/IROS.2014.6942674. | This was previously referred to as [`RotellaEstimator`](https://github.com/ami-iit/paper_ramadoss_2021_icra_proprioceptive-base-estimator/blob/main/matlab/+Estimation/+RotellaEstimator/Filter.m) in [our work](https://github.com/ami-iit/paper_ramadoss_2021_icra_proprioceptive-base-estimator) submitted to ICRA 2021. |
| Flat Foot Invariant EKF (**InvEKF_F**)                       | [`Estimation.InvEKF_F.Filter`](./../src/+Estimation/+InvEKF_F/Filter.m) | M. Qin et al., "A Novel Foot Contact Probability Estimator for Biped Robot State Estimation," 2020 IEEE International Conference on Mechatronics and Automation (ICMA), 2020, pp. 1901-1906, doi: 10.1109/ICMA49215.2020.9233715. | This is the flat-foot extension of Right Invariant EKF proposed in `Contact-aided invariant extended Kalman filtering for robot state estimation` which was previously implemented in  [`InvEKF`](https://github.com/ami-iit/paper_ramadoss_2021_icra_proprioceptive-base-estimator/blob/main/matlab/%2BEstimation/%2BInvEKF/Filter.m) for [our work](https://github.com/ami-iit/paper_ramadoss_2021_icra_proprioceptive-base-estimator) submitted to ICRA 2021. |
| Discrete Lie Group Extended Kalman Filter for Kinematic Inertial Odometry (**DILIGENT-KIO**) | [`Estimation.DILIGENT_KIO.Filter`](./../src/+Estimation/+DILIGENT_KIO/Filter.m) | P. Ramadoss, G. Romualdi, S. Dafarra, F. J. Andrade Chavez, S. Traversaro and D. Pucci, "DILIGENT-KIO: A Proprioceptive Base Estimator for Humanoid Robots using Extended Kalman Filtering on Matrix Lie Groups," 2021 IEEE International Conference on Robotics and Automation (ICRA), 2021, pp. 2904-2910, doi: 10.1109/ICRA48506.2021.9561248. | This is an [implementation](https://github.com/ami-iit/paper_ramadoss_2021_icra_proprioceptive-base-estimator/blob/main/matlab/+Estimation/+DLGEKF/Filter.m) from  [our work](https://github.com/ami-iit/paper_ramadoss_2021_icra_proprioceptive-base-estimator)  submitted to ICRA 2021. |
| DILIGENT-KIO with right invariant matrix Lie group error formulation (**DILIGENT-KIO-RIE**) | [`Estimation.DILIGENT_KIO_RIE.Filter`](./../src/+Estimation/+DILIGENT_KIO_RIE/Filter.m) | See [citing-this-work](https://github.com/ami-iit/paper_ramadoss-2022-ral-humanoid-base-estimation#citing-this-work). |                                                              |
| Continuous-Discrete Lie Group Extended Kalman Filter for Kinematic Inertial Odometry (**CODILIGENT-KIO**) |  [`Estimation.CODILIGENT_KIO.Filter`](./../src/+Estimation/+CODILIGENT_KIO/Filter.m) | See [citing-this-work](https://github.com/ami-iit/paper_ramadoss-2022-ral-humanoid-base-estimation#citing-this-work). |                                                              |
| CODILIGENT-KIO with right invariant error formulation (**CODILIGENT-KIO-RIE**) |  [`Estimation.CODILIGENT_KIO_RIE.Filter`](./../src/+Estimation/+CODILIGENT_KIO_RIE/Filter.m) | See [citing-this-work](https://github.com/ami-iit/paper_ramadoss-2022-ral-humanoid-base-estimation#citing-this-work). |                                                              |




### How to run the filter

- Calling `examples.iCubGenova04` in a MATLAB console from `src` folder runs all the filters over a dataset collected from a 1 meter walking or a center of mass trajectory tracking experiment conducted on `iCub  v2.5` humanoid robot and plots the base pose and linear velocity estimates, along with the metric error charts.

- Calling `examples.iCubGazeboV3` in a MATLAB console from `src` folder runs all the filters over a dataset collected from a 3 meter walking experiment conducted on Gazebo simulated  `iCub  v3.0` humanoid robot and plots the base pose and linear velocity estimates, along with the metric error charts.



A few high-level parameters include,

| Parameter         | Description                       | Type    | Options                     |
| ----------------- | --------------------------------- | ------- | --------------------------- |
| `experiment_name` | name of the experiment's mat file | string  | `walking` or `com-sinusoid` for `iCubGenova04`, `gazebo-v3-walking` for `iCubGazeboV3` |
| `enable_ocekf`    | enable OCEKF computations         | boolean | `true` or `false`           |
| `enable_invfekf_f` | enable InvEKF_F computations | boolean |       `true` or `false`           |
| `enable_diligent_kio` | enable DILIGENT-KIO computations | boolean |       `true` or `false`           |
| `enable_diligent_kio_rie` | enable DILIGENT-KIO-RIE computations | boolean |       `true` or `false`           |
| `enable_codiligent_kio` | enable CODILIGENT-KIO computations | boolean |       `true` or `false`           |
| `enable_codiligent_kio_rie` | enable CODILIGENT-KIO-RIE computations | boolean |       `true` or `false`           |

__NOTE:__ The MATLAB code is not optimized for performance. This means running the code will take some time to run the computations across all filters, the iterations will be displayed on the console.
__NOTE:__  The error metrics comparison on a spider chart will be displayed only if all the filters are enabled.

The filter configuration and noise parameters for `iCubGenova04` and `iCubGazeboV3` can be modified from within the [`+Config`](./../src/+Config) namespace.

| Configuration file                                           | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| `configMatlabPriors` | Set prior standard deviations for the EKF based estimator's internal state common to all filters. |
| `configMatlabSensorDev` | Set measurement noise and prediction model noise standard deviations common to all filters. |
| `configSchmittParams` | Set Schmitt Trigger parameters for contact detection based on contact normal forces |

