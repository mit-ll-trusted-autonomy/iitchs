# IITCHS Base

[![IITCHS stable documentation](https://img.shields.io/badge/docs-stable-blue)](https://mit-ll-trusted-autonomy.github.io/iitchs/)

A high-level planner for coordinating large, heterogeneous teams of autonomous agents.

## Overview

Inter- and Intra- Team Coordination From High Level Specifications (IITCHS) Code Base is a software package that allows a single human operator to deploy teams of self-coordinating, heterogeneous, autonomous robotic platforms using high-level specifications.
There is an increasing focus on developing autonomous teams of agents that can cooperate together. This work can roughly be divided into two paradigms: Swarms and Teams. Swarms typically involve large numbers of agents performing behaviors. The swarm as a whole tends to be robust to agent attrition, however swarms typically only contain homogeneous agents and perform relatively simple behaviors or tasks. Teams on the other hand are often composed of fewer heterogeneous agents. Teams typically carry out more complex behavior, but tend to not be robust to agent failure. 

IITCHS Code Base combines the best attributes of both Swarms and Teams. It enables one human operator to supervise and control large heterogeneous teams performing complex behaviors and dynamic tasks, while still being robust and resilient to agent failures and attrition. IITCHS Code Base takes in high-level objectives from a human operator and then automatically decomposes these objectives into precise, low-level plans to be executed by the autonomous robots on the heterogeneous team. In making these plans it takes into account the environment the agents are working in and the differing capabilities of individual team members. The human supervisor only needs to worry about the high-level objectives of the problem at hand without needing to figure out all details of the solution. 

IITCHS is designed to perform well on problems that

* Involve agents with differing sensors or capabilities
* Have complex time and location constraints
* Require plans that will be executed on the order of hours to days


IITCHS may not be the best match for problems that

* Require rapidly changing plans in highly uncertain environments
* Require solutions on the order of subseconds to seconds


# Citation Information

<!--Please use the following DOI reference number, published on Zenodo, when citing this software:-->

<!--\[INSERT HERE WHEN WE HAVE PUBLIC GITHUB REPO URL\]-->

If you use this code in your work, please consider citing the following papers:

[Scalable and Robust Algorithms for Task-Based Coordination From High-Level Specifications (ScRATCHeS)](https://ieeexplore.ieee.org/document/9663414)

[![Scalable and Robust Algorithms for Task-Based Coordination From High-Level Specifications (ScRATCHeS)](https://img.shields.io/badge/DOI-10.1109%2FTRO.2021.3130794-blue)](https://doi.org/10.1109/TRO.2021.3130794)

```
@article{Leahy2021ScalableAR,
  title={Scalable and Robust Algorithms for Task-Based Coordination From High-Level Specifications (ScRATCHeS)},
  author={Kevin J. Leahy and Zachary T. Serlin and Cristian Ioan Vasile and Andrew Schoer and Austin Jones and Roberto Tron and Calin A. Belta},
  journal={IEEE Transactions on Robotics},
  year={2021}
}
```

# Distribution and Disclaimer Statements

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

© 2021 MASSACHUSETTS INSTITUTE OF TECHNOLOGY

    Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014)
    SPDX-License-Identifier: BSD-3-Clause

This material is based upon work supported by the Under Secretary of Defense for 
Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any 
opinions, findings, conclusions or recommendations expressed in this material 
are those of the author(s) and do not necessarily reflect the views of the Under 
Secretary of Defense for Research and Engineering.

The software/firmware is provided to you on an As-Is basis
