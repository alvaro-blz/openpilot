VIL Setup
=======================
1. Run `openpilot/tools/ubuntu_setup.sh`
2. Install cuda and cudnn according to the Tensorflow version (https://www.tensorflow.org/install/source#tested_build_configurations)
3. If Ubuntu can't access the Pandas set Panda udev rules (`MODE:=0666` if `MODE=0666` doesn't work) 
4. Before going any further make sure Openpilot and Carla can run virtually in Ubuntu (follow the instructions inside `openpilot/tools/sim` and always update Openpilot and its subrepos to the latest version)

Note that you may have to setup [udev rules](https://community.comma.ai/wiki/index.php/Panda#Linux_udev_rules) for Linux, such as
``` bash
sudo tee /etc/udev/rules.d/11-panda.rules <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddcc", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddee", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

5. Run Carla (`./Carla.sh` on terminal or `openpilot/tools/sim/start_carla.sh`)
6. The code to avoid steering commands is located inside selfdrive/car/(maker)/(maker)can.py. The idea is to replace the value in the steering signal with 0 so that controls.py still sends steering commands to Carla but not to the vehicle. Black Panda is preferred for all CAN communications.

****Make sure the Panda is in ALL_OUTPUT mode (Blue and Red LED blinking)**** 

The output mode of the Panda is set on line 113 of `openpilot/selfdrive/board/boardd.cc`:
```
#Replace this one with ALL_OUTPUT
panda->set_safety_model(safety_model, safety_param); #Comment out
panda->set_safety_model(cereal::CarParams::SafetyModel::ALL_OUTPUT); #Add
```

7. If Openpilot runs but does not engage comment out the particular error. Errors can be found inside `events.py`and by searching the whole Openpilot repo find what file is triggering the error (`controlsd.py` is usually the first suspect) ### Add events location and how to search
8. The bridge between Carla and Openpilot is in openpilot/tools/sim/bridge_dino_carla.py. To run:

#### Add export PYTHONPATH=$HOME/openpilot to your bashrc

#### openpilot (in terminal 2) 
NOBOARD defines if the Panda will be used. 
If no Panda -> NOBOARD=1; if there is a Panda and we want to send CAN messages -> NOBOARD=0
If we want to start Openpilot without a Panda line 105 of `openpilot/tools/sim/bridge_dino_carla.py`has to be uncommented: `threading.Thread(target=health_function).start()`
```
cd ~/openpilot/selfdrive/
PASSIVE=0 NOBOARD=1 ./manager.py
```
#### bridge (in terminal 3)
```
### links carla to openpilot, will "start the car" according to manager
cd ~/openpilot/tools/sim
./bridge_dino_carla.py
```
#### Controls
The vehicle ACC determines when Openpilot engages. Speed is also controlled through the vehicle.

#### Other info:
- bridge_dino_carla.py can be run in multiple ways (following a vehicle, with human control and overriding the speed of the vehicle in front): 
More info can be found in the file comments.
- controlsd.py sends gas, brake and steering commands using the actuators object. The controls are set in lines 380 - 383:
```
# Gas/Brake PID loop
actuators.gas, actuators.brake = self.LoC.update(self.active, CS, v_acc_sol, plan.vTargetFuture, a_acc_sol, self.CP)
# Steering PID loop and lateral MPC
actuators.steer, actuators.steerAngle, lac_log = self.LaC.update(self.active, CS, self.CP, path_plan)
 ```
 Changing those lines and defining new values of `actuators.gas`, `actuators.brake`, `actuators.steer` and `actuators.steerAngle` will change the values that Openpilot sends to the vehicle and Carla. Make sure the scales are the same Openpilot uses:
 
```
gas #[0,1]
brake #[0,1]
steerAngle # degrees [-180,180]
steer is just a 0 or 1 value depending on if it needs to steer or not
```

`openpilot/tools/sim/driver_model_server.py` takes the input from a USB webcam and trasmit that to Openpilot. It may be useful if the computer can't
 handle Openpilot and Carla at the same time.


#### Possible way of adding new vehicles
- If the vehicle has long and lat control supported by Openpilot then it should be straightforward. Openpilot will control the vehicle when the ACC is engaged.
- If the vehicle has lateral Openpilot and not longitudinal an option is to create a function to send acceleration/gas commands and hope the vehicle responds to that. Somethin similar to this but replacing the values with the ones in the DBC file of the vehicle for the accel/gas message:
```
def create_accel_command(packer, accel, pcm_cancel, standstill_req, lead):
  # TODO: find the exact canceling bit that does not create a chime
  values = {
    "ACCEL_CMD": accel
    "SET_ME_X01": 1,
    "DISTANCE": 0,
    "MINI_CAR": lead,
    "SET_ME_X3": 3,
    "PERMIT_BRAKING": 1,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)

```
All vehicle functions to send CAN messages are located in `openpilot/selfdrive/car/(maker)/(maker)can.py`
These functions are called inside `openpilot/selfdrice/car/(maker)/carcontroller.py`
Therefore, a call to the created function would have to be made inside the `update`function of the `CarController` object. In particular after `can_send = []`is created and append it to `can_send`so that its actuallt sent through CAN.


#### Carla
- Carla tutorial and documentation: https://carla.readthedocs.io/en/latest/
- Edit Carla Server Address
Carla needs to connect to the Server in order to run. That connection by default is set locally within the same machine. The address and port for the server machine can be edited in case Carla Server is running on another machine in the same network. The line to edit the Server address is the following:
 `client = carla.Client("127.0.0.1", 2000)`
This line will be inside every file that makes a client run. The first argument is the IP address and the second one the port.

- Sumo and Carla tutorial: https://carla.readthedocs.io/en/latest/adv_sumo/

#### List of modified files and lines modified
- Created `openpilot/tools/sim/brideg_dino_carla.py`: Links Openpilot and Carla by sending OP commands to Carla and OP the Carla images.
- `phonelibs/json11/json11.cpp`: Moddified line 234 to initialize the variable and to solve compilation issues (it might not be required depending on the gcc and g++ compilator versions).
   - `const map<string, Json> empty_map {};`
- `selfdrive/boardd/boardd.cc`: Commented out line 112. Added line 113 to change panda SAFETY MODE to ALL_OUTPUT.
   - `panda->set_safety_model(cereal::CarParams::SafetyModel::ALL_OUTPUT);`
   
- `selfdrive/car/toyota/interface.py`: Commented out lines 321 and 324 to avoid Openpilot errors.
- `selfdrive/car/toyota/toyotacan.py `: Commented out lines 5, 6, 25, 29 and 30 to avoid steering messages being sent to the vehicle. Replaced all `STEER_REQUEST`, `STEER_TORQUE_CMD`and `STEER_REQUEST_2` values with 0.
- `selfdrive/controls/controlsd.py`: 
   - Added subscription to steering state on line 61 (`carlaState`) to replace the   vehicle's steering angle with the one in Carla.
   - Replaced steering values read from CAN (`CS.steeringAngle`) with the one sent from Carla (`self.sm['carlaState'].angle`) on lines 387, 476 and 499.
   - Commented out lines 204, 213 and 219 to avoid Openpilot errors
   
#### Schematics of vehicle connections
Openpilot is connected to the vehicle using a Panda Black, a development harness and a Toyota giraffe.
The development harness and the Giraffe are connected through a breakout box and each of their CAN buses are connected.
The pinout for the development harness can be found here: https://miro.medium.com/max/1400/1*3sAhgBvaPOexzVASj3AzXQ.png
The pinout for the Giraffe can be found here: https://github.com/commaai/neo/blob/master/giraffe/toyota/giraffe12.sch.pdf

The Panda Black requires power from the OBDII port and no power from the usb Paw (the green board that goes from USB mini to regular USB).

Table of Contents
=======================

* [What is openpilot?](#what-is-openpilot)
* [Integration with Stock Features](#integration-with-stock-features)
* [Supported Hardware](#supported-hardware)
* [Supported Cars](#supported-cars)
* [Community Maintained Cars and Features](#community-maintained-cars-and-features)
* [Installation Instructions](#installation-instructions)
* [Limitations of openpilot ALC and LDW](#limitations-of-openpilot-alc-and-ldw)
* [Limitations of openpilot ACC and FCW](#limitations-of-openpilot-acc-and-fcw)
* [Limitations of openpilot DM](#limitations-of-openpilot-dm)
* [User Data and comma Account](#user-data-and-comma-account)
* [Safety and Testing](#safety-and-testing)
* [Testing on PC](#testing-on-pc)
* [Community and Contributing](#community-and-contributing)
* [Directory Structure](#directory-structure)
* [Licensing](#licensing)

---

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW) and Lane Departure Warning (LDW) for a growing variety of supported [car makes, models and model years](#supported-cars). In addition, while openpilot is engaged, a camera based Driver Monitoring (DM) feature alerts distracted and asleep drivers.

<table>
  <tr>
    <td><a href="https://www.youtube.com/watch?v=mgAbfr42oI8" title="YouTube" rel="noopener"><img src="https://i.imgur.com/kAtT6Ei.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=394rJKeh76k" title="YouTube" rel="noopener"><img src="https://i.imgur.com/lTt8cS2.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=1iNOc3cq8cs" title="YouTube" rel="noopener"><img src="https://i.imgur.com/ANnuSpe.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=Vr6NgrB-zHw" title="YouTube" rel="noopener"><img src="https://i.imgur.com/Qypanuq.png"></a></td>
  </tr>
  <tr>
    <td><a href="https://www.youtube.com/watch?v=Ug41KIKF0oo" title="YouTube" rel="noopener"><img src="https://i.imgur.com/3caZ7xM.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=NVR_CdG1FRg" title="YouTube" rel="noopener"><img src="https://i.imgur.com/bAZOwql.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=tkEvIdzdfUE" title="YouTube" rel="noopener"><img src="https://i.imgur.com/EFINEzG.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=_P-N1ewNne4" title="YouTube" rel="noopener"><img src="https://i.imgur.com/gAyAq22.png"></a></td>
  </tr>
</table>

Integration with Stock Features
------

In all supported cars:
* Stock Lane Keep Assist (LKA) and stock ALC are replaced by openpilot ALC, which only functions when openpilot is engaged by the user.
* Stock LDW is replaced by openpilot LDW.

Additionally, on specific supported cars (see ACC column in [supported cars](#supported-cars)):
* Stock ACC is replaced by openpilot ACC.
* openpilot FCW operates in addition to stock FCW.

openpilot should preserve all other vehicle's stock features, including, but are not limited to: FCW, Automatic Emergency Braking (AEB), auto high-beam, blind spot warning, and side collision warning.

Supported Hardware
------

At the moment, openpilot supports the [EON DevKit](https://comma.ai/shop/products/eon-dashcam-devkit) and the [comma two](https://comma.ai/shop/products/comma-two-devkit). A [car harness](https://comma.ai/shop/products/car-harness) is recommended to connect the EON or comma two to the car. For experimental purposes, openpilot can also run on an Ubuntu computer with external [webcams](https://github.com/commaai/openpilot/tree/master/tools/webcam).

Supported Cars
------

| Make      | Model (US Market Reference)   | Supported Package | ACC              | No ACC accel below | No ALC below      |
| ----------| ------------------------------| ------------------| -----------------| -------------------| ------------------|
| Acura     | ILX 2016-18                   | AcuraWatch Plus   | openpilot        | 25mph<sup>1</sup>  | 25mph             |
| Acura     | RDX 2016-18                   | AcuraWatch Plus   | openpilot        | 25mph<sup>1</sup>  | 12mph             |
| Honda     | Accord 2018-20                | All               | Stock            | 0mph               | 3mph              |
| Honda     | Accord Hybrid 2018-20         | All               | Stock            | 0mph               | 3mph              |
| Honda     | Civic Hatchback 2017-19       | Honda Sensing     | Stock            | 0mph               | 12mph             |
| Honda     | Civic Sedan/Coupe 2016-18     | Honda Sensing     | openpilot        | 0mph               | 12mph             |
| Honda     | Civic Sedan/Coupe 2019-20     | Honda Sensing     | Stock            | 0mph               | 2mph<sup>2</sup>  |
| Honda     | CR-V 2015-16                  | Touring           | openpilot        | 25mph<sup>1</sup>  | 12mph             |
| Honda     | CR-V 2017-20                  | Honda Sensing     | Stock            | 0mph               | 12mph             |
| Honda     | CR-V Hybrid 2017-2019         | Honda Sensing     | Stock            | 0mph               | 12mph             |
| Honda     | Fit 2018-19                   | Honda Sensing     | openpilot        | 25mph<sup>1</sup>  | 12mph             |
| Honda     | HR-V 2019                     | Honda Sensing     | openpilot        | 25mph<sup>1</sup>  | 12mph             |
| Honda     | Insight 2019-20               | Honda Sensing     | Stock            | 0mph               | 3mph              |
| Honda     | Odyssey 2018-20               | Honda Sensing     | openpilot        | 25mph<sup>1</sup>  | 0mph              |
| Honda     | Passport 2019                 | All               | openpilot        | 25mph<sup>1</sup>  | 12mph             |
| Honda     | Pilot 2016-18                 | Honda Sensing     | openpilot        | 25mph<sup>1</sup>  | 12mph             |
| Honda     | Pilot 2019                    | All               | openpilot        | 25mph<sup>1</sup>  | 12mph             |
| Honda     | Ridgeline 2017-20             | Honda Sensing     | openpilot        | 25mph<sup>1</sup>  | 12mph             |
| Hyundai   | Sonata 2020                   | All               | Stock            | 0mph               | 0mph              |
| Lexus     | CT Hybrid 2017-18             | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Lexus     | ES 2019                       | All               | openpilot        | 0mph               | 0mph              |
| Lexus     | ES Hybrid 2019                | All               | openpilot        | 0mph               | 0mph              |
| Lexus     | IS 2017-2019                  | All               | Stock            | 22mph              | 0mph              |
| Lexus     | IS Hybrid 2017                | All               | Stock            | 0mph               | 0mph              |
| Lexus     | NX Hybrid 2018                | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Lexus     | RX 2016-17                    | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Lexus     | RX 2020                       | All               | openpilot        | 0mph               | 0mph              |
| Lexus     | RX Hybrid 2016-19             | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Lexus     | RX Hybrid 2020                | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Avalon 2016                   | TSS-P             | Stock<sup>3</sup>| 20mph<sup>1</sup>  | 0mph              |
| Toyota    | Avalon 2017-18                | All               | Stock<sup>3</sup>| 20mph<sup>1</sup>  | 0mph              |
| Toyota    | Camry 2018-20                 | All               | Stock            | 0mph<sup>4</sup>   | 0mph              |
| Toyota    | Camry Hybrid 2018-19          | All               | Stock            | 0mph<sup>4</sup>   | 0mph              |
| Toyota    | C-HR 2017-19                  | All               | Stock            | 0mph               | 0mph              |
| Toyota    | C-HR Hybrid 2017-19           | All               | Stock            | 0mph               | 0mph              |
| Toyota    | Corolla 2017-19               | All               | Stock<sup>3</sup>| 20mph<sup>1</sup>  | 0mph              |
| Toyota    | Corolla 2020                  | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Corolla Hatchback 2019-20     | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Corolla Hybrid 2020           | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Highlander 2017-19            | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Toyota    | Highlander Hybrid 2017-19     | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Toyota    | Highlander 2020               | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Highlander Hybrid 2020        | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Prius 2016                    | TSS-P             | Stock<sup>3</sup>| 0mph               | 0mph              |
| Toyota    | Prius 2017-20                 | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Toyota    | Prius Prime 2017-20           | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Toyota    | Rav4 2016                     | TSS-P             | Stock<sup>3</sup>| 20mph<sup>1</sup>  | 0mph              |
| Toyota    | Rav4 2017-18                  | All               | Stock<sup>3</sup>| 20mph<sup>1</sup>  | 0mph              |
| Toyota    | Rav4 2019-20                  | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Rav4 Hybrid 2016              | TSS-P             | Stock<sup>3</sup>| 0mph               | 0mph              |
| Toyota    | Rav4 Hybrid 2017-18           | All               | Stock<sup>3</sup>| 0mph               | 0mph              |
| Toyota    | Rav4 Hybrid 2019-20           | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Sienna 2018-20                | All               | Stock<sup>3</sup>| 0mph               | 0mph              |

<sup>1</sup>[Comma Pedal](https://github.com/commaai/openpilot/wiki/comma-pedal) is used to provide stop-and-go capability to some of the openpilot-supported cars that don't currently support stop-and-go. ***NOTE: The Comma Pedal is not officially supported by [comma](https://comma.ai).*** <br />
<sup>2</sup>2019 Honda Civic 1.6L Diesel Sedan does not have ALC below 12mph. <br />
<sup>3</sup>When disconnecting the Driver Support Unit (DSU), openpilot ACC will replace stock ACC. ***NOTE: disconnecting the DSU disables Automatic Emergency Braking (AEB).*** <br />
<sup>4</sup>28mph for Camry 4CYL L, 4CYL LE and 4CYL SE which don't have Full-Speed Range Dynamic Radar Cruise Control. <br />

Community Maintained Cars and Features
------

| Make      | Model (US Market Reference)   | Supported Package | ACC              | No ACC accel below | No ALC below |
| ----------| ------------------------------| ------------------| -----------------| -------------------| -------------|
| Buick     | Regal 2018<sup>1</sup>        | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Cadillac  | ATS 2018<sup>1</sup>          | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Chevrolet | Malibu 2017<sup>1</sup>       | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Chevrolet | Volt 2017-18<sup>1</sup>      | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Chrysler  | Pacifica 2017-18              | Adaptive Cruise   | Stock            | 0mph               | 9mph         |
| Chrysler  | Pacifica 2020                 | Adaptive Cruise   | Stock            | 0mph               | 39mph        |
| Chrysler  | Pacifica Hybrid 2017-18       | Adaptive Cruise   | Stock            | 0mph               | 9mph         |
| Chrysler  | Pacifica Hybrid 2019-20       | Adaptive Cruise   | Stock            | 0mph               | 39mph        |
| Genesis   | G70 2018                      | All               | Stock            | 0mph               | 0mph         |
| Genesis   | G80 2018                      | All               | Stock            | 0mph               | 0mph         |
| Genesis   | G90 2018                      | All               | Stock            | 0mph               | 0mph         |
| GMC       | Acadia Denali 2018<sup>2</sup>| Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Holden    | Astra 2017<sup>1</sup>        | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Hyundai   | Elantra 2017-19               | SCC + LKAS        | Stock            | 19mph              | 34mph        |
| Hyundai   | Genesis 2015-16               | SCC + LKAS        | Stock            | 19mph              | 37mph        |
| Hyundai   | Ioniq Electric Premium SE 2020| SCC + LKAS        | Stock            | 0mph               | 32mph        |
| Hyundai   | Ioniq Electric Limited 2019   | SCC + LKAS        | Stock            | 0mph               | 32mph        |
| Hyundai   | Kona 2020                     | SCC + LKAS        | Stock            | 0mph               | 0mph         |
| Hyundai   | Kona EV 2019                  | SCC + LKAS        | Stock            | 0mph               | 0mph         |
| Hyundai   | Palisade 2020                 | All               | Stock            | 0mph               | 0mph         |
| Hyundai   | Santa Fe 2019                 | All               | Stock            | 0mph               | 0mph         |
| Hyundai   | Sonata 2019                   | All               | Stock            | 0mph               | 0mph         |
| Hyundai   | Veloster 2019                 | SCC + LKAS        | Stock            | 5mph               | 0mph         |
| Jeep      | Grand Cherokee 2016-18        | Adaptive Cruise   | Stock            | 0mph               | 9mph         |
| Jeep      | Grand Cherokee 2019-20        | Adaptive Cruise   | Stock            | 0mph               | 39mph        |
| Kia       | Forte 2018-19                 | SCC + LKAS        | Stock            | 0mph               | 0mph         |
| Kia       | Optima 2017                   | SCC + LKAS/LDWS   | Stock            | 0mph               | 32mph        |
| Kia       | Optima 2019                   | SCC + LKAS        | Stock            | 0mph               | 0mph         |
| Kia       | Sorento 2018                  | SCC + LKAS        | Stock            | 0mph               | 0mph         |
| Kia       | Stinger 2018                  | SCC + LKAS        | Stock            | 0mph               | 0mph         |
| Nissan    | Leaf 2018-19                  | Propilot          | Stock            | 0mph               | 0mph         |
| Nissan    | Rogue 2019                    | Propilot          | Stock            | 0mph               | 0mph         |
| Nissan    | X-Trail 2017                  | Propilot          | Stock            | 0mph               | 0mph         |
| Subaru    | Ascent 2019                   | EyeSight          | Stock            | 0mph               | 0mph         |
| Subaru    | Crosstrek 2018-19             | EyeSight          | Stock            | 0mph               | 0mph         |
| Subaru    | Forester 2019                 | EyeSight          | Stock            | 0mph               | 0mph         |
| Subaru    | Impreza 2017-19               | EyeSight          | Stock            | 0mph               | 0mph         |
| Volkswagen| Golf 2015-19                  | Driver Assistance | Stock            | 0mph               | 0mph         |

<sup>1</sup>Requires an [OBD-II car harness](https://comma.ai/shop/products/comma-car-harness) and [community built giraffe](https://github.com/commaai/openpilot/wiki/GM). ***NOTE: disconnecting the ASCM disables Automatic Emergency Braking (AEB).*** <br />
<sup>2</sup>Requires a custom connector for the developer [car harness](https://comma.ai/shop/products/car-harness) <br />

Although it's not upstream, there's a community of people getting openpilot to run on Tesla's [here](https://tinkla.us/)

Community Maintained Cars and Features are not verified by comma to meet our [safety model](SAFETY.md). Be extra cautious using them. They are only available after enabling the toggle in `Settings->Developer->Enable Community Features`.

To promote a car from community maintained, it must meet a few requirements. We must own one from the brand, we must sell the harness for it, has full ISO26262 in both panda and openpilot, there must be a path forward for longitudinal control, it must have AEB still enabled, and it must support fingerprinting 2.0

Installation Instructions
------

Install openpilot on an EON or comma two by entering ``https://openpilot.comma.ai`` during the installer setup.

Follow these [video instructions](https://youtu.be/lcjqxCymins) to properly mount the device on the windshield. Note: openpilot features an automatic pose calibration routine and openpilot performance should not be affected by small pitch and yaw misalignments caused by imprecise device mounting.

Before placing the device on your windshield, check the state and local laws and ordinances where you drive. Some state laws prohibit or restrict the placement of objects on the windshield of a motor vehicle.

You will be able to engage openpilot after reviewing the onboarding screens and finishing the calibration procedure.

Limitations of openpilot ALC and LDW
------

openpilot ALC and openpilot LDW do not automatically drive the vehicle or reduce the amount of attention that must be paid to operate your vehicle. The driver must always keep control of the steering wheel and be ready to correct the openpilot ALC action at all times.

While changing lanes, openpilot is not capable of looking next to you or checking your blind spot. Only nudge the wheel to initiate a lane change after you have confirmed it's safe to do so.

Many factors can impact the performance of openpilot ALC and openpilot LDW, causing them to be unable to function as intended. These include, but are not limited to:

* Poor visibility (heavy rain, snow, fog, etc.) or weather conditions that may interfere with sensor operation.
* The road facing camera is obstructed, covered or damaged by mud, ice, snow, etc.
* Obstruction caused by applying excessive paint or adhesive products (such as wraps, stickers, rubber coating, etc.) onto the vehicle.
* The device is mounted incorrectly.
* When in sharp curves, like on-off ramps, intersections etc...; openpilot is designed to be limited in the amount of steering torque it can produce.
* In the presence of restricted lanes or construction zones.
* When driving on highly banked roads or in presence of strong cross-wind.
* Extremely hot or cold temperatures.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* Driving on hills, narrow, or winding roads.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of openpilot components. It is the driver's responsibility to be in control of the vehicle at all times.

Limitations of openpilot ACC and FCW
------

openpilot ACC and openpilot FCW are not systems that allow careless or inattentive driving. It is still necessary for the driver to pay close attention to the vehicle’s surroundings and to be ready to re-take control of the gas and the brake at all times.

Many factors can impact the performance of openpilot ACC and openpilot FCW, causing them to be unable to function as intended. These include, but are not limited to:

* Poor visibility (heavy rain, snow, fog, etc.) or weather conditions that may interfere with sensor operation.
* The road facing camera or radar are obstructed, covered, or damaged by mud, ice, snow, etc.
* Obstruction caused by applying excessive paint or adhesive products (such as wraps, stickers, rubber coating, etc.) onto the vehicle.
* The device is mounted incorrectly.
* Approaching a toll booth, a bridge or a large metal plate.
* When driving on roads with pedestrians, cyclists, etc...
* In presence of traffic signs or stop lights, which are not detected by openpilot at this time.
* When the posted speed limit is below the user selected set speed. openpilot does not detect speed limits at this time.
* In presence of vehicles in the same lane that are not moving.
* When abrupt braking maneuvers are required. openpilot is designed to be limited in the amount of deceleration and acceleration that it can produce.
* When surrounding vehicles perform close cut-ins from neighbor lanes.
* Driving on hills, narrow, or winding roads.
* Extremely hot or cold temperatures.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* Interference from other equipment that generates radar waves.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of openpilot components. It is the driver's responsibility to be in control of the vehicle at all times.

Limitations of openpilot DM
------

openpilot DM should not be considered an exact measurement of the alertness of the driver.

Many factors can impact the performance of openpilot DM, causing it to be unable to function as intended. These include, but are not limited to:

* Low light conditions, such as driving at night or in dark tunnels.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* The driver's face is partially or completely outside field of view of the driver facing camera.
* Right hand driving vehicles.
* The driver facing camera is obstructed, covered, or damaged.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of openpilot components. A driver should not rely on openpilot DM to assess their level of attention.

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data by pairing with the comma connect app ([iOS](https://apps.apple.com/us/app/comma-connect/id1456551889), [Android](https://play.google.com/store/apps/details?id=ai.comma.connect&hl=en_US)). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://my.comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

Safety and Testing
----

* openpilot observes ISO26262 guidelines, see [SAFETY.md](SAFETY.md) for more detail.
* openpilot has software in the loop [tests](.github/workflows/test.yaml) that run on every commit.
* The safety model code lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software in the loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware in the loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware in the loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 EONs continuously replaying routes.

Testing on PC
------

Check out the tools directory in master: lots of tools you can use to replay driving data, test and develop openpilot from your pc.

Community and Contributing
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot). Bug fixes and new car ports are encouraged.

You can add support for your car by following guides we have written for [Brand](https://medium.com/@comma_ai/how-to-write-a-car-port-for-openpilot-7ce0785eda84) and [Model](https://medium.com/@comma_ai/openpilot-port-guide-for-toyota-models-e5467f4b5fe6) ports. Generally, a car with adaptive cruise control and lane keep assist is a good candidate. [Join our Discord](https://discord.comma.ai) to discuss car ports: most car makes have a dedicated channel.

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs/).

And [follow us on Twitter](https://twitter.com/comma_ai).

Directory Structure
------
    .
    ├── apk                 # The apk files used for the UI
    ├── cereal              # The messaging spec and libs used for all logs
    ├── common              # Library like functionality we've developed here
    ├── installer/updater   # Manages auto-updates of openpilot
    ├── opendbc             # Files showing how to interpret data from cars
    ├── panda               # Code used to communicate on CAN
    ├── phonelibs           # Libraries used on NEOS devices
    ├── pyextra             # Libraries used on NEOS devices
    └── selfdrive           # Code needed to drive the car
        ├── assets          # Fonts, images, and sounds for UI
        ├── athena          # Allows communication with the app
        ├── boardd          # Daemon to talk to the board
        ├── camerad         # Driver to capture images from the camera sensors
        ├── car             # Car specific code to read states and control actuators
        ├── common          # Shared C/C++ code for the daemons
        ├── controls        # Perception, planning and controls
        ├── debug           # Tools to help you debug and do car ports
        ├── locationd       # Soon to be home of precise location
        ├── logcatd         # Android logcat as a service
        ├── loggerd         # Logger and uploader of car data
        ├── modeld          # Driving and monitoring model runners
        ├── proclogd        # Logs information from proc
        ├── sensord         # IMU / GPS interface code
        ├── test            # Unit tests, system tests and a car simulator
        └── ui              # The UI

To understand how the services interact, see `cereal/service_list.yaml`.

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>

[![openpilot tests](https://github.com/commaai/openpilot/workflows/openpilot%20tests/badge.svg?event=push)](https://github.com/commaai/openpilot/actions)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:python)
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:cpp)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
