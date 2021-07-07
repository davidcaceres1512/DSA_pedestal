<p align="center">
  <a href="" rel="noopener">
 <img width=357px height=599px src="img/pedestal.jpg" alt="Project logo"></a>
</p>

<h3 align="center">Pedestal Controller</h3>

<div align="center">

[![Status](https://img.shields.io/badge/status-active-success.svg)]()
[![GitHub Issues](https://img.shields.io/github/issues/kylelobo/The-Documentation-Compendium.svg)](https://github.com/davidcaceres1512/DSA_pedestal/issues)
[![GitHub Pull Requests](https://img.shields.io/github/issues-pr/kylelobo/The-Documentation-Compendium.svg)](https://github.com/davidcaceres1512/DSA_pedestal/pulls)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

</div>

---

<p align="center"> Pedestal controller for a weather radar.
    <br> 
</p>

## 📝 Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Deployment](#deployment)
- [Usage](#usage)
- [Built Using](#built_using)
- [TODO](../TODO.md)
- [Contributing](../CONTRIBUTING.md)
- [Authors](#authors)
- [Acknowledgments](#acknowledgement)

## 🧐 About <a name = "about"></a>
---

Write about 1-2 paragraphs describing the purpose of your project.

## 🏁 Getting Started <a name = "getting_started"></a>
---

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See [deployment](#deployment) for notes on how to deploy the project on a live system.

### Prerequisites
---

- [Firmware](#req_firmware)
- [Software](#req_software)
- [Hardware](#deployment)
#### Firmware <a name = "req_firmware"></a>

#### - libraries
libraries used in the framework arduino-cli with stm32duino_core 1.9.0.
1. decompress libraries.zip in the folder libraries

> open readme [here](libraries/firmware/readme.md)
#### Software <a name = "req_software"></a>

#### - broker

1. Install broker  [mosquitto](https://mosquitto.org/download/) 1.6.x.

2. deactivate firewall or add new rule into firewall (port 1883).

3. activate the service mosquitto.

> **warning!**
> if you install mosquitto 2.x.x you have modified mosquitto.conf
> 1. add the next words into file **mosquitto.conf**
> ```bash
> listener 1883
> allow_anonymous true 
> ```
> 2. restart the services for surting effect the modified  
> ```cmd
> windows+r => services.msc
> restart mosquitto
> ```
> *click [here](http://www.steves-internet-guide.com/install-mosquitto-broker/) for more information.* 

#### - python requirements

1. install python 3.9
2. install the requirements

```python
pip install -r libraries/software/requirements.txt
```
### Installing

A step by step series of examples that tell you how to get a development env running.

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo.

## 🔧 Running the tests <a name = "tests"></a>

Explain how to run the automated tests for this system.

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## 🎈 Usage <a name="usage"></a>

Add notes about how to use the system.

## 🚀 Deployment <a name = "deployment"></a>

Add additional notes about how to deploy this on a live system.

## ⛏️ Built Using <a name = "built_using"></a>

- [MongoDB](https://www.mongodb.com/) - Database
- [Express](https://expressjs.com/) - Server Framework
- [VueJs](https://vuejs.org/) - Web Framework
- [NodeJs](https://nodejs.org/en/) - Server Environment

## ✍️ Authors <a name = "authors"></a>

- [@kylelobo](https://github.com/kylelobo) - Idea & Initial work

See also the list of [contributors](https://github.com/kylelobo/The-Documentation-Compendium/contributors) who participated in this project.

## 🎉 Acknowledgements <a name = "acknowledgement"></a>

- Hat tip to anyone whose code was used
- Inspiration
- References
