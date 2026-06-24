# Alex GUI
This GUI was developed as an example of how to communicate with and run Alex using python. 
The GUI works in conjunction with [`ihmc-alex-sdk`](https://github.com/ihmcrobotics/ihmc-alex-sdk)

## Setting up the Workspace
These instructions are specifically for Linux systems and may not work for Windows.
### Prerequisites
- python >= 3.10 (could work on earlier, but untested)

### Instructions
1. Navigate to wherever you want to store your workspace and run `mkdir alex_ws && cd alex_ws`
2. Clone `ihmc-alex-sdk` into the same workspace as `alex-gui`. To clone the repositories, run these commands:
   1. `git clone https://github.com/ihmcrobotics/ihmc-alex-sdk.git`
   2. `git clone https://github.com/ihmcrobotics/alex-gui.git`
3. After cloning the repos,navigate into alex-gui by running `cd alex-gui`
4. Create a python virtual environment by running `python -m venv <env_name>`, replacing `<env_name>` with the desired environment name. 
5. Activate the environment by running `source <env_name>/bin/activate`
6. Upgrade pip by running `python -m pip install --upgrade pip`
   1. If you don’t complete this, some of the packages may have issues installing 
7. Finally, run `pip install .`. Once installed, your workspace is now ready!
