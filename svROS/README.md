## A QUICK REFERENCE

Every functionality operates over the concept of a *Project*, that corresponds to a certain ROS2 architecture. Each project is stored within the same directory *Projects*, within the tool's main directory *HOME/.svROS*.

#### Init

The proper installation of *HOME/.svROS*, and starting of its main directory, is achieved by running the following command. 
```
svROS init
```
This functionality helps to easily set up a working environment for this specific purpose. After this operation, the reader can now proceed for the architecture extraction of a ROS application, and later, translate the inferred architecture to perform formal analysis.

#### Exporting

Exporting of data through several nested parsers is made possible by this functionality, which deduces the topology of a ROS application by interpreting and converting ROS launch files and their associated nodes. To run this, a file must be passed as an input.
```
svROS export -f $file
```
Every file path inputted must respect the following syntax: A project name must be instantiated along with paths to launch files, that will be interpreted and respectively parsed.

```
project: PROJECT NAME
launch:
  - '/PATH/TO/LAUNCH FILE'
```

After executing the latest command, a project directory will be rightfully created within the *HOME/.svROS* projects directory. Two different templates are created inside the project's directory, which are then used to as the main data source for creating verification models in Alloy: one represents the network architecture through a *YML*-based file, whereas the other corresponds to a SROS2 policy file, in which privileges and communications are set upon nodes.

##### NOTEWORTHY MENTION
Most of the extracting procedures were implemented by using functionalities from [HAROS](https://github.com/git-afsantos/haros).

#### Launching

Launching enables the analysis using Alloy. This functionality is responsible for taking those templates and translate the inferred data into Alloy verification models. It relies on multiple file parsers and Alloy meta-models, which are refined by each concrete application model. This functionality frees the user from creating its own model specification, easing the verification of **Observable Determinism**.

In order to run this, a project name *proj* must be given as a command argument.
```
svROS launch -p $proj
```
The created models will be stored inside the project *proj* directory, in a sub-folder *models*.

#### Analyzing
