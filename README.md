Contributers:
Daniel Monascal
Roy Chun
Alex Levin
Jalal Kahn
Patrick Reynolds

### This is a cloned REPO so as to not bother former contributers with intended commits. 

### A project dedicated to the creation of workable 3D obstacle mapping for use Drones or other autonomous urban vehicles. As of now, It is no longer being worked on due to time constraints and lack of access to capital, but contains the workings for the creation of LAS files, which are what the main algorithm uses, and the functions for clustering, grouping, and removing 3D points. Additionally, though the use of ARCGIS it is possible to overlay said points if they contain the proper GPS coordianates. Unfortunately, that functionality is not implemented optimally.

It is possible this project will be resumed in the coming months, unfortunately only 2 original contributers expressed interest in this.


### To Run Main algorithm
python create_database.py [las file] [xml file]

### Currently, to produce a LAS file, this project used MicMac. Installation is found on the site (linked below)
https://micmac.ensg.eu/index.php/Install_MicMac_Windows

As long as your 3d object is LAS, it should run. So any software that can do that will suffice

### To run server:
python server.py in CMD/Terminal
Running Daniel's algorithm will send to the database, which is hosted on Alex's computer
Change IP address in the code to send to a different server
