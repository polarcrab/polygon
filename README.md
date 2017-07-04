# Polygon
This is a low level library written in C++, which allows us to create convex and concave hull.
The convex hull is based on the Graham's scan method and for finding the concave hull we have used the method given by [this paper](http://www.iis.sinica.edu.tw/page/jise/2012/201205_10.pdf).

### Dependencies
```sh
sudo apt-get install libboost-python-dev
```
### Building
```
make
```
### Example

An example code for python has been added, which demonstrates how to build concave as well as convex hull.
To run the example use the following code, after building
```
python3 example/example.py
```
In the following example, the concave as well as convex hull can be seen.
The image has been drawn using matplotlib 
![Example Image](/example/example.jpg?raw=true "Example Image")