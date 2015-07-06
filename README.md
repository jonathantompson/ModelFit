**ModelFit**
---------
---------
The offline fitting tool described in my paper "Real-Time Continuous Pose Recovery of Human Hands Using
Convolutional Networks" (Jonathan Tompson, Murphy Stein, Yann Lecun, Ken Perlin).  This code was used to fit the frames in the NYU Hand Pose Dataset <http://cims.nyu.edu/~tompson/NYU_Hand_Pose_Dataset.htm>.  Note that I used a Primesense Carmine 1.09 to capture the depth cloud and RGB, however this code shouldn't be limited to this device (as long as you adjust the relevant camera parameters).

**Overview**
--------

A word of warning: this code is provided **as-is**.  While the quality of the code is not inherently bad, it's just not well organized or user-friendly.

Note that I have made available example data at <http://blackbox.cs.nyu.edu/model_fit_example_data.zip>.  I highly suggest you download it and extract it into the ```ModelFit/data/``` directory as it is all set up and ready to go.

Lastly, I want to say that the UI is not all that intuitive or easy to use, and for this I apologize.  It is driven by a combination of keystrokes and mouse movements.  At the start of each program a list of possible keystrokes will be printed to the command window.

The work-flow is described below.  

----------
Part 1
------

* If you're using multiple Primesense devices use the ```CalibrateKinects``` project to first register the cameras (otherwise go to part 2). 

    - This program first fits a calibration model roughly using the same PSO fitting algorithm that is described in the paper, then a few iterations of ICP will be run to fine tune the extrinsic camera parameters.
    - You should capture approximately 10 frames of static "calibration geometry" in the same png format that I use for the "NYU Hand Pose Database". I use a big icosahedron model for this (<http://www.korthalsaltes.com/model.php?name_en=icosahedron>), but you could make whatever structure you think might work. 
    - Place the data in ```ModelFit/data/calib``` and run  ```CalibrateKinects```.
    - At startup the first kinect point cloud will be shown.  Use right-mouse + drag to move the icosahedron model into place.
    - Press ```[``` and ```]``` keys to cycle through the 6 other rigid-body-transform coefficients and adjust the other coefficients so that the model is approximately in place.
    - Press ```i``` to switch to the second and third kinect views.  Repeat the steps above to move the model into place.
    - Press ```f``` to fit the model to the point cloud.  After fitting, switch between views to verify the fit (it only needs to be approximate).  Press ```h``` to save the fitted 6DOF coeffs to disk.
    - Select kinect 1 using the ```i``` key.  Press ```c``` to run ICP between kinect 0 and 1.
    - Select kinect 2 using the ```i``` key.  Press ```c``` to run ICP between kinect 0 and 2.
    - Return to kinect 0 view and press ```x``` to overlay all three views to visually check the frames are OK.
    - The extrinsic camera parameters are automatically saved to disk.
    

----------
Part 2
------

* The ```ModelFit``` project is the main workhorse of this codebase and implements the PSO fitting algorithm described in the paper.  In this example I am using a LBS hand model.

    - At startup the point cloud from the first kinect is rendered.
    - Use a combination of the right mouse button + drag to change coefficient values as well as ```[``` and ```]``` to change the coefficient currently being modified.  Adjust the coefficients until the hand approximately aligns with the point cloud.
    - Press the ```f``` button to fit the current frame.  You may need to readjust the coefficients manually and repeat fitting a few times.
    - Once the first frame looks good, press ```h``` to save the coefficients to disk.
    - Press ```g``` to automatically cycle through the remaining frames, fitting each frame and saving the coefficients to disk.
    - You might find that the PSO optimizer finds a bad fit for some difficult or ambiguous frames (particularly if a finger is missing from all kinect views).  In this case you will have to re-seed the coefficients manually and restart the automated fitting.
    - Finally, press ```l``` to go back to the first frame (or ```+``` and ```-``` to cycle through 1 frame at a time), then press ```p``` to play back the fitted frames in sequence.

**Compilation and Running**
---------------

I have baked in the pre-compiled dependencies for Win 7/8/8.1 64bit + Visual Studio 2012 (which is the only OS and compiler supported anyway). You need to unzip the lib files in: ```ModelFit\lib\WIN7.zip```.  The dependancies are all cross platform (linux, windows and OS X):  [assimp](http://assimp.sourceforge.net/), [freeimage](http://freeimage.sourceforge.net/), [glfw](http://www.glfw.org/), [zeromq](http://zeromq.org/) if you want to pre-compile them from scratch for a different platform.

After unzipping ```WIN7.zip```, ```ModelFit.sln``` should compile and run.

**Style**
---------

This project follows the Google C++ style conventions: 

<http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml>

**License**
-----------
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
