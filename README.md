**ModelFit**
---------
---------
An offline fitting tool for fitting a linear-blend-skinning model to the Primesense Carmine depth cloud.

**Overview**
--------

This project is the offline model fitting portion of "Real-Time Continuous Pose Recovery of Human Hands Using Convolutional Networks".  This code is provided **as-is**... The code is not inherently bad, it's just not well organized or user-friendly.

The work-flow is the following:  

1. If you're using multiple Primesense devices (which you probably should) use the ```CalibrateKinects``` project to first register the cameras.  You should capture a few frames of static "calibration geometry"; I use an icosahedron model for this (<http://www.korthalsaltes.com/model.php?name_en=icosahedron>), but you could code up whatever you thought might work.  This will fit the model roughly using the same PSO fitting algorithm that is described in the paper, then a few iterations of ICP will be run for fine tuning.
2. Then run the ```ModelFit``` project to fit the individual frames of your dataset.  In this example I am using a LBS hand model.

Example data for both these steps is baked into the repo in ```ModelFit/data/```.  Note that the UI is not all that intuitive and for this I apologize.  It is driven by a combination of keystrokes and mouse movements.  At the start of each program a list of possible keystrokes will be printed to the command window.

**Compilation and Running**
---------------

I have baked in the pre-compiled dependencies for Win 7/8/8.1 64bit + Visual Studio 2012 (which is the only OS and compiler supported anyway).  You need to unzip the lib files in ```ModelFit\lib\WIN7.zip```.

After unzipping ```WIN7.zip```, ```ModelFit.sln``` should compile and run.

**Style**
---------

This project follows the Google C++ style conventions: 

<http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml>
