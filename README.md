**ModelFit - An offline fitting tool for fitting a linear-blend-skinning model to the Kinect depth cloud**
---------
---------

**Overview**
--------

This project is the offline model fitting described in "Real-Time Continuous Pose Recovery of Human Hands Using Convolutional Networks".  This code is provided "as-is"... That is the code is not "bad", it's just not well organized.

The work-flow is the following:  

1. If you're using multiple kinetics use the "CalibrateKinects" project to first register the cameras.  You should capture a few frames of static "calibration geometry"; I use an icosahedron model for this (http://www.korthalsaltes.com/model.php?name_en=icosahedron), but you could code up whatever you thought might work.  This will fit the model roughly using the same PSO fitting algorithm that is described in the paper followed by an ICP step to register the cameras.
2. Then run the "ModelFit" project to fit the individual frames of your dataset.

Example data for both these steps is baked into the repo.

**Compilation and Running**
---------------

TODO

**Style**
---------

This project follows the Google C++ style conventions: 

<http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml>
