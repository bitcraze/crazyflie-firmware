---
title: Unit testing
page_id: unit_testing
---

## Dependencies

Frameworks for unit testing and mocking are pulled in as git submodules.

The testing framework uses ruby and rake to generate and run code. 

To minimize the need for installations and configuration, use the docker builder
image (bitcraze/builder) that contains all tools needed. All scripts in the 
tools/build directory are intended to be run in the image. The 
[toolbelt](https://wiki.bitcraze.io/projects:dockerbuilderimage:index) makes it
easy to run the tool scripts.


## Running all unit tests
    
With the environment set up locally

        make unit
        
with the docker builder image and the toolbelt

        tb make unit
        
## Running one unit test
       
When working with one specific file it is often convenient to run only one unit test
       
       make unit FILES=test/utils/src/test_num.c

or with the toolbelt        

       tb make unit FILES=test/utils/src/test_num.c
              
## Running unit tests with specific build settings
      
Defines are managed by make and are passed on to the unit test code. Use the 
normal ways of configuring make when running tests. For instance to run test
for Crazyflie 1

      make unit LPS_TDOA_ENABLE=1
