---
title: Unit testing
page_id: unit_testing
---

### Dependencies

Frameworks for unit testing and mocking are pulled in as git submodules.

The testing framework uses ruby and `rake` as well as `libasan` (AddressSanitizer) to generate and run code.
If you run the tests on your own machine you will have to install them.

To minimize the need for installations and configuration, use the docker builder
image (bitcraze/builder) that contains all tools needed. All scripts in the
tools/build directory are intended to be run in the image. The
[toolbelt](https://github.com/bitcraze/toolbelt) makes it
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

The unit tests are affected by the settings in kbuild and the defines that are set in the configuration will be
used in the unit tests as well. In some cases unit tests must be disabled based on the configuration, for instance
if a particular file is not included in the build. A unit test file can be disabled based on configuration by using
an annotation like this:

``` c
// @IGNORE_IF_NOT CONFIG_DECK_LIGHTHOUSE
```
