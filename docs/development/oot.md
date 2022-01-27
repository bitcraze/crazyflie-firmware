---
title: Out of tree build
page_id: oot
---

It is possible to have an out-of-tree build of parts of the crazyflie firmware. This enables developers to work on elements without worrrying about merging it with the full code base. 

# App layer.
Technically the app layer is an example of an out of tree build. Follow the [app layer instructions](/docs/userguides/app_layer.md) for this.

# OOT estimators
In a seperate folder make a Makefile which contain the following content:

```
CFLAGS += -DOOT_ESTIMATOR

VPATH += src/
PROJ_OBJ += estimator_out_of_tree.o

CRAZYFLIE_BASE=[LOCATION OF THE CRAZYFLIE FIRMWARE]
include $(CRAZYFLIE_BASE)/Makefile
```  

in [your_estimator_out_of_tree].c in the src folder you will just need to make sure that the following functions are filled:

* ```init = estimatorOutOfTreeInit```
* ```test = estimatorOutOfTreeTest```
* ```update = estimatorOutOfTree```
* ```name = "OOT```

# OOT Controllers

Not yet implemented. Please request this feature in the [crazyflie-firmware issue list](https://github.com/bitcraze/crazyflie-firmware/issues)
