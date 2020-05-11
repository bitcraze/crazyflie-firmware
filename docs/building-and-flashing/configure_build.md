---
title: Configure the build
page_id: build_config
---

The build can be configured to generate binaries with different properties. Common uses cases are to enable/disable functionality or setting default values.

## config.mk

The prefered way of setting compile time options is to use the `config.mk` file. `config.mk` is included by the Makefile but is excluded from the file versioning tree to avoid permanent configuration changes by accident.

There is no `config.mk` by default, you have to create it yourself. Simply add an empty file in `tools/make/config.mk` or copy/rename `tools/make/config.mk.example` that contains common configuration examples.

Add the options youu want, for instance:

```
PLATFORM=CF2
DEBUG=1
```

## On the command line

It is also possible to add compile options on the command line, for instance

```make DEBUG=1```
