/*
Eigen.h
Brian R Taylor
brian.taylor@bolderflight.com
2017-02-08

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// Credits:   @rpavlik for writing the original header for the Eigen313 library, which this
// was derived from:
// http://forum.arduino.cc/index.php?PHPSESSID=a86gv50nb3e3ireijfmli63260&topic=144446.msg1089371#msg1089371

// Disable debug asserts.
#define EIGEN_NO_DEBUG 1

// Hint to number of registers
#define EIGEN_ARCH_DEFAULT_NUMBER_OF_REGISTERS 16

#ifdef A0
# define NEED_A0_RESTORED A0
# undef A0
#endif
#ifdef A1
# define NEED_A1_RESTORED A1
# undef A1
#endif
#ifdef B0
# define NEED_B0_RESTORED B0
# undef B0
#endif
#ifdef B1
# define NEED_B1_RESTORED B1
# undef B1
#endif
#ifdef round
# define NEED_round_RESTORED round
# undef round
#endif

namespace std {
	struct nothrow_t;
}

// Include main EIGEN Core header
#include <Eigen/Core>

#ifdef NEED_A0_RESTORED
# define A0 NEED_A0_RESTORED
# undef NEED_A0_RESTORED
#endif
#ifdef NEED_A1_RESTORED
# define A1 NEED_A1_RESTORED
# undef NEED_A1_RESTORED
#endif
#ifdef NEED_B0_RESTORED
# define B0 NEED_B0_RESTORED
# undef NEED_B0_RESTORED
#endif
#ifdef NEED_B1_RESTORED
# define B1 NEED_B1_RESTORED
# undef NEED_B1_RESTORED
#endif
#ifdef NEED_round_RESTORED
# define round NEED_round_RESTORED
# undef NEED_round_RESTORED
#endif
