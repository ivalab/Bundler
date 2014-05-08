Install on IVALab Machines
==========================

On a fresh install of kubuntu/ubuntu (12.04), a few libraries need to be
installed for Bundler to compile (besides gcc and g++).  Basically Bundler
needs: LAPACK, BLAS, CBLAS, minpack, and f2c.  Install the following
through apt-get:

```bash
> apt-get -y install gfortran liblapack-dev libatlas-dev \
  libminpack1 minpack-dev libcminpack-dev f2c
```

These might be overkill, but they should do the trick.  In principle, you 
should be able to use ATLAS (which is basically a tuned LAS).  If that does
not work out, then install BLAS.

```bash
> apt-get -y install libblas-dev \
```

Running make in the proper directory should work out.  I do not have it
working with cmake just yet, so don't bother with that.  If you do want
to try, then what I've done to make it work is:

```bash
> mkdir build
> cd build
> cmake ../
> make
```

at which point, the compilation will crash due to a missing "-lblas" linker
option.  Should happen about 3 just got into the src sub-directory and
copy/paste the compile command with "-lblas" added to the command line.  It
will compile fine.  Go back up to the build directory and then type
make again.  It will then finish compiling.

I don't know cmake well enough to figure out how to fix the problem.
Once I deconstruct the Bundler code, I should have a better fix.
[2014/05/07].
