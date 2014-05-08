Install on IVALab Machines
==========================

## Kubuntu/Ubuntu (12.04 LTS)
On a fresh install of kubuntu/ubuntu (12.04), a few libraries need to be
installed for Bundler to compile (besides gcc and g++).  Basically Bundler
needs: LAPACK, BLAS, CBLAS, minpack, and f2c.  This forked version seems to
have CBLAS caked into it, so let's roll with that one (getting CBLAS to
compile takes a little magic, at least to get the samples compiled too).
Install the following through apt-get:

```bash
> apt-get -y install gfortran liblapack-dev libatlas-dev libatlasa3gf-base \
  libminpack1 minpack-dev libcminpack-dev f2c
```

These might be overkill, but they should do the trick.  In principle, you 
should be able to use ATLAS (which is basically a tuned LAS).  If there
are any problems, just use "apt-cache search" to find the right packages.
If ATLAS does not work out, then install BLAS.

```bash
> apt-get -y install libblas-dev libblas3gf \
```

Running make in the proper directory should work out.  I do not have it
working perfectly with cmake just yet, so don't bother with that.  If you do
want to try, then what I've done to make it work is:

```bash
> mkdir build
> cd build
> cmake ../
> make
```

at which point, the compilation will crash due to a missing "-lblas" linker
option.  Should happen about 3 times.  Just go into the src sub-directory
and copy/paste the compile command with "-lblas" added to the command line.
It will compile fine.  Go back up to the build directory and then type make
again.  It will then finish compiling.

If using ATLAS, then you may want to read up in
/usr/share/doc/libatlas3gf-base/README.Debian for alternating between
the standard BLAS and the ATLAS/BLAS versions.

I don't know cmake well enough to figure out how to fix the problem.
Once I deconstruct the Bundler code, I should have a better fix.
[2014/05/07].

Note that the project comes with a pre-built SIFT implementation.  To
be able to fully modify on ones own, the SIFT code will also have to be
obtained.  See the Bundler documentation for more info.
