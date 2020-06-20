resolution = 0.1;
errmin = 0.05;
rmin_m = 3.0;
xbounds = [0, 10];
ybounds = [0, 10];
outfilename = "test_flat_mprims.mprim";
genmprim_unicycle_circular_non_uniform(outfilename, resolution, errmin, rmin_m, xbounds, ybounds);