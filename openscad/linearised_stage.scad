/*

OpenFlexure long travel XY stage

This is a linearised flexure stage with twice the nominal travel of
the standard microscope stage of the same height.  Actuation is a
work in progress (!)

The basic principle is of stacked parallelograms - both deform at
the same rate, meaning the vertical displacements cancel and we get
proper linear motion.  This is dependent on the stiffness of the 
flexure links - i.e. the vertical stiffness, in a pin-joint model, is
not infinite.  However, as the linear stiffness scales with lever
length squared (assuming constant torsional stiffness from the hinge)
we'd expect the stiffness in Z to be something like 400x stiffer than
XY, so I'm hoping all will be well...

*/
use <utilities.scad>;
d=0.05;

zflex = [5, 1.5, 0.75]; //dimensions of flexure (flex axis in XY plane)
flex_a = 0.12; //maximum flex angle in radians
sample_z = 40;
stage = [40,20,5]; //size of the moving platform
xy_lever = sample_z - stage[2]; //length of the effective lever
xy_travel = 2*xy_lever*flex_a; //maximum travel in +/- xy directions
bottom_stage = [stage[1], stage[1]+2*xy_travel+2*3, stage[2]];
mech_void = [stage[0], bottom_stage[1], sample_z+d] + [2,2,0]*(zflex[1]+zflex[0]+xy_travel/2+0.5);
flex_z2 = sample_z - stage[2];
dz=0.5;

union(){
    difference(){
        //outer casing
        translate([0,0,sample_z/2-1]) cube(mech_void + [6,6,-2-d], center=true);
        translate([0,0,sample_z/2]) cube(mech_void, center=true);
    }
    //bottom stage
    difference(){
        translate([0,0,stage[2]/2]) hull(){
            cube(stage, center=true);
            cube(bottom_stage, center=true);
        }
    }
    //top stage
    translate([0,0,sample_z-stage[2]/2+dz]) cube(stage-[0,0,2*dz],center=true);
    //legs and flexures supporting top stage
    for(corner=[[stage[0],stage[1],0]/2]){
        //legs
        reflect([1,0,0]) reflect([0,1,0]) translate(corner + [1,1,0]*zflex[1]) cube([1,1,0]*zflex[0]+[0,0,sample_z-2]);
        //flexure bridges between legs (top & bottom)
        reflect([1,0,0]) repeat([0,0,flex_z2],2){
            hull() reflect([0,1,0]) translate(corner + [1,1,0]*zflex[1]) cube(zflex);
            hull() reflect([0,1,0]) translate(corner + [zflex[1],-d,0]) cube([zflex[0],d,3]);
        }
        reflect([0,1,0]) repeat([0,0,flex_z2],2){
            hull() reflect([1,0,0]) translate(corner + [d,0,dz]) rotate(-90) cube(zflex);
        }
        echo(corner);
    }
    //legs and flexures supporting bottom stage
    rotate(90) for(corner=[[bottom_stage[1],bottom_stage[0],0]/2]){
        //legs
        reflect([1,0,0]) reflect([0,1,0]) translate(corner + [1,1,0]*zflex[1]) cube([1,1,0]*zflex[0]+[0,0,sample_z-2]);
        //flexure bridges between legs (top & bottom)
        reflect([1,0,0]) repeat([0,0,flex_z2],2){
            hull() reflect([0,1,0]) translate(corner + [1,1,0]*zflex[1]) cube(zflex);
            hull() reflect([0,1,0]) translate(corner + [zflex[1],-d,0]) cube([zflex[0],d,3]);
        }
        reflect([0,1,0]) repeat([0,0,flex_z2],2){
            reflect([1,0,0]) translate(corner + [-d,0,dz]) rotate(-90) cube(zflex + [0,2*d,0]);
        }
        echo(corner);
    }
    //anchors for the flexures (on the casing)
    reflect([0,1,0]) translate([0,0,flex_z2]) sequential_hull(){
        bs = bottom_stage;
        dy = (mech_void[1]-bs[1])/2;
        translate([-bs[0]/2, bs[1]/2-3, 0]) cube([bs[0], 3, 3]);
        translate([-bs[0]/2+1, bs[1]/2-3, -4]) cube([bs[0]-2, 3, 2]);
        translate([-bs[0]/2+1, bs[1]/2-3, -4]) cube([bs[0]-2, dy+3+d, 2]);
        translate([-bs[0]/2+1+dy*flex_a, bs[1]/2+dy, -dy-4]) cube([bs[0]-2-2*dy*flex_a, d, 3]);
    }
}
