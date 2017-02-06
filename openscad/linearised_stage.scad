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
d=0.05; //a small distance (to avoid coincident points)
dz=0.5; //thickness required for one bridge - ideally just >2 layers
function zeroz(size) = [size[0], size[1], 0];

zflex = [5, 1.5, 0.75]; //dimensions of flexure (flex axis in XY plane)
flex_a = 0.12; //maximum flex angle in radians
sample_z = 40;
stage = [40,50,5]; //size of the moving platform
leg_top_h = 2.5; //how far above the flexure link the legs extend
flex_z_offset = leg_top_h + 2.5; //staggering in Z between the XY flexure stages
xy_lever = sample_z - stage[2]-flex_z_offset; //length of the effective lever
xy_travel = 2*xy_lever*flex_a; //maximum travel in +/- xy directions
bottom_stage = zeroz(stage) + [0, -2*(zflex[0]+zflex[1]+xy_travel), leg_top_h];



module xy_table_4legs(stage, lever, leg_top_h=leg_top_h, flexure_bottom=0, zflex=zflex, dz=dz){
    // Four legs and flexures to join them, making a two-parallelogram support for an XY stage
    // Legs are first linked along the Y direction, then the two pairs are bridged in the X direction.
    // stage is the size of the moving stage (only XY matter, Z if supplied is ignored)
    // lever is the length of the legs, measured between flexures
    // leg_top_h is the extra height above the upper flexures for stiffness
    // flexure_bottom is the Z position of the lower flexures
    // zflex is the size (X=width, Y=length, Z=thickness) of flexures
    // dz is the Z offset between X and Y flexures - it must be > 1 layer, preferably >2 layers.
    corner = zeroz(stage)/2; //the corner of the stage
    leg_pos = corner + [1, 1, 0] * zflex[1]; //the position of the leg (slightly out from stage)
    leg = [1, 1, 0] * zflex[0] + [0, 0, flexure_bottom + lever + leg_top_h]; //size of legs
    //legs
    reflect([1,0,0]) reflect([0,1,0]) translate(leg_pos) cube(leg);
    //flexure bridges between legs (top & bottom)
    translate([0,0,flexure_bottom]) reflect([1,0,0]) repeat([0,0,lever],2){
        hull() reflect([0,1,0]) translate(leg_pos) cube(zflex); //flexure bridge between legs
        hull() reflect([0,1,0]) translate(corner + [zflex[1],-d,0]) cube([zflex[0],d,leg_top_h]); //stiffen the middle
    }
    //flexures between pairs of legs (in X)
    translate([0,0,flexure_bottom]) reflect([0,1,0]) repeat([0,0,lever],2){
        hull() reflect([1,0,0]) translate(corner + [d,0,dz]) rotate(-90) cube(zflex);
    }
}

module mechanism_void(){
    leg = [1,1,0] * (zflex[0]+zflex[1]); //size of legs in XY
    travel = [1,1,0] * xy_travel; //distance travelled in XY
    clearance = [1,1,0]; //extra space added around mechanism (NB half of this is added each side)
    hull(){
        translate([0,0,0]){
            // bottom of mechanism (NB travels +/- xy_travel/2)
            cube(zeroz(stage) + 2*leg + travel + clearance + [0,0,d], center=true);
            cube(zeroz(bottom_stage) + 2*leg + travel + clearance + [0,0,d], center=true);
        }
        translate([0,0,sample_z]){
            // top of mechanism
            cube(zeroz(stage) + 2*leg + 2*travel + clearance + [0,0,stage[2]*2], center=true); //moves full range of travel
            cube(zeroz(bottom_stage) + 2*leg + clearance + [0,0,stage[2]*2], center=true); //doen't move
        }
    }
}
module casing(hollow=true){
    difference(){
        minkowski(){
            mechanism_void();
            cylinder(r=3,h=2*d, $fn=8);
        }
        if(hollow) mechanism_void();
        mirror([0,0,1]) cylinder(r=999,h=999,$fn=8);
        translate([0,0,sample_z - 2]) cylinder(r=999,h=999,$fn=8);
    }
}

union(){
    //outer casing
    casing();
    //bottom stage
    difference(){
        hull(){
            translate(-zeroz(stage)/2) cube(zeroz(stage) + [0, 0, flex_z_offset + dz + zflex[2]]);
            translate(-zeroz(bottom_stage)/2) cube(bottom_stage);
        }
    }
    //top stage
    translate([0,0,sample_z-stage[2]/2+dz]) difference(){
        cube(stage-[0,0,2*dz],center=true);
        repeat([10,0,0],4,center=true) repeat([0,10,0],4,center=true) cylinder(d=2.9, h=999);
    }
    //legs and flexures supporting top stage
    rotate(90) xy_table_4legs([stage[1],stage[0]], xy_lever, flexure_bottom=flex_z_offset);
    //legs and flexures supporting bottom stage
    xy_table_4legs(bottom_stage, xy_lever, flexure_bottom=0);
    //anchor for the XY mechanism (to the casing)
    intersection(){
        sequential_hull(){
            translate([0,0,xy_lever]) cube(zeroz(bottom_stage)+[0,0,2*leg_top_h], center=true);
            translate([0,0,xy_lever - leg_top_h]) cube([bottom_stage[0],bottom_stage[1],d], center=true);
            translate([0,0,xy_lever - leg_top_h]) cube([999,bottom_stage[1],d], center=true);
            translate([0,0,xy_lever - leg_top_h - 4]) cube([999,bottom_stage[1]-2,d], center=true);
        }
        casing(hollow=false);
    }
    //notches to hold thread (v1)
    reflect([0,1,0]) difference(){
        void_d = max(stage[1]+2*xy_travel, bottom_stage[1])/2 + zflex[0] + zflex[1] + 0.5;
        sequential_hull(){
            translate([-void_d-3, -void_d - 3, sample_z - 3]) cube([2*void_d+6, 3, 6]);
            intersection(){
                casing();
                translate([0,-void_d,sample_z - 15]) cube([999,10,d],center=true);
            }
        }
        reflect([1,0,0]) translate([void_d,-999,sample_z+0.5]) cube([10,999,10]);
        mechanism_void();
    }
    //motor mounts
    //translate([
}
