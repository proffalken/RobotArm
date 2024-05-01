include <gear-lib.scad>
include <byj48.scad>
include <Nema17.scad>
include <servo.scad>

/***********************************************************\
*                                                           *
*   Usage:                                                  *
*                                                           *
*   Uncomment the appropriate part, render the file to stl  *
*   and print it                                            *
*                                                           *
\***********************************************************/


////// ONLY UNCOMMENT THE PARTS YOU NEED TO RENDER UNLESS
////// YOU NEED TO VIEW THE ENTIRE ASSEMBLY

// Display the modules without the motors in place
base();
arm1();
arm2();
wrist();

// Display the modules with the motors in place
/*
base(show_motors=1);
arm1(show_motors=1);
arm2(show_motors=1);
wrist(show_motors=1);
*/



/*************** MAIN ARM CODE *************************/

//// BASE Module
module base(show_motors=1){
    translate([0,6,0]){
        // Base Support
        translate([0,2,-20]) difference(){
            // main frame
            cube([60, 40, 60], center=true);
            // cutout
            translate([0,-5,5]) cube([50, 40, 60], center=true);
            // Main gear pivot
            translate([-50,-8,20]) rotate([0,90,0]) cylinder(d=4, h=100, $fn=100);
            // Nema17 Holes
            translate([0,40,-6.5]){   
                translate([15.5,0,15.5]) rotate([90,0,0]) cylinder(d=3, h=100, $fn=100);
                translate([-15.5,0,15.5]) rotate([90,0,0]) cylinder(d=3, h=100, $fn=100);
                translate([15.5,0,-15.5]) rotate([90,0,0]) cylinder(d=3, h=100, $fn=100);
                translate([-15.5,0,-15.5]) rotate([90,0,0]) cylinder(d=3, h=100, $fn=100);
            }
            // Main Nema17 Guide Shaft
            translate([0,40,-6.5]) rotate([90,0,0]) cylinder(d=22, h=100, $fn=100);
        }

        // WORM DRIVE bottom of arm (88)
        translate([0,15.20,-26.75]) rotate([90,0,0]) difference(){
            worm(25);
            translate([0,0,-20]) difference() {
                cylinder(h=200, r=2.5, $fn=260);
                translate([4.7, 0,0]) cube([5,5,200], center=true);
            }
        }

        // Nema17 shoulder
        if (show_motors == 1)
        {
            translate([0,69,-26.5]) rotate([90,0,0]) Nema17(47);
        }
    }
}
//// END BASE MODULE

//// 1st ARM
module arm1(show_motors = 1) {
    rotate([0,0,180]) {
        color("blue") {
            // Arm with built-in gear
            rotate([0,90,0]) {
                // Get the stepper so we know how wide the arms
                // need to be
                if (show_motors == 1){
                    translate([-75,-15,0]) rotate([0,-90,90]) stepper_28byj48();
                }
                // Main Cog
                translate([0,0,-2.25]) difference(){
                    gear(24,10,25);
                    cylinder(d=4, h=100, $fn=100);
                }
            }

            // WORM DRIVE top of arm (88)
            translate([0,16,83]) rotate([90,0,0]) difference(){
                worm(25);
                translate([0,0,-20]) difference() {
                    cylinder(h=200, r=2.5, $fn=260);
                    translate([4.7, 0,0]) cube([5,5,200], center=true);
                }
            }

            // ARM 2
            translate([-24,0,0]) color("blue") {
                rotate([0,90,0]){
                    difference(){

                        hull(){
                            cylinder(d=30.5,h=10,$fn=100);
                            translate([-100,0,0]) cylinder(d=30.5,h=10,$fn=100);
                        }
                        translate([-30,0,0]) hull(){
                            cylinder(d=20,h=10,$fn=100);
                            translate([-50,0,0]) cylinder(d=20,h=10,$fn=100);
                        }
                        // Stepper Holes
                        translate([-75,0,6.25]) rotate([90,0,0]) cylinder(d=4, h=100, $fn=100);
                        // Base Pivot
                        cylinder(d=4, h=10, $fn=100);
                        // Top pivot
                        color("red") translate([-100,0,0]) cylinder(d=4, h=10, $fn=100);

                        // Holes for 28BYJ-48
                        translate([10,0,80]) rotate([90,0,0]) cylinder(d=4, h=1000, $fn=100);
                    }
                }

            }

            translate([14,0,0]) {
                rotate([0,90,0]){
                    difference(){

                        hull(){
                            cylinder(d=30.5,h=10,$fn=100);
                            translate([-100,0,0]) cylinder(d=30.5,h=10,$fn=100);
                        }
                        translate([-30,0,0]) hull(){
                            cylinder(d=20,h=10,$fn=100);
                            translate([-50,0,0]) cylinder(d=20,h=10,$fn=100);
                        }
                        // Stepper Holes
                        translate([-75,10,3.5]) rotate([90,0,0]) cylinder(d=4, h=100, $fn=100);
                        // Base Pivot
                        cylinder(d=4, h=10, $fn=100);
                        // Top pivot
                        color("red") translate([-100,0,0]) cylinder(d=4, h=10, $fn=100);


                    }

                }
            }
            // Main Gear Pivot
            translate([-15,0,0]) rotate([0,90,0]) difference() { 
                cylinder(d=8, h=30, $fn=100);
                cylinder(d=4, h=30, $fn=100);
            }


            // Cross-braces
            // FRONT
            translate([0,12.75,30]) cube([30, 5, 10], center=true);
            // REAR
            translate([0,-12.75,60]) cube([30, 5, 10], center=true);

        }
    }
} ///// END MODULE /////


module arm2(show_motors=1){
    //////// ARM 2 /////////
    color("green") {
        // Gear
        translate([-2.25,0,100]) rotate([0,90,0]) difference(){
            gear(12,10,25);
            cylinder(d=4, h=100, $fn=100);
        }

        // Gear Pivot
        translate([-12.5,0,100]) rotate([0,90,0]) difference() { 
            cylinder(d=8, h=25, $fn=100);
            cylinder(d=4, h=25, $fn=100);
        }

        // ARMS
        translate([-13.75,0,100]) rotate([90,0,0])  {
            rotate([0,90,0]){
                difference(){

                    hull(){
                        cylinder(d=30.5,h=5,$fn=100);
                        translate([-100,0,0]) cylinder(d=30.5,h=5,$fn=100);
                    }
                    translate([-30,0,0]) hull(){
                        cylinder(d=20,h=10,$fn=100);
                        translate([-50,0,0]) cylinder(d=20,h=10,$fn=100);
                    }

                    // Base Pivot
                    cylinder(d=4, h=10, $fn=100);
                    // Top pivot
                    color("red") translate([-100,0,0]) cylinder(d=4, h=10, $fn=100);

                }
                // Additional Bearer    
                translate([0,0,-15.5]) {
                    difference(){

                        hull(){
                            cylinder(d=30.5,h=5,$fn=100);
                            translate([-100,0,0]) cylinder(d=30.5,h=5,$fn=100);
                        }
                        translate([-30,0,0]) hull(){
                            cylinder(d=20,h=10,$fn=100);
                            translate([-50,0,0]) cylinder(d=20,h=10,$fn=100);
                        }
                        // Base Pivot
                        cylinder(d=4, h=10, $fn=100);
                        // Top pivot
                        color("red") translate([-100,0,0]) cylinder(d=4, h=10, $fn=100);
                    }
                }

                // Bracer TOP
                translate([-80,10.25,-15]) difference(){
                    cube([35,5,20]);
                    translate([6,9,11.5]) rotate([90,0,0]) cylinder(d=4, h=100, $fn=100);
                }
                // Bracer Bottom
                translate([-70,-15.25,-15]) cube([25,5,20]);
            }
        }

        translate([8.75,0,100]) rotate([90,0,0]) color("green") {
            rotate([0,90,0]){
                difference(){

                    hull(){
                        cylinder(d=30.5,h=5,$fn=100);
                        translate([-100,0,0]) cylinder(d=30.5,h=5,$fn=100);
                    }
                    translate([-30,0,0]) hull(){
                        cylinder(d=20,h=10,$fn=100);
                        translate([-50,0,0]) cylinder(d=20,h=10,$fn=100);
                    }
                    // Base Pivot
                    cylinder(d=4, h=10, $fn=100);
                    // Top pivot
                    color("red") translate([-100,0,0]) cylinder(d=4, h=10, $fn=100);


                }

                // Additional Bearer    
                translate([0,00,15.5]) {
                    difference(){

                        hull(){
                            cylinder(d=30.5,h=5,$fn=100);
                            translate([-100,0,0]) cylinder(d=30.5,h=5,$fn=100);
                        }
                        translate([-30,0,0]) hull(){
                            cylinder(d=20,h=10,$fn=100);
                            translate([-50,0,0]) cylinder(d=20,h=10,$fn=100);
                        }
                        // Base Pivot
                        cylinder(d=4, h=10, $fn=100);
                        // Top pivot
                        color("red") translate([-100,0,0]) cylinder(d=4, h=10, $fn=100);
                    }
                }

                // Bracer TOP
                translate([-80,10.25,0]) difference(){
                    cube([35,5,20]);
                    translate([6,9,9]) rotate([90,0,0]) cylinder(d=4, h=100, $fn=100);
                }
                // Bracer Bottom
                translate([-70,-15.25,0]) cube([25,5,20]);


            }
        }


        // Stepper for wrist connection
        if (show_motors == 1){
            translate([0,-73.5,115]) rotate([0,0,180]) stepper_28byj48();
        }
        // WORM DRIVE Wrist Joint
        translate([0,-81.5,83]) rotate([0,0,0]) difference(){
            worm(25);
            translate([0,0,-20]) difference() {
                cylinder(h=200, r=2.5, $fn=260);
                translate([4.7, 0,0]) cube([5,5,200], center=true);
            }
        }
    }
}
module wrist(show_motors = 1){

    //////// WRIST
    color("red") {
        translate([-2.25,-100,100]) rotate([0,90,0]) difference(){
            gear(12,10,25);
            cylinder(d=4, h=100, $fn=100);
        }

        // Gear Pivot
        translate([-12.5,-100,100]) rotate([0,90,0]) difference() { 
            cylinder(d=8, h=25, $fn=100);
            cylinder(d=4, h=25, $fn=100);
        }













    }
} /// END WRIST MODULE

