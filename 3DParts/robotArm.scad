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
//platform(show_motors = 1, show_stands = 1);
//base();
//arm1();
//arm2();
// wrist();
//pallet_forks();

// Display the modules with the motors in place
//
//platform(show_motors = 1);
//base(show_motors = 1);
//arm1(show_motors = 1);
//arm2(show_motors = 1);
//wrist(show_motors = 1);
pallet_forks(show_motors = 1);
//*/



/*************** MAIN ARM CODE *************************/

//// platform Module
module platform(show_motors = 0, show_stands = 0){
    translate([0,0,-70]){
        // Main Platform
        difference(){
            translate([-50,-50,-5]) cube([100,150,5]);
            // Bearing (51105)
            translate([0,0,-1]) difference() {
                cylinder(d=42, h=2, $fn=100);
                cylinder(d=25, h=2, $fn=100);
            }
            // Base Pivot
            translate([0,0,-50]) cylinder(d=4, h=100, $fn=100);
        
        // NEMA 17 Mount
        translate([0,67,-6.5]) {   
                translate([15.5,15.5,0]) cylinder(d=3, h=100, $fn=100);
                translate([-15.5,15.5,0]) cylinder(d=3, h=100, $fn=100);
                translate([15.5,-15.5,0]) cylinder(d=3, h=100, $fn=100);
                translate([-15.5,-15.5,0]) cylinder(d=3, h=100, $fn=100);
         }
         // Main Nema17 Guide Shaft
         translate([0,67,-6.5]) rotate([0,0,0]) cylinder(d=22, h=100,$fn=100);
     }
         // NEMA 17 Motor
        if (show_motors == 1)
        {
            translate([0,67,-52]) Nema17(47);
        }
        // Motor Gear
        translate([0,67,16.5]) difference(){
            rotate([0,0,7]) gear(24,10,25);
            difference() {
                cylinder(h=200, r=2.5, $fn=260);
                translate([4.7, 0,0]) cube([5,5,200], center=true);
            }
            
        }
        if ( show_stands == 1){
            // Front Right
            translate([45,-50,-52]) cube([5, 40, 52]);
            // Front Left
            translate([-50,-50,-52]) cube([5, 40, 52]);
        }
    }
}

//// BASE Module
module base(show_motors = 0){
    translate([0,6,0]){
        // Base Support
        translate([0,2,-20]) difference(){
            // main frame
            cube([60, 40, 60], center=true);
            // cutout
            translate([0,-5,5]) cube([50, 40, 60], center=true);
            // Base Pivot
            translate([0,-8,-60]) cylinder(d=4, h=100, $fn=100);
            // Arm pivot
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
        // Base gear
        translate([0,-6,-54.5]) difference() {
            // Gear
            gear(48,10,30);
            // Pivot
            translate([0,0,-10]) cylinder(d=4, h=100, $fn=100);
            // Bearing (51105)
            difference() {
                cylinder(d=42, h=2, $fn=100);
                cylinder(d=25, h=2, $fn=100);
            }
        }
            
    }
}
//// END BASE MODULE

//// 1st ARM
module arm1(show_motors = 0) {
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
                        color("red") translate([25,0,0]) cylinder(d=4, h=10, $fn=100);

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


module arm2(show_motors = 0){
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


        // Stepper for pallet_forks connection
        if (show_motors == 1){
            translate([0,-73.5,115]) rotate([0,0,180]) stepper_28byj48();
        }
        // WORM DRIVE pallet_forks Joint
        translate([0,-81.5,83]) rotate([0,0,0]) difference(){
            worm(25);
            translate([0,0,-20]) difference() {
                cylinder(h=200, r=2.5, $fn=260);
                translate([4.7, 0,0]) cube([5,5,200], center=true);
            }
        }
    }
}

/*********** WRIST *****************/
module wrist(show_motors = 0){

    //////// wrist
    color("red") {
        // GEAR
        translate([-2.25,-100,100]) rotate([0,90,0]) difference(){
            gear(12,10,25);
            cylinder(d=4, h=100, $fn=100);
        }

        // Gear Pivot
        translate([-8,-100,100]) rotate([0,90,0]) difference() { 
            cylinder(d=8, h=16, $fn=100);
            cylinder(d=4, h=16, $fn=100);
        }

        // FRAME
             translate([-24,-100,70]) color("blue") {
                 // Attachment to pivot
                translate([15.5,-25,15]) difference(){
                    cube([5,30,20]);
                    translate([0,25,15]) rotate([0,90,0]) cylinder(d=4, h=10, $fn=100);
                }
                translate([27.5,-25,15]) difference() {
                    cube([5,30,20]);
                    translate([0,25,15]) rotate([0,90,0]) cylinder(d=4, h=10, $fn=100);
                }
                 // Beam and Arm Connectors
                translate([1.5,-25,5]) cube([8,10,30]);
                translate([39.5,-25,5]) cube([8,10,30]);
                 // BEAM
                translate([1.5,-25,15]) cube([40,5, 20]);
                // Pivot Motor
                translate([24,-35,30]) rotate([90,0,180]) stepper_28byj48();
                
                rotate([0,90,0]){
                    difference(){

                        hull(){
                            cylinder(d=30.5,h=10,$fn=100);
                            translate([-30,0,0]) cylinder(d=30.5,h=10,$fn=100);
                        }
                      
                        // Top Pivot
                        translate([-30,0,0]) cylinder(d=4, h=10, $fn=100);
                    }
                }

            }

            translate([14,-100,70]) {
                rotate([0,90,0]){
                    difference(){

                        hull(){
                            cylinder(d=30.5,h=10,$fn=100);
                            translate([-30,0,0]) cylinder(d=30.5,h=10,$fn=100);
                        }
                        // TOP Pivot
                        translate([-30,0,0]) cylinder(d=4, h=10, $fn=100);
                    }
                }
            }
          }
} /// END wrist MODULE

module pallet_forks(show_motors = 0){

    //////// pallet_forks
    color("red") {
        // GEAR
        translate([-2.25,-100,100]) rotate([0,90,0]) difference(){
            gear(12,10,25);
            cylinder(d=4, h=100, $fn=100);
        }

        // Gear Pivot
        translate([-8,-100,100]) rotate([0,90,0]) difference() { 
            cylinder(d=8, h=16, $fn=100);
            cylinder(d=4, h=16, $fn=100);
        }

        // FRAME
             translate([-24,-100,70]) color("blue") {
                 // Attachment to pivot
                translate([15.5,-25,15]) difference(){
                    cube([5,30,20]);
                    translate([0,25,15]) rotate([0,90,0]) cylinder(d=4, h=10, $fn=100);
                }
                translate([27.5,-25,15]) difference() {
                    cube([5,30,20]);
                    translate([0,25,15]) rotate([0,90,0]) cylinder(d=4, h=10, $fn=100);
                }
                 // Beam and Arm Connectors
                translate([1.5,-25,5]) cube([8,10,30]);
                translate([39.5,-25,5]) cube([8,10,30]);
                 // BEAM
                translate([1.5,-25,15]) cube([40,5, 20]);
                // Forks
                translate([1.5,-65,5]) cube([8,45, 5]);
                translate([39.5,-65,5]) cube([8,45, 5]);
                
                rotate([0,90,0]){
                    difference(){

                        hull(){
                            cylinder(d=30.5,h=10,$fn=100);
                            translate([-30,0,0]) cylinder(d=30.5,h=10,$fn=100);
                        }
                      
                        // Top Pivot
                        translate([-30,0,0]) cylinder(d=4, h=10, $fn=100);
                    }
                }

            }

            translate([14,-100,70]) {
                rotate([0,90,0]){
                    difference(){

                        hull(){
                            cylinder(d=30.5,h=10,$fn=100);
                            translate([-30,0,0]) cylinder(d=30.5,h=10,$fn=100);
                        }
                        // TOP Pivot
                        translate([-30,0,0]) cylinder(d=4, h=10, $fn=100);
                    }
                }
            }
          }
} /// END pallet_forks MODULE

