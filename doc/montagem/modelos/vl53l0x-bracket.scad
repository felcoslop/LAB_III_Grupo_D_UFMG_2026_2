/* $Id: vl53l0x-bracket.scad,v 1.2 2022/02/03 19:36:18 tobi Exp $ */

$fn=123;

// PCB test bracket for the CJ VL53L0X V2 module from AZ-Delivery
// FIXME: needs a cut-out for pin headers and/or cabling

difference() {
  hull() { // base plate
    translate([2,0,0]) cube([32-2*2, 20, 3.5]);
    translate([0,2,0]) cube([32, 20-2*2, 3.5]);
  }

  translate([32/2-16.5/2, 20/2-13/2, 1.5])
    cube([16.5,13, 3]); // pcb

  translate([32/2-16.5/2, 20/2-13/2, 2.25]) hull() { // ears
    translate([-5/2+.5, 13/2, 0]) cylinder(r=5.5/2, h=3);
    translate([16.5+5/2-.5, 13/2, 0]) cylinder(r=5.5/2, h=3);
  }

  // m3+ screw holes  
  translate([32/2-16.5/2, 20/2-13/2, -.01]) {
    translate([-5/2+.5, 13/2, 0]) cylinder(r=3.5/2, h=3.02);
    translate([16.5+5/2-.5, 13/2, 0]) cylinder(r=3.5/2, h=3.02);
  }

  // sensor window
  translate([32/2-6/2, 20/2-13/2+2.75, -.01])
    cube([6, 3, 3.02]);

}
