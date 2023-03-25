// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

/** Add your docs here. */
public class ArmInterp {
  static class HtEntry{double cycles; double height; public HtEntry(double c, double h) {cycles = c; height = h;}}
  static final HtEntry[] htTable = {
      new HtEntry(0, 0),
      new HtEntry(80, 1.5)
  };

  public static int cycleIndex(double cycles) {
    assert(cycles >= 0);
    int index = Arrays.binarySearch(htTable, new HtEntry(cycles, 0), (a,b) -> Double.compare(a.cycles, b.cycles));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    assert(index >= 0);
    assert(index < htTable.length);
    return index;
  }

  public static double cyclesToHeight(double cycles) {
    int index = cycleIndex(cycles);
     if (index == 0) {
      return htTable[index].height;
     }
     else if (htTable[index].cycles == cycles) {
       return htTable[index].height;
     }
     // Linear Interpolation
     else {
      double c0 = htTable[index-1].cycles;
      double c1 = htTable[index].cycles;
      double h0 = htTable[index-1].height;
      double h1 = htTable[index].height;
      double scaler = (cycles-c0) / (c1 - c0);
      double heightRange = h1 - h0;
      return h0 + scaler*heightRange;
     }
  }

  public static int heightIndex(double height) {
    assert(height >= 0);
    int index = Arrays.binarySearch(htTable, new HtEntry(0, height), (a,b) -> Double.compare(a.height, b.height));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    // System.out.printf("HEIGHT: %f -- INDEX: %d\n", height, index);
    assert(index >= 0);
    assert(index < htTable.length);
    return index;
  }
  
  public static double heightToCycles(double height) {
    int index = heightIndex(height);
    if (index == 0) {
      return htTable[index].cycles;
     }
     else if (htTable[index].height == height) {
       return htTable[index].cycles;
     }
     // Linear Interpolation
     else {
      double c0 = htTable[index-1].cycles;
      double c1 = htTable[index].cycles;
      double h0 = htTable[index-1].height;
      double h1 = htTable[index].height;
      double scaler = (height-h0) / (h1 - h0);
      double cycleRange = c1 - c0;
      double cycles = c0 + scaler*cycleRange;
      // System.out.printf("HEIGHT: %f, -- CYCLES: %f\n", height, cycles);
      return cycles;
     }
  }

  public static double getCyclesPerMeter(double height) {
    if (height == 0.0) {
      return 0.0;
    }
    int indexGE = heightIndex(height);
    assert(indexGE > 0);
    int indexLT = indexGE - 1;
    double deltaC = htTable[indexGE].cycles - htTable[indexLT].cycles;
    double deltaH = htTable[indexGE].height - htTable[indexLT].height;
    return deltaC / deltaH;
  }

  public static double vheightToRPM(double metersPerSecond, double height) {
    double slope = getCyclesPerMeter(height);
    double secondsPerMinute = 60;
    return slope * metersPerSecond * secondsPerMinute;
  }
}
