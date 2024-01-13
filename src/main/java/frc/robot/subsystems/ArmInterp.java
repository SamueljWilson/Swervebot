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
      new HtEntry(22.0, 0.16),
      new HtEntry(23.8, 0.18),
      new HtEntry(25.5, 0.185),
      new HtEntry(27.3, 0.195),
      new HtEntry(29.9, 0.215),
      new HtEntry(32.1, 0.235),
      new HtEntry(34.9, 0.255),
      new HtEntry(36.1, 0.265),
      new HtEntry(38.2, 0.275),
      new HtEntry(40.6, 0.29),
      new HtEntry(42.3, 0.31),
      new HtEntry(44.1, 0.33),
      new HtEntry(46.3, 0.365),
      new HtEntry(48.5, 0.4),
      new HtEntry(50.1, 0.425),
      new HtEntry(52.3, 0.46),
      new HtEntry(54.8, 0.49),
      new HtEntry(56.0, 0.51),
      new HtEntry(58.3, 0.545),
      new HtEntry(60.9, 0.6),
      new HtEntry(62.8, 0.645),
      new HtEntry(64.2, 0.685),
      new HtEntry(66.9, 0.75),
      new HtEntry(68.0, 0.775),
      new HtEntry(70.1, 0.82),
      new HtEntry(72.3, 0.865),
      new HtEntry(74.9, 0.935),
      new HtEntry(76.8, 0.995),
      new HtEntry(78.7, 1.055),
      new HtEntry(80.6, 1.12),
      new HtEntry(82.2, 1.18),
      new HtEntry(84.7, 1.26),
      new HtEntry(86.4, 1.3),
      new HtEntry(88.6, 1.36),
      new HtEntry(90.1, 1.4),
      new HtEntry(91.9, 1.46),
      new HtEntry(93.3, 1.515),
      new HtEntry(95.5, 1.58),
      new HtEntry(97.2, 1.625)
  };

  public static double getMaxCycles() {
    return htTable[htTable.length-1].cycles;
  }

  public static int cycleIndex(double cycles) {
    assert(cycles >= 0);
    int index = Arrays.binarySearch(htTable, new HtEntry(cycles, 0), (a,b) -> Double.compare(a.cycles, b.cycles));
    if (index < 0) {
      // Imperfect match
      index = -index - 1;
    }
    assert(index >= 0);
    return index;
  }

  public static double cyclesToHeight(double cycles) {
    int index = cycleIndex(cycles);
    if (index == 0) {
      return htTable[index].height;
    } else if (index == htTable.length) {
      return htTable[htTable.length-1].height;
    } else if (htTable[index].cycles == cycles) {
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
