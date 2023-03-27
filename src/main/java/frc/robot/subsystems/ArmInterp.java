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
      new HtEntry(20.0, 0.19),
      new HtEntry(24.0, 0.21),
      new HtEntry(27.1, 0.225),
      new HtEntry(29.9, 0.245),
      new HtEntry(32.0, 0.26),
      new HtEntry(34.5, 0.285),
      new HtEntry(36.1, 0.305),
      new HtEntry(37.2, 0.315),
      new HtEntry(39.4, 0.34),
      new HtEntry(40.3, 0.35),
      new HtEntry(42.7, 0.38),
      new HtEntry(44.0, 0.4),
      new HtEntry(45.3, 0.415),
      new HtEntry(46.5, 0.44),
      new HtEntry(47.5, 0.45),
      new HtEntry(48.7, 0.47),
      new HtEntry(50.0, 0.5),
      new HtEntry(51.2, 0.52),
      new HtEntry(52.4, 0.545),
      new HtEntry(53.3, 0.56),
      new HtEntry(54.5, 0.585),
      new HtEntry(56.7, 0.635),
      new HtEntry(57.9, 0.66),
      new HtEntry(58.9, 0.68),
      new HtEntry(60.3, 0.715),
      new HtEntry(61.6, 0.74),
      new HtEntry(62.0, 0.755),
      new HtEntry(63.2, 0.785),
      new HtEntry(64.0, 0.805),
      new HtEntry(65.2, 0.835),
      new HtEntry(66.4, 0.875),
      new HtEntry(67.3, 0.9),
      new HtEntry(68.4, 0.935),
      new HtEntry(69.0, 0.955),
      new HtEntry(70.4, 0.99),
      new HtEntry(71.4, 1.015),
      new HtEntry(72.2, 1.04),
      new HtEntry(72.9, 1.06),
      new HtEntry(74.1, 1.09),
      new HtEntry(75.6, 1.14),
      new HtEntry(76.1, 1.16),
      new HtEntry(77.1, 1.19),
      new HtEntry(78.1, 1.22),
      new HtEntry(79.6, 1.265),
      new HtEntry(80.3, 1.285),
      new HtEntry(81.2, 1.315),
      new HtEntry(82.2, 1.355),
      new HtEntry(83.3, 1.385),
      new HtEntry(84.3, 1.42),
      new HtEntry(85.4, 1.46),
      new HtEntry(86.4, 1.485),
      new HtEntry(87.0, 1.50)
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
