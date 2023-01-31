// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

/** Add your docs here. */
public class ArmInterp {
  static class HtEntry{double height; double ticks; public HtEntry(double h, double t) {height = h; ticks = t;}}
  static final HtEntry[] htTable = {
      new HtEntry(0, 0),
      new HtEntry(60, 10000)
  };

  public static int heightIndex(double height) {
    assert(height >= 0);
    int index = Arrays.binarySearch(htTable, new HtEntry(height, 0), (a,b) -> Double.compare(a.height, b.height));
    assert(index < htTable.length);
    return index;
  }

  public static double heightToTicks(double height) {
    int index = heightIndex(height);
     if (index == 0) {
      return htTable[index].ticks;
     }
     else if (htTable[index].height == height) {
       return htTable[index].ticks;
     }
     // Linear Interpolation
     else {
      double h0 = htTable[index-1].height;
      double h1 = htTable[index].height;
      double t0 = htTable[index-1].ticks;
      double t1 = htTable[index].ticks;
      double scaler = (height-h0) / (h1 - h0);
      double ticksRange = t1 - t0;
      return t0 + scaler*ticksRange;
     }
  }
  
  public static double ticksToHeight(double ticks) {
    assert(ticks >= 0);
    int index = Arrays.binarySearch(htTable, new HtEntry(0, ticks), (a,b) -> Double.compare(a.ticks, b.ticks));

    if (index == 0) {
      return htTable[index].height;
     }
     else if (index == htTable.length) {
      return htTable[index-1].height;
     }
     else if (htTable[index].ticks == ticks) {
       return htTable[index].height;
     }
     // Linear Interpolation
     else {
      double h0 = htTable[index-1].height;
      double h1 = htTable[index].height;
      double t0 = htTable[index-1].ticks;
      double t1 = htTable[index].ticks;
      double scaler = (ticks-t0) / (t1 - t0);
      double heightRange = h1 - h0;
      return h0 + scaler*heightRange;
     }
  }

  public static double getTicksPerMeter(double height) {
    if (height == 0.0) {
      return 0.0;
    }
    int indexGE = heightIndex(height);
    assert(indexGE > 0);
    int indexLT = indexGE - 1;
    double deltaH = htTable[indexGE].height - htTable[indexLT].height;
    double deltaT = htTable[indexGE].ticks - htTable[indexLT].ticks;
    return deltaT / deltaH;
  }

  public static double vheightToTPM(double metersPerSecond, double height) {
    double slope = getTicksPerMeter(height);
    double secondsPerMinute = 60;
    return slope * metersPerSecond * secondsPerMinute;
  }
}
