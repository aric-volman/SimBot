// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int leftDriveTalonPort = 2;
    public static final int rightDriveTalonPort = 3;
    public static final int wheelDiameterInInches = 6;
  }

  public static class SimConstants {
    public static final double kS = 0.22;
    public static final double kV = 2.98;
    public static final double kA = 0.2;

    
    public static final double kVangular = 1.0;
    public static final double kAangular = 1.0;

    public static final double kTrackwidthMeters = 0.7112;
  }
}
