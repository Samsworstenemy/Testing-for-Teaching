// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
    public static final int kDriverControllerPort = 0;
  }
  public static class PIDConstants {
    // :> Not real numbers should be changed later
    // :> Update I think these are the correct ones but uh I have no way of truly verifying that.
    /* Yeah it's either 1 or .17, you know really just fiddle around with it till it works.
     */ 
    public static double pGain = 1;
    public static double iGain = 0;
    public static double dGain = 0;
  }
  public static class DriveConstants {

    // :> Motor IDs
    public static final int frontLeftMotorid = 4;
    public static final int frontRightMotorid = 5;
    public static final int backLeftMotorid = 3;
    public static final int backRightMotorid = 2;
    
    //:> Speed Multiplier
    public static final double DriveMultiplier = .5;
    public static final double RotationMultiplier = .2;

    // :> Wheel Radius in meters
    public static final double wheelDiameter = 0.1524;
    
    // :> Mecanum wheel postions in meters.
    public static final Translation2d frontLeftMeters = new Translation2d(0.257175,0.254);
    public static final Translation2d frontRightMeters = new Translation2d(0.257175,-0.254);
    public static final Translation2d backLeftMeters = new Translation2d(-0.257175, 0.254);
    public static final Translation2d backRightMeters = new Translation2d(-0.257175,-0.254);

    // :> Motor conversion ratio stuff ------------------------------
      /**  :> maximum RPM of the drivetrain NEOs (also the conversion factor from joystick input to RPM) */
      public static final double maxNEORPM = 5500.0;
      // :> Conversion ratios for drivetrain encoders
        /** :> (velocity conversion) converts from RPM to meters per second, including gearboxes*/
      public static final double velocityConversionRatio = ((wheelDiameter * Math.PI)/(8.15 * 60));
      /**  maximum speed of robot in m/s (max rpm times conversion ratio), this also (I think) converts from RPM to m/s */
      public static final double maxWheelSpeed = maxNEORPM * velocityConversionRatio;
  }
}
