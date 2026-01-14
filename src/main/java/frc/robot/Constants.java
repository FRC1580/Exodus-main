// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
  public static final int kCommandControllerPort = 1;
  public static final double DEADBAND = 0.1;
  public static final double GROUND = 0;
  public static final double elevatorGearRatio = 20;
  public static final double FIRST_LEVEL = 0.4;
  public static final double SECOND_LEVEL = 0.885;
  public static final double THIRD_LEVEL = 1.734;
  public static final double FOURTH_LEVEL = 2.9;
  public static final double constPower = 0.04;
  public static final  double maximumSpeed = Units.feetToMeters(16);
  public static double speedLimiter = 1;
  public static final double intakeAngle = 0;//change this -> nah uh you stupid nigga:)
  public static final double slowerspeed = 0.4;
  public static double controlspeed = 0.4;
  public static double presicionSpeed = 1;
  public static final double precisionSpeedFinalValue = 0.4;
  public static boolean correctHeight = false;
}
 public static class ElevatorConstants
  {
    /*public static final double kElevatorKp = 5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;

    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.762; // volts (V)
    public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg


    public static final double kRotaionToMeters = kElevatorDrumRadius * 2 * Math.PI;
    public static final double kRPMtoMPS = (kElevatorDrumRadius * 2 * Math.PI) / 60;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 10.25;

    public static final double kElevatorMaxVelocity = 3.5;
    public static final double kElevatorMaxAcceleration = 2.5;*/
  }
}