// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants 
{
  public static final class DriveConstants 
  {
    public static final double ksVolts = 0.929;
    public static final double kvVoltSecondsPerMeter = 6.33;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

    public static final double kPDriveVel = 0.085;

    public static final double kTrackwidthMeters = 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics =
        
    new DifferentialDriveKinematics(kTrackwidthMeters);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  
  }

  public static final class ArmConstants
  {
    public static double kBottom = 0.0;
    public static double kMiddle = 0.25;
    public static double kUpper  = 0.50;
  }

  //Constants for Toshiba TB6612FNG Motor Controller 
  public static final class mControl
  {
    public enum STATE 
    {
      BRAKE, CCW, CW, COAST
    };

    //Control 
    public static class Control
    {
      public Boolean in1;   //Map to DIO 0
      public Boolean in2;   //Map to DIO 1

      //Structured Pair for the IN1 and IN2 Digital Control Values
      public Control(Boolean i1, Boolean i2)
      {
        in1 = i1;
        in2 = i2;
      }
    }

    public static double minSpeed = -1.0;     //Minumum Speed
    public static double maxSpeed = 1.0;      //Maximum Speed
    public static STATE forward = STATE.CW;   //Forward Intake State
    public static STATE reverse = STATE.CCW;  //Reverse Expell State
    public static int intakePWM = 2;
    public static int intakeIN1Port = 3;
    public static int intakeIN2Port = 4;
    

    public static Control BRAKE = new Control(true, true);  //Dynamic Braking
    public static Control CCW   = new Control(false, true);  //Motor Rotates Counter Clockwise
    public static Control CW    = new Control(true, false);  //Motor Rotates Clockwise
    public static Control COAST = new Control(false, false);  //Motor Coasts to a Stop

  }
}
