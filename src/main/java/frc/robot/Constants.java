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
    public static double kP = 0.00001, kI = 0, kD = 0, autoMoveKP = 0.01, autoMoveKI = 0.01, 
    autoMoveKD = 0, rollerSpeed = 0.5, liftSpeed = 0.5, intakeSpeed = 0.5, indexerSpeed = 0.5, 
    
    shooterSpeed = 0.5, bottomAngle = 30, targetHeight = 6, limelightHeight = 4,
            shooterKP = 0, shooterKI = 0, shooterKD = 0;
    
    public static int angel = 0;
}
