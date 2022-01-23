// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
public class Drive extends SubsystemBase {
  WPI_TalonSRX wrist = new WPI_TalonSRX(13);
  PigeonIMU gyro = new PigeonIMU(wrist);
    
  WPI_TalonSRX leftFront = new WPI_TalonSRX(22);
  WPI_TalonSRX rightRear = new WPI_TalonSRX(11);

  WPI_VictorSPX rightFront = new WPI_VictorSPX(18);
  WPI_VictorSPX leftRear = new WPI_VictorSPX(21);

  MotorControllerGroup leftDrive = new  MotorControllerGroup(leftRear, leftFront);
  MotorControllerGroup rightDrive = new MotorControllerGroup(rightFront, rightRear);
  DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);
  double[] yprGyro;
  PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();

  double x, y, a, distanceToTarget;

  public void move(double leftSpeed, double rightSpeed)
  {
    drive.tankDrive(leftSpeed, -rightSpeed);
  }
/** Creates a new Drive. */
  public Drive() {
    rightRear.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);
    leftFront.setSensorPhase(true);
    //rightRear.setSensorPhase(true);
    //leftFront.clearStickyFaults();
    //rightRear.configFactoryDefault();
    //leftFront.configFactoryDefault();
    //rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    //leftRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void configureEncoders(){
    rightRear.setSensorPhase(false);
    leftFront.setSensorPhase(true);
    rightRear.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);

  }
double encoderToFeet;
double rightEncoderToFeet;
double leftTraveled ;

  @Override

  public void periodic() { 
yprGyro = new double[3];
gyro.getYawPitchRoll(yprGyro);
    // This method will be called once per scheduler run
     leftTraveled = leftFront.getSelectedSensorPosition();
     SmartDashboard.putNumber("Sub Left", leftTraveled);
    encoderToFeet = leftTraveled * ((Math.PI*6)/49152);
    double rightTraveled = rightRear.getSelectedSensorPosition();
    SmartDashboard.putNumber("Sub Right", rightTraveled);
    rightEncoderToFeet = rightTraveled * ((Math.PI*6)/49152);
   //SmartDashboard.putNumberArray("gyroarray", yprGyro );
    //SmartDashboard.putNumber("gyro0", yprGyro[0]);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry dx = table.getEntry("tx");
    NetworkTableEntry dy = table.getEntry("ty");
    NetworkTableEntry da = table.getEntry("ta");

    x = dx.getDouble(0.0);
    y = dy.getDouble(0.0);
    a = da.getDouble(0.0);

    SmartDashboard.putNumber("limelight_x", x);
    SmartDashboard.putNumber("limelight_y", y);
    SmartDashboard.putNumber("limelight_a", a);

    double heightFromTarget = Constants.targetHeight - Constants.limelightHeight;
    double theta = Math.toRadians(Constants.bottomAngle + y);
    distanceToTarget = (heightFromTarget/Math.tan(theta));
    
  }
  public double leftDistance(){
    return leftTraveled;
  }

  public double targetDistance(){
    return distanceToTarget;
  }

  public double currentAngle(){
    return x;
  }
 // public double rightDistance(){
    //return rightEncoderToFeet;
 // }
 
  public double gyroYaw(){
    return yprGyro[0];
  }
  public double gyroRoll(){
    return yprGyro[1];
  }
  public double gyroPitch(){
    return yprGyro[2];
  }
}
