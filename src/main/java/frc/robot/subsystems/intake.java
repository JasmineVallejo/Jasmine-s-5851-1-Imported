// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  Victor intake = new Victor(5);
  /** Creates a new intake. */
  public intake() {}

  public void moveIntake(double speed){
    intake.set(speed);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
