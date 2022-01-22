// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class driveCommand extends CommandBase 
{

  private final Drive drive;
  private final double leftSpeed, rightSpeed;
  /** Creates a new DriveCommand. */


  public driveCommand(Drive drive2, double leftSpeed2, double rightSpeed2) 
  {
    drive = drive2;
    leftSpeed = leftSpeed2;
    rightSpeed = rightSpeed2;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drive);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(leftSpeed,rightSpeed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static void setDefaultCommand(RunCommand runCommand){
    
  }
}
