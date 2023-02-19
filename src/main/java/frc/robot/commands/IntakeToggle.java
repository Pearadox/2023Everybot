// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeToggle extends CommandBase {
  /** Creates a new IntakeToggle. */

  private Intake _intake;
  private String intakeType;

  public IntakeToggle(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies
    _intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intakeType == "cone"){
  
  }else if (intakeType == "cube"){

  }else{
    _intake.stop();
  }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
