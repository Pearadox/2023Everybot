// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class AutoCubeOut extends CommandBase {
  /** Creates a new AutoCubeOut. */
  private Intake _intake;

  private Timer timeSinceStart = new Timer();
  private int timeToRelease = 1;

  private boolean isDone = false;

  public AutoCubeOut(Intake intake) {
    _intake = intake;
    addRequirements(_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSinceStart.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timeSinceStart.get() < timeToRelease){
      _intake.setState(IntakeState.ConeIntake);
    }else{
      _intake.setState(IntakeState.Off);
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
