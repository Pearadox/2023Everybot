// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ADIS16470_IMU _gyro = new ADIS16470_IMU();
  private final DriveTrain _driveTrain = new DriveTrain(_gyro);
  private final Joystick _controller = new Joystick(0);
  private final Joystick _controller2 = new Joystick(1);
  private Arm _arm;
  private Intake _intake;



  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    _driveTrain.setDefaultCommand(new ArcadeDrive(_driveTrain, _controller));
    _intake.setDefaultCommand(new RunCommand(_intake::stop, _intake));
    _arm.setDefaultCommand(new RunCommand(_arm::stop, _arm));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(_controller2, JoystickConstants.Y).whileTrue(new RunCommand(_arm::raise, _arm)); //Y raises arm
    new JoystickButton(_controller2, JoystickConstants.A).whileTrue(new RunCommand(_arm::lower, _arm)); // A lowers arm
    new JoystickButton(_controller2, JoystickConstants.BUMPER_LEFT).whileTrue(new RunCommand(_intake::intakeOut, _intake)); // Left bumber cone outake
    new JoystickButton(_controller2, JoystickConstants.BUMPER_RIGHT).whileTrue(new RunCommand(_intake::intakeIn, _intake));  // Right bumber cone intake
    new JoystickButton(_controller2, JoystickConstants.B).whileTrue(new RunCommand(_intake::intakeOut, _intake)); // B is cone outake
    new JoystickButton(_controller2, JoystickConstants.X).whileTrue(new RunCommand(_intake::intakeIn, _intake)); //X is cone intake
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
