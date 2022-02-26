// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ButtonMappings;
import frc.robot.commands.*;
import frc.robot.libraries.ConsoleJoystick;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Climb m_climb = new Climb();
  private final Drive m_drive = new Drive();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Vision m_vision = new Vision();
  private final Hopper m_hopper = new Hopper();
  private final Joystick m_driveStick = new Joystick(0);
  private final ConsoleJoystick m_console = new ConsoleJoystick(1);

  private final Command m_autoCommand = new AutoControl(m_console, m_hopper, m_drive, m_climb, m_intake, m_shooter,
      m_vision);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_intake.ArmUp();

    m_drive.setDefaultCommand(
        new ArcadeDrive(
            m_drive,
            () -> m_driveStick.getY(),
            () -> m_driveStick.getTwist(),
            () -> m_driveStick.getThrottle()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("AutoDistance", new AutoDriveDistance(m_drive));
    SmartDashboard.putData("AutoTurn", new AutoTurnAngle(() -> m_drive.getDashboardTurn(), m_drive));

    new JoystickButton(m_driveStick, ButtonMappings.kSHOOTER)
        .whenHeld(
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(3),
                    new FindHub(m_vision)),
                new AutoTurnAngle(() -> m_vision.getYaw(), m_drive),
                new ParallelCommandGroup(
                    new BaggageHandlerShoot(m_shooter, () -> m_console.getZ()),
                    new SequentialCommandGroup(new WaitCommand(1), new ShootHopperFeed(m_hopper)))));

    new JoystickButton(m_driveStick, ButtonMappings.kSHOOTER_ALTERNATE)
        .whileHeld(
            new ParallelCommandGroup(
                new BaggageHandlerShoot(m_shooter, () -> m_console.getZ()),
                new SequentialCommandGroup(new WaitCommand(1), new ShootHopperFeed(m_hopper))));

    new JoystickButton(m_driveStick, ButtonMappings.kCLIMB_DEPLOY)
        .whileHeld(
            new ConditionalCommand(
                new ClimbDeploy(m_climb),
                new InstantCommand(),
                () -> m_console.getRawButton(ButtonMappings.kCLIMB_SWITCH)));

    new JoystickButton(m_driveStick, ButtonMappings.kCLIMB_UP)
    .whileHeld(
        new ClimbLift(m_climb));

    new JoystickButton(m_driveStick, ButtonMappings.kLOADER)
        .whileHeld(
            new IntakeCargo(m_hopper, m_intake));

    new Trigger(m_drive::isHighGear)
        .whenActive(new InstantCommand(m_drive::setShiftHigh, m_drive))
        .whenInactive(new InstantCommand(m_drive::setShiftLow, m_drive));

    new JoystickButton(m_driveStick, ButtonMappings.kRETRACT_INTAKE).whenPressed(
        new RetractIntake(m_intake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
