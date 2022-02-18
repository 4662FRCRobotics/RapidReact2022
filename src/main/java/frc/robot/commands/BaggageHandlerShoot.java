// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class BaggageHandlerShoot extends CommandBase {
  /** Creates a new BaggageHandlerShoot. */
  private DoubleSupplier m_shootThrottle;

  private Shooter m_shooter;

  public BaggageHandlerShoot(Shooter shooter, DoubleSupplier shootThrottle) {
      Shooter m_shooter = shooter;
      m_shootThrottle = shootThrottle;


      addRequirements(m_shooter);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
