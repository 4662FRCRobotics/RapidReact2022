// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class OverRideArcadeDrive extends CommandBase {
  /** Creates a new OverRideArcadeDrive. */
  private final Drive m_drive;
  private final DoubleSupplier m_velocity;
  private final DoubleSupplier m_heading;  
  private final DoubleSupplier m_turnRate;
  private final Boolean m_highGear;



  public OverRideArcadeDrive(Drive subsystem, Boolean highGear, DoubleSupplier velocity, DoubleSupplier heading, DoubleSupplier turnRate) {
    m_drive = subsystem;
    m_velocity = velocity;
    m_heading = heading;
    m_turnRate = turnRate;
    m_highGear = highGear;
   addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_highGear) {
      m_drive.setShiftHigh();
    } else {
      m_drive.setShiftLow();
    }
    SmartDashboard.putBoolean("InHighGear", m_highGear);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.overRideThrottledArcadeDrive(m_velocity.getAsDouble(), m_heading.getAsDouble(), m_turnRate.getAsDouble());

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
