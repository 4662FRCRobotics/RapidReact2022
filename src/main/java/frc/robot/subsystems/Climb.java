// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private CANSparkMax m_ClimbMotorFwd;
  private CANSparkMax m_ClimbMotorInv;
  private Servo m_climbBrake;

  private boolean m_bIsClimbBrakeSet;

  public Climb() {
    m_bIsClimbBrakeSet = true;

    m_ClimbMotorFwd = new CANSparkMax(ClimbConstants.kCLIMBER_FWD_PORT, MotorType.kBrushless);
    m_ClimbMotorInv = new CANSparkMax(ClimbConstants.kCLIMBER_INV_PORT, MotorType.kBrushless);

    m_ClimbMotorFwd.restoreFactoryDefaults();
    m_ClimbMotorInv.restoreFactoryDefaults();

    m_ClimbMotorFwd.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_ClimbMotorInv.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_ClimbMotorFwd.setOpenLoopRampRate(ClimbConstants.kRAMP_RATE);
    m_ClimbMotorInv.setOpenLoopRampRate(ClimbConstants.kRAMP_RATE);

    // m_ClimbMotorFwd.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    // LimitSwitchNormal.NormallyOpen, 0);
    // m_ClimbMotorFwd.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    // LimitSwitchNormal.NormallyOpen, 0);

    m_ClimbMotorInv.follow(m_ClimbMotorFwd);

    m_ClimbMotorFwd.setInverted(false);
    m_ClimbMotorInv.setInverted(true);
    // flashes same color as master but reverses output as advertised
    m_climbBrake = new Servo(ClimbConstants.kCLIMBER_BRAKE_PORT);
  }

  private void setClimbMotor(double climbSpeed) {
    SmartDashboard.putNumber("Climb Speed", climbSpeed);
    m_ClimbMotorFwd.set(climbSpeed);
    // Display the speed of the Climb motors on SmartDashboard.
  }
  
  public void climbUp(){
    setClimbMotor(ClimbConstants.kCLIMB_UP_SPEED);
  }

  public void climbDown(){
    setClimbMotor(ClimbConstants.kCLIMB_DEPLOY_SPEED);
  }

  public void climbStop(){
    setClimbMotor(ClimbConstants.kCLIMB_STOP);
  }

  public void climbBrakeSet(){
    m_climbBrake.setPosition(ClimbConstants.kCLIMB_BRAKE_CLOSE_ANGLE);
  }

  public void climbBrakeRelease(){
    m_climbBrake.setPosition(ClimbConstants.kCLIMB_BRAKE_OPEN_ANGLE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
