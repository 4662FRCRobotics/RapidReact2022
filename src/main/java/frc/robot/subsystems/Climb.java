// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private CANSparkMax m_ClimbMotorFwd;
  private CANSparkMax m_ClimbMotorInv;

  private SparkMaxLimitSwitch m_fwdFwdLimit;
  private SparkMaxLimitSwitch m_fwdRevLimit;
  private SparkMaxLimitSwitch m_invFwdLimit;
  private SparkMaxLimitSwitch m_invRevLimit;

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

    m_fwdFwdLimit = m_ClimbMotorFwd.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_fwdRevLimit = m_ClimbMotorFwd.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_invFwdLimit = m_ClimbMotorFwd.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_invRevLimit = m_ClimbMotorFwd.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    m_fwdFwdLimit.enableLimitSwitch(true);
    m_fwdRevLimit.enableLimitSwitch(true);
    m_invFwdLimit.enableLimitSwitch(true);
    m_invRevLimit.enableLimitSwitch(true);
    // m_ClimbMotorInv.follow(m_ClimbMotorFwd);

    m_ClimbMotorFwd.setInverted(false);
    m_ClimbMotorInv.setInverted(true);
    // flashes same color as master but reverses output as advertised

  }

  private void setClimbMotor(double climbSpeed) {
    SmartDashboard.putNumber("Climb Speed", climbSpeed);
    m_ClimbMotorFwd.set(climbSpeed);
    m_ClimbMotorInv.set(climbSpeed);
    // Display the speed of the Climb motors on SmartDashboard.
  }

  public void climbUp() {
    setClimbMotor(ClimbConstants.kCLIMB_UP_SPEED);
  }

  public void climbDown() {
    setClimbMotor(ClimbConstants.kCLIMB_DEPLOY_SPEED);
  }

  public void climbStop() {
    setClimbMotor(ClimbConstants.kCLIMB_STOP);
  }

  public boolean isClimbUp() {
    return m_invRevLimit.isPressed() || m_fwdRevLimit.isPressed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("invt limit", m_invFwdLimit.isPressed());
    SmartDashboard.putBoolean("fwd limit", m_fwdFwdLimit.isPressed());
  }
}
