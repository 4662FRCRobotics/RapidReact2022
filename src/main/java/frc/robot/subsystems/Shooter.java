/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   * okay
   */

  private WPI_TalonSRX m_shooterMotor0;
  private WPI_TalonSRX m_shooterMotor1;
  
  private boolean m_bIsMotorOn;

  private double m_voltUpper;
  private double m_voltLower;

  public Shooter() {
    m_shooterMotor0 = new WPI_TalonSRX(ShooterConstants.kSHOOTER_MOTOR0_PORT);
    m_shooterMotor1 = new WPI_TalonSRX(ShooterConstants.kSHOOTER_MOTOR1_PORT);

    m_shooterMotor0.configFactoryDefault();
    m_shooterMotor1.configFactoryDefault();
    m_shooterMotor0.setNeutralMode(NeutralMode.Coast);
    m_shooterMotor1.setNeutralMode(NeutralMode.Coast);
    m_shooterMotor0.configContinuousCurrentLimit(ShooterConstants.kSHOOTER_LIMIT_AMPS);
    m_shooterMotor1.configContinuousCurrentLimit(ShooterConstants.kSHOOTER_LIMIT_AMPS);
    m_shooterMotor0.configOpenloopRamp(ShooterConstants.kSHOOTER_RAMP_SEC);
    m_shooterMotor1.configOpenloopRamp(ShooterConstants.kSHOOTER_RAMP_SEC);

    m_bIsMotorOn = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LowVolts", m_voltLower);
    SmartDashboard.putNumber("UpVolts", m_voltUpper);
  }  
  
  public void setMotor(double voltLower, double voltUpper){

    m_shooterMotor0.setVoltage(voltLower * ShooterConstants.kSHOOTER_DIRECTION);
    m_shooterMotor1.setVoltage(voltUpper * -ShooterConstants.kSHOOTER_DIRECTION);
  }

  public void setVoltage(DoubleSupplier throttle, DoubleSupplier upperOffset) {
    //double adjustedThrottle = 1 - ((-throttle.getAsDouble() + 1) * 0.25);
    double adjustedThrottle = ((throttle.getAsDouble() + 1) / 2) * ShooterConstants.kSHOOTER_RANGE;
    SmartDashboard.putNumber("Shooter Thottle", adjustedThrottle);
    m_voltLower = ShooterConstants.kSHOOTER_MIN_VOLTS + adjustedThrottle;
    //double voltUpper = ShooterConstants.kSHOOTER_MAX_VOLTS * Math.pow(adjustedThrottle * ShooterConstants.kSHOOTER_LOW_OFFSET,2);
    //m_voltUpper = ShooterConstants.kUPPER_WHEEL_K2 - (ShooterConstants.kUPPER_WHEEL_K1 / adjustedThrottle);
    m_voltUpper = m_voltLower * ((upperOffset.getAsDouble() + 1) / 2);
    //double voltUpper = voltLower + (ShooterConstants.kUPPER_WHEEL_K4 - (ShooterConstants.kUPPER_WHEEL_K3 / adjustedThrottle));
  }

  public void setMotorOn(DoubleSupplier throttle, DoubleSupplier upperOffset){
    setVoltage(throttle, upperOffset);
    setMotor(m_voltLower, m_voltUpper);
    m_bIsMotorOn = true;
    //SmartDashboard.putBoolean("Shooter Motor", m_bIsMotorOn);
  }

  public void setMotorOff(){
    m_shooterMotor0.stopMotor();
    m_shooterMotor1.stopMotor();
    m_bIsMotorOn = false;
    //SmartDashboard.putBoolean("Shooter Motor", m_bIsMotorOn);
  }
}