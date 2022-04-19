/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   * okay
   */

  private WPI_TalonFX m_shooterMotor0;
  //private WPI_TalonFX m_shooterMotor1;
  
  //private boolean m_bIsMotorOn;

   private double m_voltLower;

  public Shooter() {
    m_shooterMotor0 = new WPI_TalonFX(ShooterConstants.kSHOOTER_MOTOR0_PORT);
    m_shooterMotor0.configFactoryDefault();
    m_shooterMotor0.setNeutralMode(NeutralMode.Coast);
    m_shooterMotor0.configOpenloopRamp(ShooterConstants.kSHOOTER_RAMP_SEC);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LowVolts", m_voltLower);
  }  
  
  public void setMotor(double voltLower){

    m_shooterMotor0.setVoltage(voltLower * ShooterConstants.kSHOOTER_DIRECTION);
  }

  public void setVoltage(DoubleSupplier throttle) {
    double adjustedThrottle = ((throttle.getAsDouble() + 1) / 2) * ShooterConstants.kSHOOTER_RANGE;
    SmartDashboard.putNumber("Shooter Thottle", adjustedThrottle);
    m_voltLower = ShooterConstants.kSHOOTER_MIN_VOLTS + adjustedThrottle;
  }

  public void setMotorOn(DoubleSupplier throttle){
    setVoltage(throttle);
    setMotor(m_voltLower);

  }

  public void setMotorOff(){
    m_shooterMotor0.stopMotor();
  }
}