// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;


public class Hopper extends SubsystemBase {
  private CANSparkMax m_beltmotor;
  private CANSparkMax m_wheelmotor;


  
  public Hopper() {
    m_beltmotor = new CANSparkMax(HopperConstants.kHOPPER_MOTOR_PORT, MotorType.kBrushless);
    m_beltmotor.restoreFactoryDefaults();
    m_beltmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
    m_wheelmotor = new CANSparkMax(HopperConstants.kHOPPER_MOTOR2_PORT, MotorType.kBrushless);
    m_wheelmotor.restoreFactoryDefaults();
    m_wheelmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}   
 
