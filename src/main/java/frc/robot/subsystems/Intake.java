// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Common;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private DoubleSolenoid m_IntakeDeploy;
  private CANSparkMax m_IntakeMotor;

  
  public Intake() {
    m_IntakeDeploy = new DoubleSolenoid(Common.kPCM_PORT, PneumaticsModuleType.CTREPCM, IntakeConstants.kINTAKE_DOWN, IntakeConstants.kINTAKE_UP);
   m_IntakeMotor = new CANSparkMax(IntakeConstants. kBELT_MOTOR_PORT, MotorType.kBrushless);

  }

  public void ArmUp() {
    m_IntakeDeploy.set(Value.kReverse);
  }

  public void ArmDown() {
    m_IntakeDeploy.set(Value.kForward);
  }

  public void IntakeOn(){
    m_IntakeMotor.set(IntakeConstants.kBELT_MOTOR_SPEED);
    //SmartDashboard.putBoolean("Feeder Belt", true);
  }

  public void beltRev() {
    m_IntakeMotor.set(-IntakeConstants.kBELT_MOTOR_SPEED);
  }

  public void beltOff(){
    m_IntakeMotor.stopMotor();
    //SmartDashboard.putBoolean("Feeder Belt", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
