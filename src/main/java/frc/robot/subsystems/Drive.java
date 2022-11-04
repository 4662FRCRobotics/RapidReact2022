// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.Common;
import frc.robot.Constants.DriveConstants;
import frc.robot.libraries.ConsoleJoystick;

public class Drive extends SubsystemBase {
  
  private ConsoleJoystick m_console;
  private CANSparkMax m_leftController1;
  private CANSparkMax m_leftController2;
  private CANSparkMax m_rightController1;
  private CANSparkMax m_rightController2;
  // private MotorControllerGroup m_leftControllerGroup;
  // private MotorControllerGroup m_rightControllerGroup;
  private DifferentialDrive m_differentialdrive;
  private RelativeEncoder m_leftEncoder1;
  private RelativeEncoder m_rightEncoder1;
  private AHRS m_gyroK;
  public PIDController m_drivePIDController;
  private double m_dDriveDistanceP;
  private double m_dDriveDistanceI;
  private double m_dDriveDistanceD; 
  private double m_turnP;
  private double m_turnI;
  private double m_turnD;

  private final TrapezoidProfile.Constraints m_constraints;
private final Encoder m_encoder = new Encoder(1, 2);
private final ProfiledPIDController m_controller;
private final Joystick m_joystick = new Joystick(1);
private boolean newPidTest;

  private double m_dDriveDistanceTolerance;
  private double m_dDistance;
  private double m_leftEncoderSign;
  private double m_rightEncoderSign;
  private double m_currentEncoderDistancePerPulse;
  private double m_headingSign;
  private boolean m_bInHighGear;
  private DoubleSolenoid m_gearShifter;

  public PIDController m_turnPIDController;
  private double m_dAngle;
  private double m_dStartAngle;

  private Value m_currShift;

  public Drive() {
   
  SmartDashboard.getBoolean("pid test", newPidTest);
    m_leftController1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    m_leftController2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    m_rightController1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    m_rightController2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

    m_rightController1.restoreFactoryDefaults();
    m_rightController2.restoreFactoryDefaults();
    m_leftController1.restoreFactoryDefaults();
    m_leftController2.restoreFactoryDefaults();

    m_leftController1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftController2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightController1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightController2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_leftController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_leftController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController1.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);
    m_rightController2.setOpenLoopRampRate(DriveConstants.kRAMP_RATE);

    m_leftController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_leftController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_rightController1.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);
    m_rightController2.setSmartCurrentLimit(DriveConstants.kCURRENT_LIMT);

    m_leftEncoder1 = m_leftController1.getEncoder();
    m_rightEncoder1 = m_rightController1.getEncoder();
    m_currentEncoderDistancePerPulse = DriveConstants.kENCODER_DISTANCE_PER_PULSE_M_LOW;
    /*
     * m_leftControllerGroup= new SpeedControllerGroup(m_leftController1,
     * m_leftController2);
     * m_rightControllerGroup= new SpeedControllerGroup(m_rightController1,
     * m_rightController2);
     * m_leftControllerGroup.setInverted(false);
     * m_rightControllerGroup.setInverted(false);
     * m_differentialdrive= new DifferentialDrive(m_leftControllerGroup,
     * m_rightControllerGroup);
     */

    m_leftController2.follow(m_leftController1);
    m_rightController2.follow(m_rightController1);
    m_differentialdrive = new DifferentialDrive(m_leftController1, m_rightController1);
    m_rightController1.setInverted(!DriveConstants.kIS_DRIVE_INVERTED);
    m_leftController1.setInverted(DriveConstants.kIS_DRIVE_INVERTED);

    if (DriveConstants.kIS_DRIVE_INVERTED) {
      m_leftEncoderSign = 1;
      m_rightEncoderSign = -1;
      m_headingSign = 1;
    } else {
      m_leftEncoderSign = -1;
      m_rightEncoderSign = 1;
      m_headingSign = -1;
    }
    m_bInHighGear = false;
  
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);

    m_dDriveDistanceP = DriveConstants.kDRIVE_P;
    m_dDriveDistanceI = DriveConstants.kDRIVE_I;
    m_dDriveDistanceD = DriveConstants.kDRIVE_D;
    m_dDriveDistanceTolerance = DriveConstants.kDRIVE_TOLERANCE;
    m_dDistance = getDashboardDistance();
    m_drivePIDController = new PIDController(m_dDriveDistanceP, m_dDriveDistanceI, m_dDriveDistanceD);
    m_drivePIDController.setTolerance(m_dDriveDistanceTolerance);

    m_constraints =
    new TrapezoidProfile.Constraints(1.75, 0.75);
    m_controller =
    new ProfiledPIDController(m_dDriveDistanceP, m_dDriveDistanceI, m_dDriveDistanceD, m_constraints);

    m_turnP = Constants.DriveConstants.kTURN_ANGLE_P;
    m_turnI = Constants.DriveConstants.kTURN_ANGLE_I;
    m_turnD = Constants.DriveConstants.kTURN_ANGLE_D;

    m_gyroK = new AHRS(SerialPort.Port.kMXP);
    m_turnPIDController = new PIDController(m_turnP, m_turnI, m_turnD);

    m_turnPIDController.setTolerance(DriveConstants.kTURN_ANGLE_TOLERANCE,
        DriveConstants.kTURN_ANGLE_TOLERANCE_DEG_PER_S);
    m_dAngle = 0;

    getDashboardTurn();

    m_gearShifter = new DoubleSolenoid(Common.kPCM_PORT, PneumaticsModuleType.CTREPCM, DriveConstants.kSHIFT_DOWN,
        DriveConstants.kSHIFT_UP);
    m_currShift = m_gearShifter.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("LeftEncdr:", getLeftDistance());
    SmartDashboard.putNumber("RightEncdr:", getRightDistance());
    SmartDashboard.putNumber("GyroYaw", getYaw());
    SmartDashboard.putNumber("Distance", getDistance());
  }

  public void arcadeDrive(double velocity, double heading) {
    m_differentialdrive.arcadeDrive(velocity, -1 * heading);
    // System.out.println("velocity="+velocity);
    // System.out.println("heading="+heading);
  }

  public void throttledArcadeDrive(double velocity, double heading, double throttle, double turnRate) {
    throttle = 1;
    if (throttle < 0) {
      m_bInHighGear = true;
      velocity = velocity * ((1.0 - throttle) / 2.0);
      //heading = heading * (-(1.0 / (throttle - 1.0)));

    } else {
      m_bInHighGear = false;
      velocity = velocity * (1.0 / (throttle + 1.0));
    }
    heading = heading * (1- ((turnRate + 1) * 0.25));
    SmartDashboard.putBoolean("InHighGear", m_bInHighGear);
    arcadeDrive(velocity, heading);
  }

  public void overRideThrottledArcadeDrive(double velocity, double heading, double turnRate) {
    heading = heading * (1- ((turnRate + 1) * 0.25));
    arcadeDrive(velocity, heading);
  }

  public void setShiftHigh() {
    m_currShift = m_gearShifter.get();
    m_gearShifter.set(Value.kForward);
    m_currentEncoderDistancePerPulse = DriveConstants.kENCODER_DISTANCE_PER_PULSE_M_HIGH;

  }

  public void setShiftLow() {
    m_currShift = m_gearShifter.get();
    m_gearShifter.set(Value.kReverse);
    m_currentEncoderDistancePerPulse = DriveConstants.kENCODER_DISTANCE_PER_PULSE_M_LOW;
  }

  public void resetShiftPrev() {
    m_gearShifter.set(m_currShift);
  }
  
  public boolean isHighGear() {
    return m_bInHighGear;
  }

  public double getDistance() {
    return (m_leftEncoder1.getPosition() + m_rightEncoder1.getPosition()) / 2;
    // return -(getLeftDistance() - getRightDistance());
  }

  private double getLeftDistance() {
    return m_leftEncoderSign * m_leftEncoder1.getPosition() * m_currentEncoderDistancePerPulse;
  }

  private double getRightDistance() {
    return m_rightEncoderSign * m_rightEncoder1.getPosition() * m_currentEncoderDistancePerPulse;
  }

  public double getYaw() {
    return m_gyroK.getYaw();
  }

  public double getAngleK() {

    return m_gyroK.getAngle();
  }

  public double getDashboardDistance() {

    m_dDistance = SmartDashboard.getNumber("DriveDistance", m_dDistance);
    m_dDriveDistanceP = SmartDashboard.getNumber("DriveDistanceP", m_dDriveDistanceP);
    m_dDriveDistanceI = SmartDashboard.getNumber("DriveDistanceI", m_dDriveDistanceI);
    m_dDriveDistanceD = SmartDashboard.getNumber("DriveDistanceD", m_dDriveDistanceD);
    m_dDriveDistanceTolerance = SmartDashboard.getNumber("DriveDistanceTolerance", m_dDriveDistanceTolerance);
    m_dDistance = SmartDashboard.getNumber("DriveDistance", 2);
    SmartDashboard.putNumber("DriveDistance", m_dDistance);
    SmartDashboard.putNumber("DriveDistanceP", m_dDriveDistanceP);
    SmartDashboard.putNumber("DriveDistanceI", m_dDriveDistanceI);
    SmartDashboard.putNumber("DriveDistanceD", m_dDriveDistanceD);
    SmartDashboard.putNumber("DriveDistanceTolerance", m_dDriveDistanceTolerance);
    SmartDashboard.putNumber("DriveDistance", m_dDistance);

    return -m_dDistance;
  }

  public void resetEncoders() {
    // m_gyroAndCollison.reset();
    m_leftEncoder1.setPosition(0);
    m_rightEncoder1.setPosition(0);
  }

  public void resetPIDDriveController() {
    m_drivePIDController.setPID(m_dDriveDistanceP, m_dDriveDistanceI, m_dDriveDistanceD);
    m_drivePIDController.setTolerance(m_dDriveDistanceTolerance);
  }

  public void initDriveController(double distance) {
    if (m_bInHighGear) {
      double encoderDistance = distance / DriveConstants.kENCODER_DISTANCE_PER_PULSE_M_HIGH;
      m_drivePIDController.setSetpoint(encoderDistance);
      resetEncoders();
      m_drivePIDController.reset();
      SmartDashboard.putNumber("EncoderDistance", encoderDistance);
    }else{
      double encoderDistance = distance / DriveConstants.kENCODER_DISTANCE_PER_PULSE_M_LOW;
    m_drivePIDController.setSetpoint(encoderDistance);
    resetEncoders();
    m_drivePIDController.reset();
    SmartDashboard.putNumber("EncoderDistance", encoderDistance);
    }
  }

  public void execDriveController() {
    double speed = MathUtil.clamp(m_drivePIDController.calculate(getDistance()), -DriveConstants.kDRIVE_PID_LIMIT,
        DriveConstants.kDRIVE_PID_LIMIT);
    arcadeDrive(speed, 0);
    // System.out.println("speed="+speed);
    // SmartDashboard.putNumber("Speed", speed);
  }

  public void endDriveController() {
    arcadeDrive(0.0, 0.0);
  }

  public boolean isDriveAtSetpoint() {
    return m_drivePIDController.atSetpoint();
  }

  public void resetProfiledController() {
    m_controller.setPID(m_dDriveDistanceP, m_dDriveDistanceI, m_dDriveDistanceD);
    m_controller.setTolerance(m_dDriveDistanceTolerance);
  }

  /*public void initProfiledController(double distance) {
    if (m_bInHighGear) {
      double encoderDistance = distance / DriveConstants.kENCODER_DISTANCE_PER_PULSE_M_HIGH;
      m_controller.setSetpoint(encoderDistance);
      resetEncoders();
      m_controller.reset();
      SmartDashboard.putNumber("EncoderDistance", encoderDistance);
    }else{
      double encoderDistance = distance / DriveConstants.kENCODER_DISTANCE_PER_PULSE_M_LOW;
    m_controller.setSetpoint(encoderDistance);
    resetEncoders();
    m_controller.reset();
    SmartDashboard.putNumber("EncoderDistance", encoderDistance);
    }
  } */


  public double getDashboardTurn() {

    m_turnP = SmartDashboard.getNumber("turnP", m_turnP);
    m_turnI = SmartDashboard.getNumber("turnI", m_turnI);
    m_turnD = SmartDashboard.getNumber("turnD", m_turnD);
    // m_dDriveDistanceTolerance =
    // SmartDashboard.getNumber("DriveDistanceTolerance",
    // m_dDriveDistanceTolerance);
    m_dAngle = SmartDashboard.getNumber("turn angle", m_dAngle);
    SmartDashboard.putNumber("turnP", m_turnP);
    SmartDashboard.putNumber("turnI", m_turnI);
    SmartDashboard.putNumber("turnD", m_turnD);
    // SmartDashboard.putNumber("DriveDistanceTolerance",
    // m_dDriveDistanceTolerance);
    SmartDashboard.putNumber("turn angle", m_dAngle);
    m_turnPIDController.setPID(m_turnP, m_turnI, m_turnD);

    return m_dAngle;
  }

  public void initTurnController(DoubleSupplier targetAngle) {
    m_dStartAngle = getAngleK();
    m_turnPIDController.setSetpoint(targetAngle.getAsDouble() + m_dStartAngle);
    m_turnPIDController.reset();
  }

  public void execTurnController() {
    // System.out.println("Drive exec turn controller");
    // System.out.println(MathUtil.clamp(m_turnPIDController.calculate(getAngleK()),
    // -1, 1));
    arcadeDrive(0, MathUtil.clamp(m_turnPIDController.calculate(getAngleK()), -DriveConstants.kTURN_PID_LIMIT,
        DriveConstants.kTURN_PID_LIMIT));
  }

  public void endTurnController() {
    arcadeDrive(0, 0);
  }

  public boolean isTurnAtSetpoint() {
    return m_turnPIDController.atSetpoint();
  }

}
