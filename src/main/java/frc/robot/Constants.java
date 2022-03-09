// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Common {
        public static final int kPCM_PORT = 1;
    }

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 2;
        public static final int kLeftMotor2Port = 3;
        public static final int kRightMotor1Port = 4;
        public static final int kRightMotor2Port = 5;
        public static final boolean kIS_DRIVE_INVERTED = true;

        public static final double kRAMP_RATE = 1.0;
        public static final int kCURRENT_LIMT = 40;

        public static final double kTIRE_SIZE_IN = 6.0;
        public static final double kTIRE_SIZE_M = Units.inchesToMeters(kTIRE_SIZE_IN);
        public static final int kPULSE_PER_ROTATION = 1;

        public static final double kLOW_GEAR_REDUCTION = (42.0 / 12.0) * (60.0 / 14.0);
        public static final double kHIGH_GEAR_REDUCTION = (42.0 / 12.0) * (50.0 / 24.0);
        public static final double kENCODER_DISTANCE_PER_PULSE_M_HIGH = ((double) kPULSE_PER_ROTATION / kHIGH_GEAR_REDUCTION);
        public static final double kENCODER_DISTANCE_PER_PULSE_M_LOW = ((double) kPULSE_PER_ROTATION   / kLOW_GEAR_REDUCTION);
        // * (kTIRE_SIZE_M * Math.PI);
        public static final double kTRACK_WIDTH_M = 0.64;

        // public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new
        // DifferentialDriveKinematics(kTRACK_WIDTH_M);

        // shifter numatic ports
        public static final int kSHIFT_UP = 1;
        public static final int kSHIFT_DOWN = 0;

        public static final double kDRIVE_P = 0.3;
        public static final double kDRIVE_I = 0.0;
        public static final double kDRIVE_D = 0.0;
        public static final double kDRIVE_TOLERANCE = 2;
        public static final double kDRIVE_PID_LIMIT = 0.75;
        public static final double kTURN_PID_LIMIT = 0.6;
        public static final double kTURN_ANGLE_P = 0.33;
        public static final double kTURN_ANGLE_I = 0.0;
        public static final double kTURN_ANGLE_D = 0.41;
        public static final double kTURN_ANGLE_TOLERANCE = 1;
        public static final double kTURN_ANGLE_TOLERANCE_DEG_PER_S = 10;
        /*
         * public static final double kKEEP_HEADING_P = 0.2;
         * public static final double kKEEP_HEADING_I = 0.0;
         * public static final double kKEEP_HEADING_D = 0.4;
         * public static final double kKEEP_HEADING_TOLERANCE = 1;
         */
    }

    public static final class ShooterConstants {
        public static final int kSHOOTER_MOTOR0_PORT = 9;
        public static final int kSHOOTER_MOTOR1_PORT = 10;

        public static final int kSHOOTER_LIMIT_AMPS = 40;
        public static final double kSHOOTER_RAMP_SEC = 0.3;

        public static final double kSHOOTER_MAX_VOLTS = 10;
        public static final double kSHOOTER_MIN_VOLTS = 5;
        public static final double kSHOOTER_RANGE = kSHOOTER_MAX_VOLTS - kSHOOTER_MIN_VOLTS;
        public static final double kSHOOTER_SPEED_AUTO = .8;
        // check the Launcher Wheel Variables spreadsheet to see what these numbers do
        // low offset is for formula 1
        // public static final double kSHOOTER_LOW_OFFSET = 0.90;
        // constants for formula 2
        // public static final double kUPPER_WHEEL_K2 = 14;
        // public static final double kUPPER_WHEEL_K1 = 6;
        // contstans for formula 3
        // public static final double kUPPER_WHEEL_K3 = 3;
        // public static final double kUPPER_WHEEL_K4 = 1;

        public static final int kSHOOTER_DIRECTION = 1;
    }

    public static final class VisionConstants {
        public static final int VISION_SAMPLE_COUNT = 5;
    }

    public static final class IntakeConstants {
        public static final int kBELT_MOTOR_PORT = 11;
        public static final double kBELT_MOTOR_SPEED = -0.85; // was 3 9/26/2021
        public static final int kINTAKE_DOWN = 3;
        public static final int kINTAKE_UP = 2;
    }
    
    public static final class ButtonMappings {

        // joystick buttons
        public static final int kSHOOTER = 1;
        public static final int kLOADER = 2;
        public static final int kRETRACT_INTAKE = 3;
        public static final int kOVERRIDEGEARHIGH = 6;
        public static final int kLOADER_SPIT = 5;
        public static final int kOVERRIDEGEARLOW = 4;
        public static final int kVISION_DOWN = 7;
        public static final int kVISION_ON = 8;
        public static final int kSHOOTER_ALTERNATE = 9;
        public static final int kHARVESTER_REVERSE = 10;

        // console buttons
        public static final int kCLIMB_SWITCH = 1;
    }

    public static final class ClimbConstants {
        public static final int kCLIMBER_FWD_PORT = 7;
        public static final int kCLIMBER_INV_PORT = 12;
        public static final int kCLIMBER_BRAKE_PORT = 1;
        public static final double kCLIMB_SPEED = 0.7;
        public static final double kCLIMB_UP_DIRECTION = 1;
        public static final double kCLIMB_UP_SPEED = kCLIMB_SPEED * kCLIMB_UP_DIRECTION;
        public static final double kCLIMB_DEPLOY_SPEED = kCLIMB_SPEED * -kCLIMB_UP_DIRECTION;
        public static final double kCLIMB_STOP = 0;
        public static final double kCLIMB_BRAKE_CLOSE_ANGLE = 1;
        public static final double kCLIMB_BRAKE_OPEN_ANGLE = 0;
        public static final double kRAMP_RATE = 1.0;
    }
    public static final class HopperConstants {
        public static final int kHOPPER_MOTOR_PORT = 8;
        public static final int kHOPPER_MOTOR2_PORT = 13;
        //public static final int kSHOOTER_SENSOR_PORT = 0
        //public static final int kINTAKE_SENSOR_PORT = 1;
        public static final double kHOPPER_SPEED = 0.75;
        public static final double kHOPPER_INTAKE_SPEED = 0.80;
        public static final double kHOPPER_LAUNCH_SPEED = 0.80;
        public static final double kHOPPER_ZERO_SPEED = 0;
        public static final double kDISTANCE_THRESHOLD = 10.0;
        

        //^ Numbers above are placeholders, not final by any means
    }
}
