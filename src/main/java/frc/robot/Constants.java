// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

 public final class Constants {
    public static final class MotorIDs {
        public static final int IDFrontLeftMotor = 4;
        public static final int IDBackLeftMotor = 5;
        public static final int IDFrontRightMotor = 1;
        public static final int IDBackRightMotor = 2;
    }

    public static final class DriveConstants {
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(22.48);

        // Distance between centers of front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(19.81);

        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
            );

        public static final double manualVoltageRampingConstant = 0.21;
        public static final double closedVoltageRampingConstant = 0.21;

        public static final int IDPID = 0;
        public static final double PIDPeriod = 1.0 / 20.0;
        public static final double KF = 0.0485; 
        public static final double KP = 0.001;
        public static final double KI = 0.0; 
        public static final double KD = 0.001;

        public static final double GearRatio = 10.71 ;
        public static final double CountsPerWheelRevolution = 2048.0 * GearRatio;
        public static final double WheelDiameter = Units.inchesToMeters(6.0);
        public static final double MetersPerRevolution = WheelDiameter * Math.PI ;
        public static final double MetersPerCount = MetersPerRevolution / CountsPerWheelRevolution;
        public static final double CountsPerMeter = 1.0 / MetersPerCount ;

        // public static final double WheelDiamater = 6.0 * InchesToMeters; // meters
        // public static final double WheelCircumference = Math.PI * WheelDiamater; // meters
        // public static final double WheelCountsPerMeter = (1.0 / WheelCircumference) * Counts_Per_Wheel_Revolution;
        public static final double MotorVelocityOneMeterPerSecond = CountsPerMeter / 10.0;

        public static final double RPMsToUnitsPerHundredMilliseconds = 1.0 / 600.0;
        // public static final double DesiredRPMsForDrive = 560.0;
        // public static final double MaxDriveVelocity = 6000.0;
        // public static final double VelocityInputConversionFactor = DesiredRPMsForDrive * Counts_Per_Revolution * RPMsToUnitsPerHundredMilliseconds;

        public static final int EncoderTimeout = 30;  // mSec
        public static final int MainFeedbackLoop = 0;

        public static final double CorrectionRotationSpeed = 2.0 ;  // meters/sec

        // public static final double maxAngleChangeForAlignFinish = 0.5;  // degrees
        // public static final double maxAngleDifferenceBetweenNavXAndVision = 0.01;
        // public static final double alignTimeoutTime = 1000;
        // public static final double alignMemorySize = 3;
    }


    public static final class AutoConstants {
        // public static final double kMaxSpeedMetersPerSecond = 4 ;
        // public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxSpeedMetersPerSecond = 2.25; // this works
        public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 10*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond / 0.5;
        // public static final double kAutoDriveDistanceInches = 1;
        // public static final double kAutoDriveSpeed = 0.2;

        // these are the constants from waterbury
        // public static final double kPXController = 6.0;
        // public static final double kPYController = 6.0;
        // public static final double kPThetaController = 12.0;

        public static final double kPXController = 6.0 ;
        public static final double kPYController = 6.0 ;
//        public static final double kPThetaController = 4.0;  // this works
        public static final double kPThetaController = 8.0;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints 
            = new TrapezoidProfile.Constraints( kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared );
    }

    public static final class VisionConstants {
        public static final int ID_CameraLight = 5;
        public static final double AlignmentTolerence = 5.0 ; // pixels
    }
}