// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class MiscConstants{
		public static final boolean TUNING_MODE = true;
	}

	public static final class CurrentLimits {
		public static final int driveMotorStall = 30;
		public static final int driveMotorFree = 35;

		public static final int turningMotorStall = 30;
		public static final int turningMotorFree = 35;
	}

	public static final class DriveConstants {
		public static final int kFrontLeftDriveMotorPort = 2;
		public static final int kRearLeftDriveMotorPort = 8;
		public static final int kFrontRightDriveMotorPort = 4;
		public static final int kRearRightDriveMotorPort = 6;

		public static final int kFrontLeftTurningMotorPort = 1;
		public static final int kRearLeftTurningMotorPort = 7;
		public static final int kFrontRightTurningMotorPort = 3;
		public static final int kRearRightTurningMotorPort = 5;

		public static final int kFrontLeftTurningEncoderPorts = 17;
		public static final int kRearLeftTurningEncoderPorts = 19;
		public static final int kFrontRightTurningEncoderPorts = 18;
		public static final int kRearRightTurningEncoderPorts = 20;

		public static final double kFrontLeftTurningEncoderOffset = 0; // 8
		public static final double kRearLeftTurningEncoderOffset = 0; // 6
		public static final double kFrontRightTurningEncoderOffset = 0;// 2
		public static final double kRearRightTurningEncoderOffset = 0;// 4

		public static final boolean kFrontLeftTurningEncoderReversed = true;
		public static final boolean kRearLeftTurningEncoderReversed = true;
		public static final boolean kFrontRightTurningEncoderReversed = true;
		public static final boolean kRearRightTurningEncoderReversed = true; 

		public static final boolean kFrontLeftDriveEncoderReversed = true;
		public static final boolean kRearLeftDriveEncoderReversed = true;
		public static final boolean kFrontRightDriveEncoderReversed = true;
		public static final boolean kRearRightDriveEncoderReversed = true;

		public static final double kTrackWidth = 0.65;
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = 0.45;
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		public static final boolean kGyroReversed = false;

		public static final double kMaxSpeedMetersPerSecond = 4.2;
		public static final double kDemoSpeedMetersPerSecond = 1.0;

		// used for keeping the robot pointed in the right direction
		public static final class RotationPID {
			public static final double kP = 4.5;
			public static final double kI = 0.0;
			public static final double kD = 0.2;
		}

		public static final class RotationFF {
			public static final double kS = 0.00;
			public static final double kV = 0.0;
			public static final double kA = 0.0;
		}

		public static final double rotationPostitionTolerance = 2.0;
		public static final double rotationVelocityTolerance = 0.0002;
	}

	public static final class ModuleConstants {

		public static final class TurningPID {
			public static double kP = 0.16;
			public static double kI = 0.0;
			public static double kD = 0.0;
		}

		public static final class DrivePID {
			public static double kFF = 0.16;
			public static double kP = 0.0;
			public static double kI = 0.0;
			public static double kD = 0.0;
		}

		public static final double kMaxModuleAngularSpeed = 100; // rad/s
		public static final double kMaxModuleAngularAcceleration = 100; // rad/s^2

		public static final double kWheelDiameter = 0.095; // meters
		public static final double driveGearRatio = 1.0 / 6.75;

		// converting to m
		public static final double kdrivePositionConversionFactor = driveGearRatio * kWheelDiameter * Math.PI;
		// converting to m/s
		public static final double kdriveVelocityConversionFactor = (driveGearRatio / 60.0) * kWheelDiameter * Math.PI;
		// converting to radians
		public static final double kTurningConversionFactor = 1.0 / 12.8;
	}

	public static final class PDPConstants {
		public static final int deviceID = 10;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kMechanismControllerPort = 1;

		public static final double joystickDeadband = 0.15;

		public static final double rotationMultiplier = 0.01;
	}
}
