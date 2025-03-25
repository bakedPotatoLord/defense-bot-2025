// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.DrivePID;
import frc.robot.Constants.ModuleConstants.TurningPID;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.tunable.gains.TunableDouble;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;

public class SwerveModule extends SubsystemBase {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;

  private final CANcoder m_turningEncoder;

  private final SparkClosedLoopController m_driveController;

  public final TunableDouble turningEncoderOffset;

  public final DoubleTelemetryEntry velocitySetpoint = new DoubleTelemetryEntry(getName()+"/velocitySetpoint", true);
  public final DoubleTelemetryEntry velocityActual = new DoubleTelemetryEntry(getName()+"/velocityActual", true);


  // Using a TrapezoidProfile PIDController to allow for smooth turning

  private final TunableTelemetryPIDController turningcontroller;

  public SwerveModule(
      String name,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double turingEncoderOffset,
      boolean driveMotorReversed,
      boolean turningMotorReversed) {
        setName(name);
    m_driveMotor = new TelemetryCANSparkMax(driveMotorChannel, MotorType.kBrushless,getName()+"drive/",MiscConstants.TUNING_MODE);

    m_turningMotor = new TelemetryCANSparkMax(turningMotorChannel, MotorType.kBrushless,getName()+"turning/",MiscConstants.TUNING_MODE);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new CANcoder(turningEncoderChannel);

    m_driveController = m_driveMotor.getClosedLoopController();

    var driveConfig = new SparkMaxConfig();
    


    driveConfig.smartCurrentLimit(
        CurrentLimits.driveMotorStall,
        CurrentLimits.driveMotorFree)
        .idleMode(IdleMode.kBrake);
    driveConfig.closedLoop
        .pidf(
            DrivePID.kP, DrivePID.kI, DrivePID.kD, DrivePID.kFF);
    driveConfig.encoder
        .positionConversionFactor(ModuleConstants.kdrivePositionConversionFactor)
        .velocityConversionFactor(ModuleConstants.kdriveVelocityConversionFactor);
    driveConfig.inverted(driveMotorReversed);

    m_driveMotor.configure(driveConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    System.out.println("velocity conversion factor"+m_driveMotor.configAccessor.encoder.getVelocityConversionFactor());

    SparkMaxConfig turningConfig = new SparkMaxConfig();

    turningConfig
        .smartCurrentLimit(
            CurrentLimits.turningMotorStall,
            CurrentLimits.turningMotorFree)
        .idleMode(IdleMode.kCoast)
        .inverted(turningMotorReversed);
        
    m_turningMotor.configure(turningConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.turningEncoderOffset = new TunableDouble(getName()+"/offset", turingEncoderOffset, MiscConstants.TUNING_MODE) ;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningcontroller = new TunableTelemetryPIDController(
      getName(),
      new TunablePIDGains(
        getName()+"/PID",
        TurningPID.kP,
        TurningPID.kI,
        TurningPID.kD,
        MiscConstants.TUNING_MODE)
      );

    turningcontroller.enableContinuousInput(-Math.PI, Math.PI);
    turningcontroller.setTolerance(0.1);

    addChild("turning encoder", m_turningEncoder);
  }

  public Rotation2d getTurningRotation() {
    return Rotation2d.fromRotations(-m_turningEncoder.getAbsolutePosition().getValueAsDouble() + turningEncoderOffset.get());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getTurningRotation());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getTurningRotation());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(getTurningRotation());
    
    // Calculate the drive output from the drive PID controller.
    m_driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    

    velocitySetpoint.append( desiredState.speedMetersPerSecond);
    velocityActual.append(m_driveEncoder.getVelocity());

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = turningcontroller
    .calculate(getTurningRotation().getRadians(),
        desiredState.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_turningMotor.set(turningcontroller.atSetpoint() ?0 : turnOutput);

    
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }

  public void periodic() {
    
    // used for tuning. DO NOT DELETE
    // m_driveController.setFF(FF, 0);
    // m_driveController.setP(kP, 0);
    // m_driveController.setD(kD, 0);

    // SmartDashboard.putNumber("encoder/offset " + m_turningMotor.getDeviceId(),
    // getTurningRotation().getRotations());
  }
}
