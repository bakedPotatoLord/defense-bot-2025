// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final Gyro gyro = new Gyro();
	private final DriveSubsystem driveBase = new DriveSubsystem(gyro);
	private final LEDSubsystem LEDs = new LEDSubsystem();


	// The driver's controller
	CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
	// the mechanism controller
	CommandXboxController m_MechanismController = new CommandXboxController(OIConstants.kMechanismControllerPort);

	SendableChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// register commands

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// driveBase.setDefaultCommand(
		// 		new RunCommand(() -> driveBase.drive(

		// 				-m_driverController.getRightX(),
		// 				true),
		// 				driveBase));
		driveBase.setDefaultCommand(
				new RunCommand(() -> driveBase.drive(
						MathUtil.applyDeadband(-m_driverController.getLeftY(),0.1),
						MathUtil.applyDeadband(m_driverController.getLeftX(),0.1),
						MathUtil.applyDeadband(-m_driverController.getRightX(),0.1),
						// MathUtil.applyDeadband(1,0.1),
						
						true),
						driveBase));

		LEDs.startAll();

		autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
		SmartDashboard.putData("Auto Mode", autoChooser);

	}

	private void configureButtonBindings() {


		// Config driver controller buttons
		m_driverController.leftBumper()
				.onTrue(driveBase.turnAmmount(Rotation2d.fromRotations(-0.25)));
		m_driverController.rightBumper()
				.onTrue(driveBase.turnAmmount(Rotation2d.fromRotations(0.25)));

	}


	public Command getAutonomousCommand() {
		return autoChooser == null
				? new PrintCommand("uh... there's no auto, dude")
				: autoChooser.getSelected();
	}
}
