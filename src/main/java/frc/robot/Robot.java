// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PersistentException;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.swerve.FourCornerSwerveDrive;
import frc.robot.swerve.ISwerveDrive;
import frc.robot.swerve.ISwerveModule;
import frc.robot.swerve.PIDConfig;
import frc.robot.swerve.SDSMk4Module;

public class Robot extends TimedRobot {
	
	private Joystick joystick;
	private ISwerveDrive swerveDrive;
	private PIDConfig pidConfig;

	private IDriverProfile[] profiles = new IDriverProfile[]
	{
		new DefaultDriverProfile()
	};
	private int selectedProfile = 0;

	public Robot() {
		super(0.05);
	}

	@Override
	public void robotInit() {
		try
		{
			NetworkTableInstance.getDefault().getTable("swervealignment")
				.loadEntries(Filesystem.getOperatingDirectory() + "/swerve.ini");
		}
		catch (PersistentException e)
		{
			System.err.println("Error loading swerve drive alignment");
			System.err.println(e);
		}
		
		joystick = new Joystick(0);

		TalonSRX talon1 = new TalonSRX(1);
		TalonSRX talon2 = new TalonSRX(2);
		TalonSRX talon3 = new TalonSRX(3);
		TalonSRX talon4 = new TalonSRX(4);
		talon1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon4.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		talon1.configFeedbackNotContinuous(true, 0);
		talon2.configFeedbackNotContinuous(true, 0);
		talon3.configFeedbackNotContinuous(true, 0);
		talon4.configFeedbackNotContinuous(true, 0);

		NetworkTable table = NetworkTableInstance.getDefault().getTable("swervealignment");
		pidConfig = new PIDConfig(1.0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
									0.6, -0.05, 0.05,
									0.4, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
		double maxThrottle = 1;
		ISwerveModule frontLeft = new SDSMk4Module(21, 22, 1, table.getEntry("index0").getDouble(0), pidConfig, maxThrottle);
		ISwerveModule frontRight = new SDSMk4Module(11, 12, 3,table.getEntry("index1").getDouble(0), pidConfig, maxThrottle);
		ISwerveModule backLeft = new SDSMk4Module(31, 32, 4, table.getEntry("index2").getDouble(0), pidConfig, maxThrottle);
		ISwerveModule backRight = new SDSMk4Module(41, 42, 2, table.getEntry("index3").getDouble(0), pidConfig, maxThrottle);
		((SDSMk4Module)frontLeft).setDebug(true);
		
		swerveDrive = new FourCornerSwerveDrive(frontLeft, frontRight, backLeft, backRight,
												new ADXRS450_Gyro(Port.kOnboardCS0), 0.001, 30, 30);
	}

	@Override
	public void robotPeriodic() {
		swerveDrive.tick();
	}

	@Override
	public void teleopPeriodic() {
		double[] inputs = profiles[selectedProfile].getInputs(joystick);
		swerveDrive.setTargetVelocity(inputs[0], inputs[1], inputs[2]);
	}

	@Override
	public void testPeriodic() {
		if (joystick.getRawButton(7) && joystick.getRawButton(8))
		{
			if (joystick.getRawButtonPressed(7) || joystick.getRawButtonPressed(8))
			{
				swerveDrive.align();
				double[] alignments = swerveDrive.getAlignments();
				try
				{
					NetworkTable table = NetworkTableInstance.getDefault().getTable("swerve");
					table.getEntry("index0").setDouble(alignments[0]);
					table.getEntry("index1").setDouble(alignments[1]);
					table.getEntry("index2").setDouble(alignments[2]);
					table.getEntry("index3").setDouble(alignments[3]);
					table.saveEntries(Filesystem.getOperatingDirectory() + "/swerve.ini");
				}
				catch (PersistentException e)
				{
					System.err.println("Error saving swerve drive alignment");
					System.err.println(e);
				}
			}
			joystick.setRumble(RumbleType.kLeftRumble, 1);
		}
		else
		{
			joystick.setRumble(RumbleType.kLeftRumble, 0);
		}

		final double step = 0.05;
		if (joystick.getRawButtonPressed(3))
			pidConfig.pGain -= step;
		else if (joystick.getRawButtonPressed(2))
			pidConfig.pGain += step;
		else if (joystick.getRawButtonPressed(1))
			pidConfig.dGain -= step;
		else if (joystick.getRawButtonPressed(4))
			pidConfig.dGain += step;
		else if (joystick.getRawButtonPressed(5))
			pidConfig.iGain -= step;
		else if (joystick.getRawButtonPressed(6))
			pidConfig.iGain += step;
			
		double[] inputs = profiles[selectedProfile].getInputs(joystick);
		swerveDrive.setTargetVelocity(inputs[0], inputs[1], inputs[2]);
	}
}
