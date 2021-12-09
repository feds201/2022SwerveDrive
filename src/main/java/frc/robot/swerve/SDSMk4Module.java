package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SDSMk4Module implements ISwerveModule {

	public static final double DRIVE_GEAR_ENCODER_COUNTS = 8.41 * 2048;
	public static final double STEER_GEAR_ENCODER_COUNTS = 4096;

	private final TalonFX steer;
	private final TalonFX drive;
	private double angleOffset;

	private double targetAngle = 0;
	private double targetSpeed = 0;
	private boolean reversed = false;

	private PIDConfig pidConfig;
	private double iValue = 0;
	private double dLast = 0;
	private boolean debug = false;

	public SDSMk4Module(int steerChannel, int driveChannel, int encoderChannel, double angleOffset,
						PIDConfig pidConfig, double maxThrottle) {
		steer = new TalonFX(steerChannel);
		steer.setNeutralMode(NeutralMode.Brake);
		steer.configRemoteFeedbackFilter(encoderChannel, RemoteSensorSource.TalonSRX_SelectedSensor, 0);
		steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
		
		drive = new TalonFX(driveChannel);
		drive.setNeutralMode(NeutralMode.Coast);
		drive.configOpenloopRamp(maxThrottle);

		this.angleOffset = angleOffset;
		this.pidConfig = pidConfig;
	}

	@Override
	public void setTargetVelocity(double angle, double speed) {
		targetAngle = (angle % 1 + 1) % 1;
		targetSpeed = speed;
	}
	
	// Written by Michael Kaatz (2021)
	@Override
	public void tick() {
		// Whether the wheel is reversed or not plays a big role in what we should do.
		double effectiveCurrentAngle = (getCurrentAngle() + (reversed ? 0.5 % 1 : 0)) % 1;

		// The loop error is just the negative opposite of the continous error.
		double errorContinous = -(effectiveCurrentAngle - targetAngle);
		double errorLoop = -(1 - Math.abs(errorContinous)) * Math.signum(errorContinous);

		// Sign is preserved since it is used to determine which way we need to turn the wheel.
		double targetError;
		if (Math.abs(errorContinous) < Math.abs(errorLoop))
			targetError = errorContinous;
		else
			targetError = errorLoop;

		// In some cases it is better to reverse the direction of the drive wheel rather than spinning all the way around.
		// The threshold to reverse is 0.3 vs what you might expect (0.25) to prevent the wheels from losing control.
		if (Math.abs(targetError) > 0.3)
		{
			reversed = !reversed;
			// Quick way to recalculate the offset of the new angle from the target.
			targetError = -Math.signum(targetError) * (0.5 - Math.abs(targetError));
		}

		// We don't want to move the wheels if we don't have to.
		if (targetSpeed != 0)
		{
			// Run PID
			double pValue = targetError * pidConfig.pGain;
			pValue = clamp(pValue, pidConfig.pMin, pidConfig.pMax);
			iValue += targetError * pidConfig.iGain;
			iValue = clamp(iValue, pidConfig.iMin, pidConfig.iMax);
			double dValue = clamp((targetError - dLast) * pidConfig.dGain, pidConfig.dMin, pidConfig.dMax);
			dLast = targetError;
			double outputRaw = pValue + iValue + dValue;
			double outputClamped = clamp(outputRaw, -1, 1);
			
			double steerTarget = outputClamped;
			double driveTarget = targetSpeed * (reversed ? -1 : 1);

			// The module's wheels are actually reversed relative to the motor shaft so this is negative.
			steer.set(ControlMode.PercentOutput, -steerTarget);
			drive.set(ControlMode.PercentOutput, -driveTarget);

			if (debug)
			{
				NetworkTable table = NetworkTableInstance.getDefault().getTable("swervedebug");
				table.getEntry("pGain").setDouble(pidConfig.pGain);
				table.getEntry("iGain").setDouble(pidConfig.iGain);
				table.getEntry("dGain").setDouble(pidConfig.dGain);
				table.getEntry("pValue").setDouble(pValue);
				table.getEntry("iValue").setDouble(iValue);
				table.getEntry("dValue").setDouble(dValue);
				table.getEntry("error").setDouble(targetError);
				table.getEntry("outputRaw").setDouble(outputRaw);
				table.getEntry("outputClamped").setDouble(outputClamped);
			}
		}
		else
		{
			steer.set(ControlMode.PercentOutput, 0);
			drive.set(ControlMode.PercentOutput, 0);
			iValue = 0;
			dLast = 0;
		}
	}

	private static double clamp(double value, double min, double max)
	{
		if (value > max)
			return max;
		if (value < min)
			return min;
		return value;
	}

	// BOILERPLATE CODE

	@Override
	public double getAngleOffset() {
		return angleOffset;
	}

	@Override
	public void setAngleOffsetAbsolute(double offset) {
		angleOffset = offset;
	}

	@Override
	public void setAngleOffsetRelative(double offset) {
		angleOffset += offset;
	}

	@Override
	public double getCurrentAngle() {
		return ((1 - steer.getSelectedSensorPosition() / STEER_GEAR_ENCODER_COUNTS - angleOffset) % 1 + 1) % 1;
	}

	@Override
	public double getCurrentSpeed() {
		return -drive.getSelectedSensorVelocity() / DRIVE_GEAR_ENCODER_COUNTS * 10;
	}

	@Override
	public double getTargetAngle() {
		return targetAngle;
	}

	@Override
	public double getTargetSpeed() {
		return targetSpeed;
	}

	public PIDConfig getPIDConfig() {
		return pidConfig;
	}

	public void setPIDConfig(PIDConfig config) {
		pidConfig = config;
	}

	public void setDebug(boolean debug) {
		this.debug = debug;
	}
}
