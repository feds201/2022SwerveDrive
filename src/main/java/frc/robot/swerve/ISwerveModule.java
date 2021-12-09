package frc.robot.swerve;

public interface ISwerveModule extends ITickable {
	public void setTargetVelocity(double angle, double speed);

	public double getAngleOffset();
	public void setAngleOffsetAbsolute(double offset);
	public void setAngleOffsetRelative(double offset);

	public double getTargetAngle();
	public double getTargetSpeed();
	public double getCurrentAngle();
	public double getCurrentSpeed();
}
