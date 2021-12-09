package frc.robot.swerve;

public interface ISwerveDrive extends ITickable {
	public void setTargetVelocity(double linearAngle, double linearSpeed, double rotate);

	public double[] getAlignments();
	public void setAlignmentsAbsolute(double[] alignments);
	public void setAlignmentsRelative(double[] alignments);
	public void align();

	public double getTargetLinearAngle();
	public double getTargetLinearSpeed();
	public double getTargetRotate();
}
