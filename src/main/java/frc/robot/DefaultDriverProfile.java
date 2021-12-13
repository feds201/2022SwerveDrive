package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DefaultDriverProfile implements IDriverProfile {

    public static final double DEADZONE = 0.05;
    
    public double[] getInputs(Joystick joystick) {
        double forward = -joystick.getRawAxis(1);
        double strafe = joystick.getRawAxis(0);
        double rotate = joystick.getRawAxis(4);

        double linearAngle = -Math.atan2(forward, strafe) / Math.PI / 2 + 0.25;
        linearAngle = (linearAngle % 1 + 1) % 1;
        double linearSpeed = Math.sqrt(forward * forward + strafe * strafe);

        return new double[] { linearAngle, deadzone(linearSpeed, DEADZONE), deadzone(rotate, DEADZONE) };
    }

	private static double deadzone(double input, double threshold) {
		if (Math.abs(input) < threshold)
			return 0;
		return Math.signum(input) * (Math.abs(input) - threshold) / (1 - threshold);
	}
}
