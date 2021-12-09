package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public interface IDriverProfile {
    
    public double[] getInputs(Joystick joystick);
}
