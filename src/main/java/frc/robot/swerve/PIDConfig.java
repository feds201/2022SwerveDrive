package frc.robot.swerve;

public class PIDConfig {
    public double pGain;
    public double pMin;
    public double pMax;

    public double iGain;
    public double iMin;
    public double iMax;

    public double dGain;
    public double dMin;
    public double dMax;

    public PIDConfig(double pGain, double pMin, double pMax,
                            double iGain, double iMin, double iMax,
                            double dGain, double dMin, double dMax) {
        this.pGain = pGain;
        this.pMin = pMin;
        this.pMax = pMax;

        this.iGain = iGain;
        this.iMin = iMin;
        this.iMax = iMax;

        this.dGain = dGain;
        this.dMin = dMin;
        this.dMax = dMax;
    }
}
