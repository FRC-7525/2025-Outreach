package frc.robot.Subsystems.Indexer;

public interface IndexerIO {
    
    public void logInfo();

    public double getMotorSpeed();

    public double getMotorSpeedSetpoint();

    public int getBallCount();

    public void setMotorSpeed(double speed);



}
