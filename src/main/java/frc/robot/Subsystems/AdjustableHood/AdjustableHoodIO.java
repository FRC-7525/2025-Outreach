package frc.robot.Subsystems.AdjustableHood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface AdjustableHoodIO {

    @AutoLog
    public static class AdjustableHoodIOInputs {
       public Angle hoodSetpoint;
       public double hoodPosition;
    }

    public abstract void updateInputs(AdjustableHoodIOInputs inputs);

    public abstract void setHoodSetpoint(Angle setpoint);

    public abstract boolean atHoodSetpoint();

}
