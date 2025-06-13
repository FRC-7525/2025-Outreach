package frc.robot.Subsystems.AdjustableHood;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

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
