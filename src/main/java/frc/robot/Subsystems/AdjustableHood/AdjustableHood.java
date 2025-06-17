package frc.robot.Subsystems.AdjustableHood;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.AdjustableHood.AdjustableHoodConstants.SUBSYSTEM_NAME;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class AdjustableHood extends Subsystem<AdjustableHoodStates> {

	private AdjustableHoodIO io;
	private AdjustableHoodIOInputsAutoLogged inputs;
	private Angle dynamicSetpoint;
	private static AdjustableHood instance;

	public static AdjustableHood getInstance() {
		if (instance == null) {
			AdjustableHoodIO AdjustableHoodio =
				switch (ROBOT_MODE) {
					case SIM -> new AdjustableHoodIOSim();
					case REAL -> new AdjustableHoodIOReal();
					case TESTING -> new AdjustableHoodIOReal();
				};
			instance = new AdjustableHood(AdjustableHoodio);
		}

		return instance;
	}

	private AdjustableHood(AdjustableHoodIO io) {
		super(SUBSYSTEM_NAME, AdjustableHoodStates.IDLE);
		this.io = io;
		inputs = new AdjustableHoodIOInputsAutoLogged();
	}

	@Override
	public void runState() {
		if (getState() == AdjustableHoodStates.DYNAMIC) {
			io.setHoodSetpoint(dynamicSetpoint);
		} else {
			io.setHoodSetpoint(getState().getSetpoint());
		}

		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
	}

	public void setDynamicHoodSetpoint(Angle setpoint) {
		dynamicSetpoint = setpoint;
	}

	public boolean atSetpoint() {
		return io.atHoodSetpoint();
	}
}