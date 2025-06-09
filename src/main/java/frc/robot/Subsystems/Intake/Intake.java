package frc.robot.Subsystems.Intake;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Intake extends Subsystem<IntakeStates> {

	// The IO does something
	private IntakeIO io;
	private IntakeIOInputsAutoLogged inputs;
	private static Intake instance;

	private Intake(IntakeIO io) {
		super("Intake", IntakeStates.IDLE);
		this.io = io;
		inputs = new IntakeIOInputsAutoLogged();
	}

	@Override
	protected void runState() {
		io.setPivotSetpoint(getState().getPivotSetpoint());
		io.setWheelSpeed(getState().getWheelSpeedSetpoint());

		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
	}

	public static Intake getInstance() {
		if (instance == null) {
			IntakeIO intakeIO =
				switch (ROBOT_MODE) {
					case SIM -> new IntakeIOSim();
					case REAL -> new IntakeIOReal();
					case TESTING -> new IntakeIOReal();
				};
			instance = new Intake(intakeIO);
		}
		return instance;
	}

	public boolean hasGamepiece() {
		return io.hasGamepiece();
	}

	public double getStateTime() {
		return super.getStateTime();
	}
}
