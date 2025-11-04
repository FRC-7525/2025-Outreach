package frc.robot.Subsystems.Intake;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import frc.robot.Subsystems.Intake.IntakeIO.IntakeIOInputs;
import frc.robot.TeamLib.subsystem.*;


public class Intake extends Subsystem<IntakeStates> {

	private IntakeIO io;
	private IntakeIOInputs inputs;
	private static Intake instance;

	private Intake(IntakeIO io) {
		super("Intake", IntakeStates.IDLE);
		this.io = io;
		inputs = new IntakeIOInputs();
	}

	@Override
	protected void runState() {
		io.setPivotSetpoint(getState().getPivotSetpoint());
		io.setWheelSpeed(getState().getWheelSpeedSetpoint());

		io.updateInputs(inputs);
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
