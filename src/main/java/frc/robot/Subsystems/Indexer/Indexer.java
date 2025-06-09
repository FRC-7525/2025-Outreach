package frc.robot.Subsystems.Indexer;

import static frc.robot.Subsystems.Indexer.IndexerConstants.SUBSYSTEM_NAME;

import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Indexer extends Subsystem<IndexerState> {

	private IndexerIO io;
	private static Indexer instance;

	private Indexer() {
		super("Indexer", IndexerState.IDLE);
		this.io = switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new IndexerIOTalon();
			case SIM -> new IndexerIOSim();
			case TESTING -> new IndexerIOTalon();
		};
	}

	public static Indexer getInstance() {
		if (instance == null) {
			instance = new Indexer();
		}
		return instance;
	}

	@Override
	protected void runState() {
		Logger.recordOutput(SUBSYSTEM_NAME + "/state", getState().getStateString());
		io.setMotorSpeed(getState().getMotorSpeed());
		io.logInfo();
	}

	public int getBallCount() {
		return io.getBallCount();
	}

	public void setBallCount(int amount) {
		io.setBallCount(amount);
	}
}
