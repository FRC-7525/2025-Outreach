package frc.robot.Subsystems.Indexer;

import static frc.robot.Subsystems.Indexer.IndexerConstants.SUBSYSTEM_NAME;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.TeamLib.subsystem.*;

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
		SmartDashboard.putString(SUBSYSTEM_NAME + "/state", getState().getStateString());
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
