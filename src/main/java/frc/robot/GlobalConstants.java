package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class GlobalConstants {

	public enum RobotMode {
		REAL,
		TESTING,
		SIM,
	} // No point in having replay tbh

	public static final RobotMode ROBOT_MODE = RobotBase.isReal() ? RobotMode.REAL : RobotMode.SIM; // Change this to TESTING for testing purposes

	public static final double SIMULATION_PERIOD = 0.02;
}
