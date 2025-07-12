package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;

public class GlobalConstants {

	public enum RobotMode {
		REAL,
		TESTING,
		SIM,
	} // No point in having replay tbh

	public static final RobotMode ROBOT_MODE = RobotBase.isReal() ? RobotMode.REAL : RobotMode.SIM; // Change this to TESTING for testing purposes

	public static final double SIMULATION_PERIOD = 0.02;

	public static class Controllers {

		public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
		public static final XboxController OPERATOR_CONTROLLER = new XboxController(1);
	}
}
