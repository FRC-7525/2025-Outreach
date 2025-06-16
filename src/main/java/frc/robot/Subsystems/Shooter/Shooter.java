package frc.robot.Subsystems.Shooter;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Shooter.ShooterConstants.SUBSYSTEM_NAME;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Shooter extends Subsystem<ShooterStates> {
	private ShooterIO io;
	private ShooterIOInputsAutoLogged inputs;
	private AngularVelocity dynamicSetpoint;
	private static Shooter instance;

	public static Shooter getInstance() {
		if (instance == null) {
			ShooterIO shooterIO =
				switch (ROBOT_MODE) {
					case SIM -> new ShooterIOSim();
					case REAL -> new ShooterIOReal();
					case TESTING -> new ShooterIOReal();
				};
			instance = new Shooter(shooterIO);
		}

		return instance;
	}

	private Shooter(ShooterIO io) {
		super(SUBSYSTEM_NAME, ShooterStates.IDLE);
		this.io = io;
		inputs = new ShooterIOInputsAutoLogged();
	}

	@Override
	public void runState() {
		if (getState() == ShooterStates.DYNAMIC) {
			io.setShooterSetpoint(dynamicSetpoint);
		} else {
			io.setShooterSetpoint(getState().getSetpoint());
		}

		io.updateInputs(inputs);
		Logger.processInputs("Shooter Inputs", inputs);
		Logger.recordOutput("Shooter/State", getState().getStateString());
	}

	public boolean atSetpoint() {
		return io.atTargetSpeed();
	}
}
