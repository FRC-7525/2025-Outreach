package frc.robot.Manager;

import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Manager.ManagerConstants.*;
import static frc.robot.Manager.ManagerStates.*;

import frc.robot.Subsystems.AdjustableHood.AdjustableHood;
import frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystem;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Manager extends Subsystem<ManagerStates> {

	private static Manager instance;
	private Intake intake;
	private AdjustableHood adjustableHood;
	private Shooter shooter;
	//public Drive drive;
	private Indexer indexer;
	private HoodedShooterSupersystem hoodedShooterSupersystem;
	//private Vision vision;

	public static Manager getInstance() {
		if (instance == null) {
			instance = new Manager();
		}
		return instance;
	}

	public Manager() {
		super(SUBSYSTEM_NAME, ManagerStates.IDLE);
		intake = Intake.getInstance();
		hoodedShooterSupersystem = HoodedShooterSupersystem.getInstance();
		adjustableHood = AdjustableHood.getInstance();
		shooter = Shooter.getInstance();
		//drive = Drive.getInstance();
		indexer = Indexer.getInstance();

		//add triggers
		addTrigger(IDLE, OUTTAKING, DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(INTAKE_PASSING, OUTTAKING, DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(INTAKING, OUTTAKING, DRIVER_CONTROLLER::getXButtonPressed);

		addTrigger(IDLE, INTAKING, DRIVER_CONTROLLER::getAButtonPressed);
		addTrigger(INTAKING, INTAKE_PASSING, DRIVER_CONTROLLER::getAButtonPressed);
		addTrigger(INTAKE_PASSING, FIXED_ALIGN, DRIVER_CONTROLLER::getAButtonPressed);
		addTrigger(FIXED_ALIGN, FIXED_SHOOT, DRIVER_CONTROLLER::getAButtonPressed);
		addTrigger(FIXED_ALIGN, FIXED_SHOOT, () -> hoodedShooterSupersystem.readyToShoot());

		addTrigger(INTAKE_PASSING, DYNAMIC_ALIGN, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(DYNAMIC_ALIGN, DYNAMIC_SHOOT, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(DYNAMIC_ALIGN, DYNAMIC_SHOOT, () -> hoodedShooterSupersystem.readyToShoot());
	}

	@Override
	public void runState() {
		if (DRIVER_CONTROLLER.getBButtonPressed()) {
			setState(IDLE);
		}

		logData();

		intake.setState(getState().getIntakeStates());
		hoodedShooterSupersystem.setState(getState().getHoodedShooterSupersystemStates());
		indexer.setState(getState().getIndexerStates());
		adjustableHood.setState(getState().getAdjustableHoodStates());
		shooter.setState(getState().getShooterStates());

		shooter.periodic();
		hoodedShooterSupersystem.periodic();
		indexer.periodic();
		intake.periodic();
		adjustableHood.periodic();
		//drive.periodic();
		//vision.periodic;
	}

	public boolean hasGamepiece() {
		return intake.hasGamepiece();
	}

	public void logData() {
		Logger.recordOutput(SUBSYSTEM_NAME + "/State Time", getStateTime());
		Logger.recordOutput(SUBSYSTEM_NAME + "/State String", getState().getStateString());
	}
}
