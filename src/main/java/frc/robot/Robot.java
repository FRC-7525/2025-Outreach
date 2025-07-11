// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Manager.Manager;
import frc.robot.Utilitys.Utilitys;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.team7525.misc.CommandsUtil;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

	public Manager manager;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {}

	public void robotInit() {
		manager = Manager.getInstance();

		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
		CommandsUtil.logCommands();
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		manager.periodic();
		Utilitys.controllers.clearCache();
	}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}
}
