package frc.robot.Utilitys;

import frc.robot.GlobalConstants.Controllers;

public class Utilitys {

	public static class controllers {

		public static void clearCache() {
			// Clear operator controller cache
			Controllers.OPERATOR_CONTROLLER.getAButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getBButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getXButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getYButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getBackButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getLeftBumperButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getLeftStickButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getRightBumperButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getRightStickButtonPressed();
			Controllers.OPERATOR_CONTROLLER.getStartButtonPressed();

			// Clear driver controller cache
			Controllers.DRIVER_CONTROLLER.getAButtonPressed();
			Controllers.DRIVER_CONTROLLER.getBButtonPressed();
			Controllers.DRIVER_CONTROLLER.getXButtonPressed();
			Controllers.DRIVER_CONTROLLER.getYButtonPressed();
			Controllers.DRIVER_CONTROLLER.getBackButtonPressed();
			Controllers.DRIVER_CONTROLLER.getLeftBumperButtonPressed();
			Controllers.DRIVER_CONTROLLER.getLeftStickButtonPressed();
			Controllers.DRIVER_CONTROLLER.getRightBumperButtonPressed();
			Controllers.DRIVER_CONTROLLER.getRightStickButtonPressed();
			Controllers.DRIVER_CONTROLLER.getStartButtonPressed();
		}
	}
}
