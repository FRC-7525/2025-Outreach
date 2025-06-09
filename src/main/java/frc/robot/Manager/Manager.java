

package frc.robot.Manager;

import static frc.robot.Manager.ManagerStates.*;
import static frc.robot.Manager.ManagerConstants.*;
import static frc.robot.GlobalConstants.Controllers.*;
import org.team7525.subsystem.Subsystem;

import frc.robot.Subsystems.AdjustableHood.AdjustableHood;
import frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystem;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

import org.littletonrobotics.junction.Logger;

public class Manager extends Subsystem<ManagerStates> {
    public static Manager instance;
    public Intake intake;
    public AdjustableHood adjustableHood;
    public Shooter shooter;
    //public Drive drive;
    public Indexer indexer;
    public HoodedShooterSupersystem hoodedShooterSupersystem;


    public static Manager getInstance() {
        if(instance == null){
            instance = new Manager();
        }
        return instance;

     }


    public Manager() {
        super("Manager", ManagerStates.IDLE);

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

        addTrigger(INTAKE_PASSING, DYNAMIC_ALIGN, DRIVER_CONTROLLER::getYButtonPressed);
        addTrigger(DYNAMIC_ALIGN, DYNAMIC_SHOOT, DRIVER_CONTROLLER::getYButtonPressed);

    }

    @Override
    public void runState() {

        if(DRIVER_CONTROLLER.getBButtonPressed()){
            setState(IDLE);
        }

        if(hoodedShooterSupersystem.readyToShoot() && getState() == FIXED_ALIGN ) {
            setState(FIXED_SHOOT);
        }

        if(hoodedShooterSupersystem.readyToShoot() && getState() == DYNAMIC_ALIGN ) {
            setState(DYNAMIC_SHOOT);
        }

       
        Logger.recordOutput(SUBSYSTEM_NAME + "/State Time", getStateTime());
	    Logger.recordOutput(SUBSYSTEM_NAME + "/State String", getState().getStateString());

        intake.setState(getState().getIntakeStates());
        intake.periodic();

        hoodedShooterSupersystem.setState(getState().getHoodedShooterSupersystemStates());
        hoodedShooterSupersystem.periodic();

        adjustableHood.setState(getState().getAdjustableHoodStates());
        adjustableHood.periodic();

        shooter.setState(getState().getShooterStates());
        shooter.periodic();


        indexer.setState(getState().getIndexerStates());
        indexer.periodic();

        //drive.periodic();
    }


    public boolean hasGamepiece() {
        return intake.hasGamepiece();
    }

}


