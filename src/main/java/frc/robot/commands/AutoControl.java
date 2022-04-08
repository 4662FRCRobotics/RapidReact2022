package frc.robot.commands;

//import java.lang.module.ModuleDescriptor.Requires;

//import com.fasterxml.jackson.databind.deser.std.AtomicBooleanDeserializer;

//import static edu.wpi.first.wpilibj.DriverStation.getInstance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.libraries.AutonomousCommands;
import frc.robot.libraries.ConsoleJoystick;
import frc.robot.libraries.Step;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.ShooterConstants;
import frc.robot.libraries.AutoStepCommand;

public class AutoControl extends CommandBase {
    AutonomousCommands<AutoStepCommand> m_autoStepCommand;
    ConsoleJoystick m_console;
    Drive m_drive;
    Hopper m_hopper;
    Intake m_intake;
    Shooter m_shooter;
    Vision m_vision;
    Climb m_climb;

    WaitForCount m_wait;

    int m_positionSwitch;

    AutoStepCommand m_currentStepName;
    Command m_currentCommand;
    int m_waitCount;

    private Command m_cmdLaunch1;
    private Step m_stepLaunch1;

    private Command m_cmdDrive1;
    private Step m_stepDrive1;

    private Command m_cmdLaunch2;
    private Step m_stepLaunch2;
        
    private Command m_waitLoop;
    private Step m_stepWaitLoop;

    private Command m_driveIntake;   

    private int m_stepIndex = 0;
    /*
    Step m_step[] = { new Step(AutoStepCommand.DRIVE1.name()),
               new Step(AutoStepCommand.TURNP90.name(), () -> m_console.cnsl_btn_2.get()),
               new Step(AutoStepCommand.TURNP90.name(), () -> m_console.cnsl_btn_2.get()),
               new Step(AutoStepCommand.WAITLOOP.name(), () -> true),
               new Step(AutoStepCommand.DRIVE1.name(), () -> m_console.cnsl_btn_3.get()),
               new Step(AutoStepCommand.LAUNCH1.name(), () -> m_console.cnsl_btn_4.get()),
               new Step(AutoStepCommand.END.name(), () -> true)
        };
    */
    /*private Step<AutoStepCommand> m_step[] = {m_stepLaunch1,
                new Step(AutoStepCommand.DRIVE1)
    };
    */
    //private Step<AutoStepCommand>[] m_step = (Step<AutoStepCommand>[]) new Step[10];
    private Step[] m_step;

    public AutoControl(ConsoleJoystick console, Hopper hopper, Drive drive, Climb climb, Intake intake,
            Shooter shooter, Vision vision) {
        m_console = console; //lets us interact with the robot remotely
        m_drive = drive; //lets us move the robot
        m_hopper = hopper; //lets us store balls
        m_shooter = shooter; //lets us shoot balls
        m_intake = intake; //lets us take in balls
        m_climb = climb; //lets us climb things
        m_vision = vision; //lets us see things

        addRequirements(m_drive, m_hopper, m_shooter, m_intake, m_climb, m_vision);

        m_autoStepCommand = new AutonomousCommands<AutoStepCommand>();
        
        //cmdLaunch1: assumes cargo is in shooter
        m_cmdLaunch1 = new ParallelRaceGroup(new WaitCommand(1),
            new BaggageHandlerShoot(m_shooter, () -> ShooterConstants.kSHOOTER_SPEED_AUTO));
        m_stepLaunch1 = new Step(AutoStepCommand.LAUNCH1, () -> m_console.cnsl_btn_2.get());
        m_autoStepCommand.addOption(AutoStepCommand.LAUNCH1, m_cmdLaunch1);

        //cmdLaunch2: turns on shooter, waits, then turns on hopper 
        m_cmdLaunch2 = new ParallelRaceGroup(new WaitCommand(2),
            new BaggageHandlerShoot(m_shooter, () -> ShooterConstants.kSHOOTER_SPEED_AUTO),
            new SequentialCommandGroup(new WaitCommand(0.5), new ShootHopperFeed(m_hopper))
            );  
        m_stepLaunch2 = new Step(AutoStepCommand.LAUNCH2, () -> m_console.cnsl_btn_2.get());
        m_autoStepCommand.addOption(AutoStepCommand.LAUNCH2, m_cmdLaunch2);
        
        //waitLoop: waits how many seconds the right knob says
        m_waitLoop = new WaitForCount(1, () -> m_console.getROT_SW_1());
        m_stepWaitLoop = new Step(AutoStepCommand.WAITLOOP);
        m_autoStepCommand.addOption(AutoStepCommand.WAITLOOP, m_waitLoop);

        //cmdDrive1: drives back before shooting
        m_cmdDrive1 = new AutoDriveDistance(1.5, m_drive);m_autoStepCommand.addOption(AutoStepCommand.LAUNCH2, m_cmdLaunch2);
        m_stepDrive1 = new Step(AutoStepCommand.DRIVE1, () -> m_console.cnsl_btn_3.get());
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE1, m_cmdDrive1);

        //driveIntake: gives ball to shooter
        m_driveIntake = new ParallelRaceGroup(new AutoDriveDistance(-1.5, m_drive),
        new IntakeCargo(m_hopper, m_intake));
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE_INTAKE, m_driveIntake);

        //
        m_autoStepCommand.addOption(AutoStepCommand.DEPLOY_INTAKE, new IntakeCargo(hopper, intake));
        m_autoStepCommand.addOption(AutoStepCommand.DRIVE2, new AutoDriveDistance(1.5, m_drive));
        m_autoStepCommand.addOption(AutoStepCommand.TURNP90, new AutoTurnAngle(90.0, m_drive));
        m_autoStepCommand.addOption(AutoStepCommand.WAIT2, new WaitCommand(2));
        m_autoStepCommand.addOption(AutoStepCommand.END, new End());

        m_step = (Step[]) new Step[] 
            {m_stepWaitLoop, m_stepLaunch1, m_stepDrive1};

        m_stepIndex = 0;
    
    }

    private void dashboardCmd(AutoStepCommand cmdName) {
        SmartDashboard.putString("Auto Cmd Step", cmdName.name()); //displays "Auto Cmd Step" on the dashboard
    }

    @Override
    public void initialize() {
        // getInstance();
        if (DriverStation.isEnabled()) {
            m_drive.setShiftLow();
            m_waitCount = 0;
            m_stepIndex = -1;
            m_currentStepName = getNextActiveCommand();
            m_currentCommand = m_autoStepCommand.getSelected(m_currentStepName);
            dashboardCmd(m_currentStepName);
            m_currentCommand.initialize();
        }
    }

    @Override
    public void execute() {
        m_currentCommand.execute();
        // System.out.println("execute");
    }

    @Override
    public void end(boolean interrupted) {
        m_currentCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        boolean areWeThereYet = true;
        if (m_currentCommand.isFinished() == false) {
            areWeThereYet = false;
        } else {
            areWeThereYet = stepNextCommand();
        }
        // System.out.println("isFinished");
        return areWeThereYet;
    }

    public boolean stepNextCommand() {
        boolean areWeThereYet = true;

        //gets next active command
        m_currentStepName = getNextActiveCommand(); 
        if (m_currentStepName.equals(AutoStepCommand.END)) {
            areWeThereYet = true;
        } else {
            switchCommand(m_autoStepCommand.getSelected(m_currentStepName));
            areWeThereYet = false;
        }
        //makes Donkey appear and name appear on dashboard
        dashboardCmd(m_currentStepName);
        return areWeThereYet; 
    }

    //stops current command then goes to next one
    private void switchCommand(final Command cmd) {
        m_currentCommand.end(false);
        m_currentCommand = cmd;
        m_currentCommand.initialize();
    }

    //gets the next available command
    private AutoStepCommand getNextActiveCommand() {

        // System.out.println("getNextActiveCommand");

        AutoStepCommand returnStepName = null;

        while (returnStepName == null) {
            m_stepIndex++;
            if (m_stepIndex >= m_step.length) {
                returnStepName = AutoStepCommand.END;
            } else {
                if (m_step[m_stepIndex].isTrue()) {
                    returnStepName = m_step[m_stepIndex].getName();
                }
            }
        }
        // System.out.println(returnStepName);
        return returnStepName;
    }
}
