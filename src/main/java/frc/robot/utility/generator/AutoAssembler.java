package frc.robot.utility.generator;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.command.factories.IntakeCommand;
import frc.robot.command.factories.ShooterCommand;

import java.util.*;

public class AutoAssembler {

    public enum StartPosition {
        SubwooferLeft,
        SubwooferMiddle,
        SubwooferRight
    }

    public enum Note {
        CloseLeft,
        CloseMiddle,
        CloseRight,
        CenterLeft,
        CenterMiddleLeft,
        CenterMiddle,
        CenterMiddleRight,
        CenterRight
    }

    private static final Map<Note, Integer> noteId = (DriverStation.getAlliance().isPresent() &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ?
            Map.of(Note.CloseRight, 0, Note.CloseMiddle, 1, Note.CloseLeft, 2,
                    Note.CenterRight, 3, Note.CenterMiddleRight, 4, Note.CenterMiddle, 5,
                    Note.CenterMiddleLeft, 6, Note.CenterLeft, 7) :
            Map.of(Note.CloseLeft, 0, Note.CloseMiddle, 1, Note.CloseRight, 2,
                    Note.CenterLeft, 3, Note.CenterMiddleLeft, 4, Note.CenterMiddle, 5,
                    Note.CenterMiddleRight, 6, Note.CenterRight, 7);

    public enum EndAction {
        PrepShoot,
        PrepGrab,
        PrepAmp,
        PrepSource,
        PrepTrap
    }

    private static StartPosition startPosition;
    private static Note[][] notes;
    private static EndAction endAction;

    public static void setStartPosition(StartPosition position) {
        startPosition = position;
    }
    public static void setNotes(Note[][] notePositions) {
        notes = notePositions;
    }
    public static void setEndAction(EndAction action) {
        endAction = action;
    }

//    public static Command getAutoCommand() {
//        ArrayList<Command> commands = new ArrayList<Command>();
//
//        commands.add(IntakeCommand.toIntakePosition());
//        commands.add(ShooterCommand.warmup(Constants.Shooter.Setpoints.subwoofer));
//    }
//
//
//
//    private static Command getNoteTransferCommand(Note a, Note b) {
//        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory()
//    }
}
