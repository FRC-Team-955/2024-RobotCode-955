package frc.robot.dashboard;

public enum DashboardSubsystem {
    ROBOT,
    SHOOTER,
    DRIVE,
    INTAKE;

    public String prefix() {
        return switch (this) {
            case ROBOT -> "1 Robot";
            case SHOOTER -> "2 Shooter";
            case DRIVE -> "3 Drive";
            case INTAKE -> "4 Intake";
        };
    }
}
