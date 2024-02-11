package frc.robot.subsystem.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.utility.conversion.AngleUtil;

public class IntakeIOSim extends IntakeIO {

    private final SingleJointedArmSim deploy = new SingleJointedArmSim(DCMotor.getNEO(1),
            Constants.Intake.gearRatioDeploy, Constants.Intake.Simulation.moi, Constants.Intake.Simulation.length,
            0.0, AngleUtil.degToRad(Constants.Intake.maxAngle), true, 0.0);

    private boolean brakeMode = true;



    public IntakeIOSim(IntakeIOInputsAutoLogged input) {
        inputs = input;
    }



    @Override
    public void updateInputs() {

    }

    @Override
    public void setDeployMotor(double volts) {
        deploy.setInputVoltage(volts);
    }

    @Override
    public void setIntakeMotor(double volts) {

    }

    @Override
    public void setDeployBrake(boolean brake) {
        brakeMode = brake;
    }
}
