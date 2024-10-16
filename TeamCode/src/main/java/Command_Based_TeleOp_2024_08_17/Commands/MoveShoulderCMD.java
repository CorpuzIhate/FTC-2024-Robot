package Command_Based_TeleOp_2024_08_17.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command_Based_TeleOp_2024_08_17.Constants;
import Command_Based_TeleOp_2024_08_17.Subsystems.ShoulderSubsystem;

public class MoveShoulderCMD extends CommandBase {
    private final ShoulderSubsystem m_shoulderSub;
    private final Telemetry m_dashboardTelemetry;
    private final double m_currentSetpoint;

    private  Motor shoulderMotor;
    private  PIDFController feedforward;
    private  double output;
    private boolean shoulderisAtpoint;

    public MoveShoulderCMD(ShoulderSubsystem shoulderSub, Telemetry dashboardTelemetry,
                           double currentSetpoint){
        m_shoulderSub = shoulderSub;
        m_dashboardTelemetry = dashboardTelemetry;
        m_currentSetpoint = currentSetpoint;
        addRequirements(m_shoulderSub);
    }

    @Override
    public void initialize(){
        shoulderMotor  = m_shoulderSub.getShoulderMotor();
        feedforward = m_shoulderSub.getShoulderFeedforward();

        shoulderMotor.resetEncoder();
        shoulderMotor.setRunMode(Motor.RunMode.RawPower);

        m_dashboardTelemetry.addData("kP", Constants.ShoulderPIDConstants.kP);
        m_dashboardTelemetry.addData("kI",Constants.ShoulderPIDConstants.kI);
        m_dashboardTelemetry.addData("kD",Constants.ShoulderPIDConstants.kD);
        m_dashboardTelemetry.addData("kF",Constants.ShoulderPIDConstants.kF);
        m_dashboardTelemetry.addData("position",shoulderMotor.getCurrentPosition());
        m_dashboardTelemetry.update();

    }
    @Override
    public void execute(){
        output = feedforward.calculate(shoulderMotor.getCurrentPosition(), m_currentSetpoint);

        m_dashboardTelemetry.update();
        if(feedforward.atSetPoint()){

            m_dashboardTelemetry.addData("atPoint","yes");


        }

            shoulderMotor.set(output); // error might happen here cuz
            // we just pass the shoulderMotor object through the subsystem


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
