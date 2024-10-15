package Command_Based_TeleOp_2024_08_17.Commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

import Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;

public class TeleOpJoystickRobotCentricCMD extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumDriveBaseSubsystem m_MecanumSub;
    private final Telemetry m_dashboardTelemetry;
    private DoubleSupplier m_forwardPower;
    private DoubleSupplier m_strafePower;
    private DoubleSupplier m_rotationPower;


    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;

    Motor m_FL,m_FR,m_BL,m_BR;

    public TeleOpJoystickRobotCentricCMD(MecanumDriveBaseSubsystem mecanumDriveBaseSubsystem,
                                         Telemetry dashboardTelemetry, DoubleSupplier forwardPower,
                                         DoubleSupplier strafePower, DoubleSupplier rotationPower,
                                         Motor FL, Motor FR, Motor BL, Motor BR
    ) {
        m_dashboardTelemetry = dashboardTelemetry;
        m_MecanumSub = mecanumDriveBaseSubsystem;

        m_forwardPower = forwardPower;
        m_strafePower = strafePower;
        m_rotationPower = rotationPower;

        m_FL = FL;
        m_FR = FR;
        m_BL = BL;
        m_BR = BR;


        addRequirements(mecanumDriveBaseSubsystem);
    }
    @Override
    public  void execute(){
        double[] motorSpeeds = new double[4];
        motorSpeeds = m_MecanumSub.setMotorSpeeds(m_forwardPower.getAsDouble(),
                m_strafePower.getAsDouble(),m_rotationPower.getAsDouble());

        m_FL.set(motorSpeeds[0]);
        m_FR.set(motorSpeeds[1]);
        m_BL.set(motorSpeeds[2]);
        m_BR.set(motorSpeeds[3]);


        m_dashboardTelemetry.addData("m_forwardPower (COMMAND)", m_forwardPower);
        m_dashboardTelemetry.addData("m_strafePower (COMMAND)", m_strafePower);
        m_dashboardTelemetry.addData("m_rotationPower (COMMAND)", m_rotationPower);




    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
