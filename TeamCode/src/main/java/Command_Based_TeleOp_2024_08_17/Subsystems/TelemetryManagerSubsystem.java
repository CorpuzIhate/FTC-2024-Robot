package Command_Based_TeleOp_2024_08_17.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryManagerSubsystem extends SubsystemBase {
    FtcDashboard dashboard;
    Telemetry main_dashboardTelemetry;

    public TelemetryManagerSubsystem(){
        dashboard = FtcDashboard.getInstance();
        main_dashboardTelemetry = dashboard.getTelemetry();

    }
    public Telemetry getTelemetryObject(){
        return main_dashboardTelemetry;
    }

}