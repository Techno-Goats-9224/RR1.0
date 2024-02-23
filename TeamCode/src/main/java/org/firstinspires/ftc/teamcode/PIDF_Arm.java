package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

@Config
@TeleOp
@MotorType(gearing=0.5, ticksPerRev = 3895.9, maxRPM = 43)
public class PIDF_Arm extends OpMode {
    private PIDController controller;

    public static double p = 00.01, i = 0, d = 0.001;
    public static double f = 00.05;

    private double power = 0;
    private int target = 0;

    private int current;
    private final double ticks_in_degrees = 3895.9 / 360;
    private DcMotorEx arm;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        current = arm.getCurrentPosition();
        double pid = controller.calculate(current, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        power = pid + ff;

        arm.setPower(power);

        telemetry.addData("arm position", current);
        telemetry.addData("target", target);
        telemetry.update();
    }
    public int getTarget() {
        return target;
    }
    public void setTarget(int target) {
        this.target = target;
    }

    public double getPower() {
        return power;
    }
    public void setPower(double power) {
        this.power = power;
    }

    public int getCurrent() {
        return current;
    }
    public void setCurrent(int current) {
        this.current = current;
    }
}
