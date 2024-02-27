package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@MotorType(gearing=0.5, ticksPerRev = 3895.9, maxRPM = 43)
@DeviceProperties(name = "PIDF_Motor", xmlTag = "PIDF_Motor")
public class PIDF_Arm {
    private PIDController controller;

    public static double p = 00.01, i = 0, d = 0.001;
    public static double f = 00.05;

    private double power = 0;
    private int target = 0;

    private int current;
    private final double ticks_in_degrees = 3895.9 / 360;
    private DcMotorEx arm;
    private Telemetry telemetry;
    private boolean busy;

    public PIDF_Arm(DcMotorEx arm, Telemetry telemetry){
        this.arm = arm;
        this.telemetry = telemetry;
    }
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop(){
        controller.setPID(p, i, d);
        current = arm.getCurrentPosition();
        double pid = controller.calculate(current, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        power = pid + ff;

        power = Range.scale(power, -40, 40, -1, 1);

        arm.setPower(power);

        if(power > 0.05 || power < -0.05){
            busy = true;
        }else{
            busy = false;
        }
        telemetry.addData("arm position", current);
        telemetry.addData("arm target", target);
        telemetry.addData("arm power", power);
        telemetry.addData("is busy?", busy);
    }
    public int getTargetPos() {
        return target;
    }
    public void setTargetPos(int target) {
        this.target = target;
    }

    public double getPower() {
        return power;
    }
    public void setPower(double power) {
        this.power = power;
    }

    public int getCurrentPos() {
        return current;
    }
    public void setCurrentPos(int current) {
        this.current = current;
    }
    public boolean isBusy(){
        return busy;
    }
}
